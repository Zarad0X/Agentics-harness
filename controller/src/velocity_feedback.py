"""v16 bounded velocity-target feedback. No position/drive-property changes.

D*k changes effective restoring action (K=1000, D=200 => +200/+1200).
These are declared experiments, not identified dynamics or production gains.
"""
import copy
import hashlib
import numpy as np
from provenance import canonical, digest
from preview_errors import CandidateInfeasible
from return_clearance import DT

SETTINGS = {'off': (0., 0.), 'k1': (1., .01), 'k6': (6., .04)}
SCOPES = ('retreat', 'all_unloaded', 'calibration')
V_MAX = .1
SLEW = .4 * DT
READBACK_ATOL = 1e-8  # target storage is float32 in the inspected setter


def reject(reason):
    raise CandidateInfeasible('velocity_feedback_' + reason)


def vector(value, n=7):
    a=np.asarray(value, dtype=float)
    if a.shape!=(n,) or not np.isfinite(a).all():reject('nonfinite_or_shape')
    return a


def targets(controls, arm):
    try:
        names=[j['name'] for j in controls[arm]]
        if len(names)!=len(set(names)):reject('duplicate_joint')
        joints={j['name']:j for j in controls[arm]}
        q=[vector(joints[f'panda_joint{i}']['target'],1)[0] for i in range(1,8)]
        v=[vector(joints[f'panda_joint{i}']['velocity_target'],1)[0] for i in range(1,8)]
        return vector(q),vector(v)
    except (KeyError,TypeError):reject('missing_readback')


def retained(controls,arm):
    c=copy.deepcopy(controls)
    for j in c[arm]:
        if j['name'] in [f'panda_joint{i}' for i in range(1,8)]:
            j.pop('target',None);j.pop('velocity_target',None)
    return digest(c)


def bounded(nominal, error, previous, setting, withdrawn=False):
    if setting not in SETTINGS:reject('setting')
    nominal,error,previous=map(vector,(nominal,error,previous))
    if np.max(np.abs(previous))>V_MAX:reject('retained_velocity_bound')
    if np.max(np.abs(nominal))>V_MAX+1e-12:reject('nominal_velocity_bound')
    k,cap=SETTINGS[setting]
    with np.errstate(over='ignore',invalid='ignore'):raw=k*error
    if not np.isfinite(raw).all():reject('correction_overflow')
    correction=np.zeros(7) if withdrawn else np.clip(raw,-cap,cap)
    requested=nominal+correction
    limited=np.clip(requested,-V_MAX,V_MAX)
    issued=previous+np.clip(limited-previous,-SLEW,SLEW)
    # Prove bounds also after the public setter's float32 storage, without
    # assuming previous target zero. Round toward previous when necessary.
    stored=issued.astype(np.float32)
    for i in range(7):
        while abs(float(stored[i]))>V_MAX or abs(float(stored[i])-previous[i])>SLEW:
            direction=0. if abs(float(stored[i]))>V_MAX else previous[i]
            stored[i]=np.nextafter(stored[i],np.float32(direction))
    issued=stored.astype(float)
    if np.max(np.abs(issued))>V_MAX or np.max(np.abs(issued-previous))>SLEW:reject('online_bound')
    return issued,dict(nominal_qdot=nominal.tolist(),requested_correction=correction.tolist(),
        requested_total_qdot=requested.tolist(),issued_qdot=issued.tolist(),previous_readback_qdot=previous.tolist(),
        total_saturated=bool(np.any(requested!=limited)),correction_saturated=bool(np.any(np.abs(raw)>cap)) if not withdrawn else False,
        slew_limited=bool(np.any(np.abs(limited-previous)>SLEW)),withdrawn=withdrawn,
        max_issued_slew=float(np.max(np.abs(issued-previous))))


class Recovery:
    def __init__(self):self.withdrawn=False;self.streak=0;self.first_ready=None;self.first_qualified=None
    def observe(self,ready,zero,tick):
        if ready:
            self.withdrawn=True
            if self.first_ready is None:self.first_ready=tick
        self.streak=self.streak+1 if ready and zero and self.withdrawn else 0
        if self.streak>=25 and self.first_qualified is None:self.first_qualified=tick
        return self.streak>=25


class FeedbackAPI:
    """ON-only command/readback proof, synchronized bounded numeric telemetry.

    No guard replacement; no extra geometry. Tick errors are observed then
    propagated, and finish never commands recovery. 512 expanded packets total.
    """
    def __init__(self,api,arm,target,phase,setting,telemetry=True):
        if phase not in ('retreat','home','handoff') or arm not in ('left','right'):reject('phase_or_arm')
        if setting not in SETTINGS:reject('setting')
        self.api=api;self.arm=arm;self.target=target;self.phase=phase;self.setting=setting
        self.velocity_feedback_controller=self;self.recovery=None;self.last_recovery=None;self.last=None;self.pending=None
        self.start=self.clock();self.last_step=self.start;self.total=0;self.attempts=0
        self.retained=retained(api.controls(),arm);q,v=targets(api.controls(),arm)
        vector(api.state()['robots'][arm]['q'],9)
        if np.max(np.abs(v))>V_MAX:reject('retained_velocity_bound')
        self.initial_velocity=v.tolist();self.expected=(q,v);self.samples=[];self.window=[];self.count=0
        self.hash=hashlib.sha256();self.telemetry_on=telemetry;self.hold_ticks=0;self.substantive=False
        self.saturation_count=0;self.slew_count=0
    def __getattr__(self,name):return getattr(self.api,name)
    def clock(self):
        rb=self.api.telemetry();step=rb['scene_step_count'];dt=rb['scene_timestep_s']
        if type(step) is not int or not np.isfinite(dt) or abs(dt-DT)>1e-9:reject('clock')
        return step
    def read(self):
        s=self.api.state();rb=self.api.telemetry();c=self.api.controls()
        q=vector(s['robots'][self.arm]['q'],9);native=vector(s['robots'][self.arm]['qdot'],9)
        tq,tv=targets(c,self.arm)
        if retained(c,self.arm)!=self.retained:reject('retained_controls_changed')
        if np.max(np.abs(tv))>V_MAX:reject('readback_bound')
        if not np.allclose(tq,self.expected[0],atol=2e-7,rtol=0) or not np.allclose(tv,self.expected[1],atol=READBACK_ATOL,rtol=0):reject('target_readback_mismatch')
        external=rb.get('applied_passive_qf')
        if external:
            for value in external.values():vector(value,9)
        return dict(frame=rb['scene_step_count'],dt=rb['scene_timestep_s'],q=q.tolist(),native_qvel=native.tolist(),
            readback_q=tq.tolist(),readback_qdot=tv.tolist(),external_qf=external,
            drive_force=dict(available=False,reason='No identified public measured drive-force getter; external qf is not drive torque.'))
    def begin_recovery(self):
        if self.recovery is None:self.recovery=Recovery()
    def observe_ready(self,ready,completed=False):
        self.begin_recovery();_,v=targets(self.api.controls(),self.arm)
        zero=bool(np.array_equal(v,np.zeros(7)))
        if completed:return self.recovery.observe(ready,zero,self.total)
        if ready:
            self.recovery.withdrawn=True
            if self.recovery.first_ready is None:self.recovery.first_ready=self.total
        return False
    def zero(self):return bool(np.array_equal(targets(self.api.controls(),self.arm)[1],np.zeros(7)))
    def resume(self):
        if not self.zero():reject('resume_nonzero')
        self.last_recovery=vars(self.recovery).copy() if self.recovery is not None else None
        self.recovery=None
    def command(self,arm,q,v):
        if arm!=self.arm:reject('other_arm_command')
        q=vector(q);v=vector(v)
        if self.clock()!=self.last_step or self.pending is not None:reject('command_clock')
        if self.total>=36000 or self.attempts>=36000:reject('total_cap')
        pre=self.read();actual=vector(pre['q'],9)[:7];previous=vector(pre['readback_qdot'])
        if self.recovery is not None and np.any(v):reject('nonzero_recovery_path_velocity')
        issued,proof=bounded(v,q-actual,previous,self.setting,self.recovery is not None and self.recovery.withdrawn)
        self.attempts+=1;self.api.command(arm,q,issued)
        self.expected=(q.copy(),issued.copy());post=self.read()
        if np.max(np.abs(vector(post['readback_qdot'])-previous))>SLEW:reject('stored_slew_bound')
        self.pending=dict(pre=pre,just_issued_q=q.tolist(),proof=proof,write_readback=post,
                          mode='recovery' if self.recovery else 'motion')
        self.last=proof;self.saturation_count+=proof['total_saturated'] or proof['correction_saturated'];self.slew_count+=proof['slew_limited']
    def tick(self,*args):
        if self.pending is None:reject('tick_without_command')
        row=self.pending;before=self.last_step
        try:return self.api.tick(*args)
        finally:
            now=self.clock();self.total+=max(0,now-before);self.last_step=now
            row['post']=self.read();row['completed_ticks']=now-before
            row['single_tick_fd']=((vector(row['post']['q'],9)-vector(row['pre']['q'],9))/DT).tolist() if now-before==1 else None
            if row['mode']=='recovery':self.hold_ticks+=1;self.substantive=True
            self.count+=1;self.hash.update(canonical(row));self.window=(self.window+[row])[-16:]
            # Short segment before the first hold, then first 200 recovery ticks.
            if self.telemetry_on:
                if not self.substantive:self.samples=(self.samples+[row])[-300:]
                elif row['mode']=='recovery' and self.hold_ticks<=200 and len(self.samples)<500:self.samples.append(row)
            self.pending=None
            if now-before!=1:reject('tick_discontinuity')
    def finish(self,include_packets=True):
        try:final_targets=targets(self.api.controls(),self.arm)[1].tolist()
        except Exception as exc:final_targets=dict(unavailable=type(exc).__name__+':'+str(exc))
        self.api.event('velocity_feedback_receipt',schema='velocity-feedback/16',arm=self.arm,object_id=self.target,
            motion_phase=self.phase,setting=self.setting,initial_retained_velocity=self.initial_velocity,
            effective_restoring_action_changed=self.setting!='off',drive_properties_changed=False,
            completed_ticks=self.total,command_attempts=self.attempts,packet_count=self.count,
            numeric_packets=self.samples if include_packets else [],last_packets=self.window[-12:] if include_packets else [],all_numeric_sha256=self.hash.hexdigest(),
            saturation_count=self.saturation_count,slew_count=self.slew_count,
            last_issued_proof=self.last,actual_final_targets=final_targets,formal_credit=False)


def enabled(api,phase):
    setting=getattr(api,'velocity_feedback','off');scope=getattr(api,'feedback_scope','retreat')
    if setting not in SETTINGS or scope not in SCOPES:reject('configuration')
    if setting=='off' or scope=='calibration':return False
    if phase not in ('retreat','home','handoff'):reject('loaded_phase')
    if not getattr(api,'vertex_governor',False) or getattr(api,'retreat_time_scale',1)!=1:reject('compatibility')
    return scope=='all_unloaded' or phase=='retreat'
