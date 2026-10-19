"""Experimental bounded physical capture. No scene/state/velocity restoration.

Native adapter owns readbacks, identity-qualified contacts and geometric guards.
Old settling/regulation is retained for uninterrupted bilateral contact. A lost
contact interrupts that hold, authorizes a small unilateral search only with a
stable contacting opposite finger, then requires a new complete settling dwell.
"""
import copy
import math
import numpy as np
from provenance import digest
from velocity_relief import goals as velocity_relief_goals, POLICY as VELOCITY_RELIEF_POLICY
from preview_errors import CandidateInfeasible
from closure_hold import settle_closure
from closure_regulation import Regulator, targets as relief_targets, wire_controls

DT=.004
SLEW=.01*DT
PRELOAD=.00025
TOTAL_TICKS=3002               # old 2002 sweep + 1000 hold; recovery borrows this
SWEEP_TICKS=2002
HOLD_TICKS=1000
RECOVERY_TICKS=100             # <=0.4 s per attempt, never an unbounded retry
RECOVERY_ROUNDS=2
RECOVERY_DISTANCE=.001        # <=1 mm cumulative inward target motion / finger
OTHER_PRELOAD=.0005           # feedback trigger, NOT force or final acceptance
FEEDBACK_TICKS=200            # 0.8 s aggregate, borrowed from unchanged total
DRIFT_M=.002
DRIFT_RAD=math.radians(2.)


def need(ok, why):
    if not ok: raise CandidateInfeasible('capture_'+why)


def raw_touch(points):
    """Every point has already been identity-qualified by the native adapter."""
    found=[]
    for finger in points:
        for point in finger:
            need(len(point['impulse'])==3 and all(math.isfinite(x) for x in
                 [point['separation'],*point['impulse']]),'nonfinite_contact')
            need(point['separation']>=-.0003,'target_penetration')
        found.append(any(-.0003<=p['separation']<=.0001 and
                         math.sqrt(sum(x*x for x in p['impulse']))>1e-6 for p in finger))
    need(len(found)==2,'contact_shape')
    return found


def drift(pose, initial):
    need(len(pose)==len(initial)==7 and all(math.isfinite(x) for x in [*pose,*initial]),'object_pose')
    a=np.asarray(pose[3:],float);b=np.asarray(initial[3:],float)
    need(np.linalg.norm(a)>0 and np.linalg.norm(b)>0,'object_quaternion')
    angle=2*math.acos(min(1.,abs(float(np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b))))))
    return float(np.linalg.norm(np.asarray(pose[:3])-initial[:3])),angle


def emitted(previous, goals):
    """Native-representable step inside both the desired and slew intervals."""
    result=[]
    for old,goal in zip(previous,goals):
        need(math.isfinite(old) and math.isfinite(goal) and 0<=old<=.04 and 0<=goal<=.04,'target_travel')
        goal=float(np.float32(goal))  # same native final preload as old caller
        requested=old+max(-SLEW,min(SLEW,goal-old))
        value=np.float32(requested)
        if goal<old and float(value)<requested:value=np.nextafter(value,np.float32(np.inf))
        if goal>old and float(value)>requested:value=np.nextafter(value,np.float32(-np.inf))
        value=float(value)
        need(abs(value-old)<=SLEW+1e-15 and min(old,goal)<=value<=max(old,goal),'native_slew')
        result.append(value)
    return result


class LostContact(Exception): pass
class TargetMotion(Exception): pass

def probe(value):
    """Retain nonfinite readbacks explicitly without nonstandard JSON numbers."""
    if isinstance(value,dict):return {k:probe(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [probe(v) for v in value]
    if isinstance(value,(float,np.floating)) and not math.isfinite(value):return {"nonfinite":str(value)}
    return value

def motion(row):
    return (max(map(abs,row["finger_velocity"]))>.01 or
            np.linalg.norm(row["object_linear_velocity"])>.03 or
            np.linalg.norm(row["object_angular_velocity"])>.15)


class Capture:
    def __init__(self,api,source):
        self.api=api;self.source=dict(source,schema='capture-feedback/018-followup001',
            feedback_policy=dict(VELOCITY_RELIEF_POLICY),
            limits=dict(total_ticks=TOTAL_TICKS,sweep_ticks=SWEEP_TICKS,hold_ticks=HOLD_TICKS,
                        recovery_ticks=RECOVERY_TICKS,recovery_rounds=RECOVERY_ROUNDS,
                        recovery_distance_m=RECOVERY_DISTANCE,feedback_ticks=FEEDBACK_TICKS,dt=DT,slew_m=SLEW,
                        drift_m=DRIFT_M,drift_rad=DRIFT_RAD))
        self.ticks=0;self.sweep=0;self.hold=0;self.rounds=0;self.feedback_ticks=0
        self.recovery_distance=[0.,0.];self.latches=[None,None];self.first=[None,None]
        self.baseline=api.controls();self.initial=api.observe();self.last=self.initial;self.last_tick=0;self.last_validated=False
        self.attempt=self.initial;self.attempt_tick=0;self.attempt_controls=self.baseline
        self.slots=source['slots'];self.initial_pose=self.initial['object_pose']
        self.source.update(initial=probe(self.initial),controls=probe(wire_controls(self.baseline)))
        self.source['sha256']=digest(self.source);self.chain=self.source['sha256'];self.serial=0
        self.api.emit('capture_begin',source=self.source)

    def record(self,kind,**values):
        row=dict(source_sha256=self.source['sha256'],previous_sha256=self.chain,seq=self.serial,
                 ticks=self.ticks,**values)
        row['sha256']=digest(row);self.chain=row['sha256'];self.serial+=1
        self.api.emit(kind,record=row)

    def command(self):return [float(self.baseline[i][4][0]) for i in self.slots]

    def check(self,row):
        need(abs(self.api.dt-DT)<=1e-9,'timestep')
        need(self.api.controls()==self.baseline,'actuator_tampering')
        need(row['held_id'] is None and row['drive_active'] is False,'unloaded')
        numbers=[*row['finger_position'],*row['finger_velocity'],row['position_error_m'],row['rotation_error_rad'],
                 *row['object_linear_velocity'],*row['object_angular_velocity']]
        need(len(row['finger_position'])==len(row['finger_velocity'])==2 and all(math.isfinite(v) for v in numbers),'nonfinite_readback')
        need(all(0<=v<=.04 for v in row['finger_position']),'measured_travel')
        need(row['position_error_m']<=.005 and row['rotation_error_rad']<=.04,'grasp_pose')
        pe,re=drift(row['object_pose'],self.initial_pose)
        need(pe<=DRIFT_M and re<=DRIFT_RAD,'target_drift')
        touch=raw_touch(row['points'])
        need(row['touching']==touch,'contact_readback')
        for i in self.slots:
            need(self.baseline[i][:4]==(1000.,200.,4.,'force') and self.baseline[i][5]==(0.,),'drive_contract')
        self.api.guard()  # raises on the unchanged native forbidden-contact rule
        return touch

    def sense(self):
        row=self.api.observe();self.attempt=row;self.attempt_tick=self.ticks;self.attempt_controls=self.api.controls()
        self.check(row);self.last=row;self.last_tick=self.ticks;self.last_validated=True
        return row

    def latch(self,index,row,renew=False):
        value=max(0.,min(.04,row['finger_position'][index]-PRELOAD))
        self.latches[index]=value
        if self.first[index] is None:self.first[index]=value
        self.record('capture_latch',finger=index,renewal=renew,target=value,observation=row)

    def feedback_needed(self,row):
        old=self.command()
        return motion(row) or any(t and q-u>OTHER_PRELOAD for t,q,u in
                                  zip(row['touching'],row['finger_position'],old))

    def feedback(self,row):
        """No inward drive motion while transient feedback is active.

        A pre-existing dynamic lag is relieved at the same 40um/tick limit.
        The opposite finger is held unless it independently needs relief.
        Renewed contact is still handled exclusively by recover/latch.
        """
        need(self.feedback_needed(row),'feedback_without_trigger')
        need(self.feedback_ticks<FEEDBACK_TICKS,'feedback_budget')
        old=self.command();goals=list(old)
        object_fast=(np.linalg.norm(row['object_linear_velocity'])>.03 or
                     np.linalg.norm(row['object_angular_velocity'])>.15)
        for i in range(2):
            q=row['finger_position'][i];touch=row['touching'][i]
            compressed=touch and q-old[i]>OTHER_PRELOAD
            if compressed and self.latches[i] is not None:
                target=max(self.latches[i],max(0.,q-PRELOAD))
                self.record('capture_rebase',finger=i,previous_latch=self.latches[i],target=target,observation=row)
                self.latches[i]=target
            if compressed or row['finger_velocity'][i]<-.01 or (object_fast and touch):
                goals[i]=max(old[i],max(0.,q-PRELOAD))
        goals=velocity_relief_goals(row,old,goals)
        self.step(goals,'feedback')

    def step(self,goals=None,reason='hold',active=None,regulator=None):
        need(self.ticks<TOTAL_TICKS,'total_budget')
        before=self.sense();old=self.command()
        command=old if goals is None else emitted(old,goals)
        if reason=='hold':need(self.hold<HOLD_TICKS,'hold_budget')
        if reason=='feedback':need(self.feedback_ticks<FEEDBACK_TICKS and self.feedback_needed(before),'feedback_budget_or_trigger')
        elif reason!='hold':need(not self.feedback_needed(before),'feedback_required')
        if reason=='sweep':need(self.sweep<SWEEP_TICKS,'sweep_budget')
        if reason=='reacquire':
            need(active in (0,1) and before['touching'][1-active] and not before['touching'][active],'reacquire_contact')
            need(command[1-active]==old[1-active] and command[active]<=old[active],'reacquire_direction')
            delta=old[active]-command[active]
            need(self.recovery_distance[active]+delta<=RECOVERY_DISTANCE+1e-15,'recovery_distance')
        certificate=self.api.geometry(command);need(certificate,'command_geometry')
        expected=list(self.baseline)
        for i,value in zip(self.slots,command):expected[i]=(*expected[i][:4],(value,),(0.,))
        self.record('capture_command',reason=reason,active=active,round=self.rounds,
                    before=before,previous_targets=old,targets=command,
                    controls_sha256=digest(wire_controls(tuple(expected))),geometry=certificate)
        if regulator is None:
            if command!=old:self.api.issue(command)
        else:
            regulator()
        need(self.api.controls()==tuple(expected),'command_readback')
        self.baseline=tuple(expected)
        if reason=='reacquire':self.recovery_distance[active]+=delta
        self.record('capture_readback',operation='command',controls_sha256=digest(wire_controls(self.baseline)))
        self.api.guard()
        self.record('capture_tick_started')
        self.ticks+=1  # count attempted guarded ticks even if native tick raises
        if reason=='feedback':self.feedback_ticks+=1
        if reason=='sweep':self.sweep+=1
        if reason=='hold':self.hold+=1
        self.api.tick()
        after=self.sense()
        self.record('capture_observation',observation=after,controls_sha256=digest(wire_controls(self.baseline)))

    def recover(self,active):
        row=self.sense()
        need(self.rounds<RECOVERY_ROUNDS,'recovery_rounds')
        need(self.latches[active] is not None,'recovery_without_latch')
        self.record('capture_loss',finger=active,stale_latch=self.latches[active],observation=row)
        need(row['touching']==[i!=active for i in range(2)],'both_contacts_lost')
        self.rounds+=1
        fixed=self.command()[1-active]
        self.record('capture_reacquire',finger=active,round=self.rounds,held_target=fixed)
        for _ in range(RECOVERY_TICKS):
            row=self.sense()
            need(row['touching'][1-active],'support_contact_lost')
            # Feedback may open the supporting drive; never close it further.
            need(self.command()[1-active]>=fixed,'held_target_compressed')
            if row['touching'][active]:
                self.latch(active,row,renew=True)
                return
            if self.feedback_needed(row):
                self.feedback(row);continue
            old=self.command();goals=list(old)
            # Do not accumulate more than original spring preload while native
            # position lags a command; obstacle guards remain independently live.
            floor=row['finger_position'][active]-PRELOAD
            native_floor=np.float32(floor)
            if float(native_floor)<floor:native_floor=np.nextafter(native_floor,np.float32(np.inf))
            goals[active]=max(0.,old[active]-SLEW,float(native_floor))
            goals[active]=min(old[active],goals[active])
            self.step(goals,'reacquire',active)
        raise CandidateInfeasible('capture_recovery_budget')

    def sweep_until_reached(self):
        reached_before=False
        while True:
            row=self.sense();touch=row['touching']
            missing=[i for i in range(2) if self.latches[i] is not None and not touch[i]]
            if missing:
                # No blind re-close of both after a transient first contact.
                self.recover(missing[0]);reached_before=False;continue
            for i in range(2):
                if touch[i] and self.latches[i] is None:self.latch(i,row)
            goals=[v if v is not None else 0. for v in self.latches]
            if self.feedback_needed(row):
                self.feedback(row);reached_before=False;continue
            command=emitted(self.command(),goals)
            reached=all(abs(a-b)<=2e-9 for a,b in zip(command,goals))
            self.step(goals,'sweep')
            if reached and reached_before:
                if all(self.last['touching']):return
                need(any(v is not None for v in self.latches),'no_bilateral_capture')
            reached_before=reached

    def settle(self):
        while True:
            # Every resumed hold binds the current controls and must earn its
            # own original 100-tick minimum and 25-tick consecutive dwell.
            row=self.sense()
            regulator=Regulator(self.api.joints,self.slots,self.api.controls,self.api.legacy_emit,
                                self.api.closure_source())
            def measure():
                r=self.sense()
                if np.linalg.norm(r['object_linear_velocity'])>.03 or np.linalg.norm(r['object_angular_velocity'])>.15:
                    self.record('capture_motion_interrupted',observation=r)
                    raise TargetMotion()
                if not all(r['touching']):
                    self.record('capture_hold_interrupted',observation=r)
                    raise LostContact()
                return dict(finger_velocity=r['finger_velocity'],finger_position=r['finger_position'],
                            bilateral_contact=True,position_error_m=r['position_error_m'],rotation_error_rad=r['rotation_error_rad'])
            # Existing Regulator runs before settle_closure's tick. Validate its
            # opening-only correction before it writes, then absorb readback.
            def regulate(r,elapsed):
                old=self.command()
                desired=relief_targets(old,r['finger_position']) if elapsed<100 or max(map(abs,r['finger_velocity']))>.01 else old
                if desired==old:return False
                before=self.sense();certificate=self.api.geometry(desired);need(certificate,'relief_geometry')
                expected=list(self.baseline)
                for i,v in zip(self.slots,desired):expected[i]=(*expected[i][:4],(v,),(0.,))
                self.record('capture_relief',before=before,previous_targets=old,targets=desired,
                            controls_sha256=digest(wire_controls(tuple(expected))),geometry=certificate,elapsed_ticks=elapsed)
                changed=regulator(r,elapsed)
                need(self.api.controls()==tuple(expected),'relief_readback');self.baseline=tuple(expected)
                self.record('capture_readback',operation='relief',controls_sha256=digest(wire_controls(self.baseline)))
                return changed
            try:
                result=settle_closure(tick=lambda:self.step(),measure=measure,guard=self.api.guard,
                    controls=self.api.controls,emit=self.api.legacy_emit,dt=self.api.dt,regulate=regulate)
                return result
            except TargetMotion:
                while self.feedback_needed(self.sense()):self.feedback(self.last)
                self.sweep_until_reached()
            except LostContact:
                missing=[i for i,v in enumerate(self.last['touching']) if not v]
                self.recover(missing[0]);self.sweep_until_reached()

    def run(self):
        status='failed';failure=None
        try:
            self.check(self.initial);self.last_validated=True;self.sweep_until_reached();result=self.settle()
            need(all(self.sense()['touching']),'terminal_contact');status='accepted'
            return dict(first_latches=self.first,current_latches=self.latches,settle=result,ticks=self.ticks)
        except Exception as exc:
            failure=type(exc).__name__+':'+str(exc);raise
        finally:
            try:
                self.record('capture_terminal',status=status,failure=failure,sweep_ticks=self.sweep,hold_ticks=self.hold,
                            recovery_rounds=self.rounds,recovery_distance=self.recovery_distance,
                            last_observation=probe(self.last),last_observation_tick=self.last_tick,last_observation_validated=self.last_validated,feedback_ticks=self.feedback_ticks,
                            attempt_observation=probe(self.attempt),attempt_observation_tick=self.attempt_tick,
                            attempt_controls=probe(wire_controls(self.attempt_controls)),
                            attempt_is_not_necessarily_violating=True,
                            recovery_requires_fresh_planning=status!='accepted',native_qualified=False)
            except Exception:
                if failure is None:raise  # missing terminal can never be audited accepted


def run(api,source):return Capture(api,source).run()
