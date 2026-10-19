"""Live extraction transport/lowering qualification. No native imports or state setters.

Inherited empirical bounds are experimental assumptions, never endpoint-derived
proofs. Original grasp/forecast, actuators, release, HOME and return remain owned
by the continuous runner. A monitors original motion; B admits actual commands.
"""
import runtime_profile as timing
from dataclasses import asdict
import numpy as np
from loaded_common import digest
from loaded_geometry import binding as obstacle_binding
from loaded_clearance import PayloadAdmission, Monitor, ee
from collision_world import pose_matrix
import astra_planner as planner
from preview_errors import CandidateInfeasible
from loaded_tracking import Tracking,POLICY
from slip_supervisor import Supervisor,observation,POLICY as SLIP_POLICY
from earlyhold import EarlyHold,CONTRACT as EARLYHOLD_CONTRACT
from target_contacts import observe_target_contacts

PROFILE=dict(dt=.004,max_velocity=.25,max_acceleration=None,first_sample=True,
             handoff='V36_measured_start_no_bias',hold_steps=100,extra_hold_max=1000,
             extra_hold_stride=50,endpoint_position_m=.005,endpoint_rotation_rad=.04,
             endpoint_joint_rad=.025)

SMOOTH_PROFILE=dict(PROFILE,max_acceleration=.4,clock='segment_quintic_rest_to_rest')

TRACKING_PROFILE=dict(SMOOTH_PROFILE,handoff='retained_drive_bias_with_fixed_measured_reference',tracking=POLICY,slip_supervisor=SLIP_POLICY)

def profile_for(variant):
    return TRACKING_PROFILE if variant=='B_clearance' else PROFILE

def commands(path,profile=PROFILE):
    q=np.asarray(planner.resample_path(path,max_velocity=.25,dt=.004,max_acceleration=profile['max_acceleration']))
    v=np.vstack(((q[1:]-q[:-1])/.004,np.zeros(7)))
    return [dict(q=a.tolist(),v=b.tolist()) for a,b in zip(q,v)]+[dict(q=q[-1].tolist(),v=[0.]*7) for _ in range(1100)]

def fixed_controls(c,arm):
    return {a:[{k:v for k,v in j.items() if a!=arm or i>=7 or k not in ('target','velocity')}
               for i,j in enumerate(rows)] for a,rows in c.items()}

def scoped_contacts(env,target):
    rows=[];total=0
    all_contacts=list(env.scene.get_contacts())
    for contact in all_contacts:
        total+=1;names=[b.entity.get_name() for b in contact.bodies]
        if 'roundtrip_'+str(target) in names or any(np.linalg.norm(p.impulse)>.002 or p.separation<-.0003 for p in contact.points):
            rows.append(dict(bodies=names,points=[dict(position=p.position.tolist(),normal=p.normal.tolist(),separation=float(p.separation),impulse=p.impulse.tolist()) for p in contact.points]))
    return dict(total_contact_pairs=total,scoped_contacts=rows,target_contact_observation=observe_target_contacts(env,target,all_contacts))

class Live:
    def __init__(self,api,arm,target,variant,log,earlyhold=False):
        self.earlyhold_enabled=earlyhold
        self.api=api;self.arm=arm;self.target=target;self.variant=variant;self.log=log
        self.armed=False;self.monitor=None;self.pending=None;self.tracking=None;self.issued=None;self.index=0;self.phase=None
        self.attempts=0;self.completed=0;self.goal_position=None;self.goal_rotation=None;self.rejected=None
    @timing.measured('placement')
    def snapshot(self):
        s=self.api.state();native=pose_matrix(self.api.ee_pose(self.arm))
        fk=ee(self.api.model,self.arm,s['robots'][self.arm]['q'][:7])
        if np.linalg.norm(native[:3,3]-fk[:3,3])>2e-5 or np.linalg.norm(planner.so3_log(native[:3,:3]@fk[:3,:3].T))>1e-4:
            raise CandidateInfeasible('loaded_native_FK_calibration')
        relative=np.linalg.inv(native)@pose_matrix(s['objects'][str(self.target)]['pose'])
        return s,self.api.controls(),PayloadAdmission(self.api.world(s),self.arm,self.target,relative)
    def slots(self,original,used):
        if self.variant=='A_original':return original
        side=-1 if self.arm=='left' else 1
        extra=[[side*x,float(y),.753,1,0,0,0] for x in (.40,.42,.44,.48,.54,.60,.66,.72) for y in (-.6,-.5,-.4,-.3,-.2,-.1,0.,.1,.2)]
        return [list(s) for s in dict.fromkeys(tuple(s) for s in original+extra) if list(s) not in used]
    @timing.measured('placement')
    def prepare(self,above,rotation,up_q,down_path,slot,yaw):
        """Called only AFTER unchanged HOME/endpoint/retreat/resource gates.

        No physics here. Search direct and high outward route, certify also the
        proposed lowering. Actual lowering is freshly rebound after transport.
        """
        if self.variant=='A_original':return True
        self.attempts+=1
        if self.attempts>48:raise CandidateInfeasible('loaded_slot_yaw_budget')
        s,c,a=self.snapshot();start=s['robots'][self.arm]['q'][:7]
        lower=a.certify(down_path)
        self.log.append(dict(kind='lowering_preflight',target=self.target,slot=slot,yaw=float(yaw),certificate=lower))
        if not lower['safe']:return False
        current=ee(a.world.model,self.arm,start);high=max(float(above[2]),float(current[2,3]),1.21)
        routes=[[],[(np.r_[current[:2,3],high],current[:3,:3]),
                     (np.array([above[0],current[1,3],high]),current[:3,:3]),
                     (np.r_[above[:2],high],rotation)]]
        bounds=np.array(a.world.model.bounds)+[.03,-.03]
        for route_index,waypoints in enumerate(routes):
            q=list(start);path=[q];valid=True
            for p,r in waypoints+[(above,rotation)]:
                final=(p is above)
                ik={'success':True,'q':up_q} if final else planner.solve_ik(a.world.model,self.arm,p,r,q,rng=self.api.rng,max_iters=150,restarts=2)
                if not ik['success']:valid=False;break
                plan=planner.plan_joint_path(q,ik['q'],bounds,lambda x:a.world.free(self.arm,x,self.target,True,np.clip(s['robots'][self.arm]['q'][-2:],0,.04).tolist()),rng=self.api.rng,max_nodes=80,resolution=.025)
                if not plan['success']:valid=False;break
                path.extend(plan['path'][1:]);q=ik['q']
            if not valid:continue
            certificate=a.certify(path)
            self.log.append(dict(kind='transport_preflight',target=self.target,slot=slot,yaw=float(yaw),route=route_index,path=path,certificate=certificate))
            if certificate['safe']:
                self.pending=dict(path=path,state=digest(s),controls=digest(c),above=np.asarray(above).tolist(),rotation=np.asarray(rotation).tolist(),up_q=list(up_q))
                return True
        return False
    def bind(self,arm,path,target,loaded,smooth,preserve_bias,acceleration,max_velocity):
        if not self.armed:return
        profile=profile_for(self.variant)
        if (arm!=self.arm or target!=self.target or not loaded or smooth!=(self.variant=='B_clearance') or preserve_bias or max_velocity!=.25 or (smooth and acceleration!=profile['max_acceleration'])):
            raise CandidateInfeasible('loaded_execution_profile_changed')
        s,c,a=self.snapshot()
        if not np.array_equal(path[0],s['robots'][arm]['q'][:7]):raise CandidateInfeasible('loaded_stale_path_start')
        certificate=a.certify(path) if self.variant=='B_clearance' else {'safe':None,'reason':'original_monitor_only'}
        if self.variant=='B_clearance' and not certificate['safe']:raise CandidateInfeasible('loaded_final_command_clearance:'+str(certificate))
        self.expected=commands(path,profile);self.index=0;self.monitor=Monitor(a);self.fixed=fixed_controls(c,arm)
        self.tracking=Tracking(a,self.expected,s,c) if self.variant=='B_clearance' else None
        self.issued=None;self.rejected=None
        self.slip=Supervisor(arm,target,a.env) if self.tracking is not None else None
        self.earlyhold=EarlyHold(self.earlyhold_enabled,self.phase)
        self.slip_packet=None;self.previous_reference_v=[0.]*7
        from earlyhold_integration import contract,ENABLED
        if self.earlyhold_enabled is not ENABLED:raise CandidateInfeasible("earlyhold_arm_setting")
        binding=dict(native_integration=contract(),state=digest(s),controls=digest(c),path=np.asarray(path).tolist(),commands_sha256=digest(self.expected),
                     earlyhold=dict(EARLYHOLD_CONTRACT,enabled=self.earlyhold_enabled),relative=a.relative.tolist(),envelope=asdict(a.env),profile=profile,geometry=digest(a.world.meshes),
                     goal_position=self.goal_position,goal_rotation=self.goal_rotation,**obstacle_binding(a.obstacles))
        self.api.event('loaded_live_admission',object_id=target,arm=arm,loaded_phase=self.phase,binding_sha256=digest(binding))
        self.log.append(dict(kind='live_admission',variant=self.variant,arm=arm,target=target,phase=self.phase,certificate=certificate,binding=binding,binding_sha256=digest(binding),commands=self.expected,state=s,controls=c,native_ee=pose_matrix(self.api.ee_pose(arm)).tolist()))
        # No native stepping between binding and this validation or first command.
        if digest(self.api.state())!=binding['state'] or digest(self.api.controls())!=binding['controls']:
            raise CandidateInfeasible('loaded_stale_live_binding')
        metrics=self.monitor.measure(s,pose_matrix(self.api.ee_pose(arm)),self.expected[0]['q'])
        if metrics['violations']:raise CandidateInfeasible('loaded_initial_empirical_bound:'+str(metrics['violations']))
        if self.slip is not None:self.slip_packet=observation(0,s,pose_matrix(self.api.ee_pose(arm)),metrics,arm,target)
    @timing.measured('live_before')
    def before_tick(self):
        if self.tracking is None:return
        if self.issued is not None:raise RuntimeError('loaded_unobserved_issued_command')
        s=self.api.state();c=self.api.controls()
        if fixed_controls(c,self.arm)!=self.fixed:raise CandidateInfeasible('loaded_actuator_or_idle_control_changed')
        self.tracking.last_trials=[]
        self.earlyhold.last=None
        try:
            native=pose_matrix(self.api.ee_pose(self.arm))
            factors,slip=self.slip.decide(self.slip_packet,digest(s),digest(native.tolist()),self.index,self.previous_reference_v)
            decision=self.earlyhold.select(self.tracking,self.slip,s,factors,slip)
        except CandidateInfeasible as exc:
            self.rejected=dict(pre_state=s,pre_controls=c,index=self.index,reason=str(exc),trials=getattr(exc,'proposal_trials',self.tracking.last_trials),slip=self.slip.last_witness,earlyhold=self.earlyhold.last)
            raise
        self.issued=dict(decision=decision,pre_state_sha256=digest(s),pre_controls_sha256=digest(c))
        self.api.command(self.arm,decision['q'],decision['v'])
        applied=self.api.controls()
        self.issued.update(applied_controls=applied,applied_controls_sha256=digest(applied))
        if fixed_controls(applied,self.arm)!=self.fixed:raise CandidateInfeasible('loaded_issue_fixed_control_changed')
        for field,key in (('target','q'),('velocity','v')):
            if not np.allclose([j[field][0] for j in applied[self.arm][:7]],decision[key],rtol=0,atol=2e-7):
                raise CandidateInfeasible('loaded_issue_control_mismatch:'+field)
    @timing.measured('live_pre')
    def pre_step(self,qf):
        if self.issued is None:raise RuntimeError('loaded_prestep_without_command')
        if digest(self.api.state())!=self.issued['pre_state_sha256'] or self.api.controls()!=self.issued['applied_controls']:
            raise RuntimeError('loaded_command_to_step_order_changed')
        if set(qf)!={'left','right'} or any(len(qf[a])!=len(self.api.state()['robots'][a]['q']) or not np.isfinite(qf[a]).all() for a in qf):
            raise RuntimeError('loaded_qf_readback')
        self.issued['pre_step_qf']=qf
        self.issued['ordering']='command_readback_then_passive_qf_then_readback_then_scene_step'
    def run_tracking(self,tick):
        # First tick preserves previous held drive target and prior velocity.
        # Every following command is selected BEFORE qf and native scene.step.
        while self.tracking.index==0 or not self.tracking.at_end:tick()
        for _ in range(100):tick()
        if self.slip.mode=='hold':raise CandidateInfeasible('loaded_slip_endpoint_hold_unresolved')
    @timing.measured('live_observe')
    def observe(self,target,held):
        if self.monitor is None:return
        if target!=self.target or not held:raise CandidateInfeasible('loaded_scope_changed')
        if self.tracking is None and self.index>=len(self.expected):raise CandidateInfeasible('loaded_command_time_budget')
        if self.tracking is not None and (self.issued is None or 'pre_step_qf' not in self.issued):raise RuntimeError('loaded_tick_without_issue')
        command=self.expected[self.index] if self.tracking is None else {k:self.issued['decision'][k] for k in ('q','v')}
        reference=command['q'] if self.tracking is None else self.issued['decision']['reference_q']
        c=self.api.controls();s=self.api.state()
        if fixed_controls(c,self.arm)!=self.fixed:raise CandidateInfeasible('loaded_actuator_or_idle_control_changed')
        for field,key in (('target','q'),('velocity','v')):
            actual=[j[field][0] for j in c[self.arm][:7]]
            if not np.allclose(actual,command[key],rtol=0,atol=2e-7):raise CandidateInfeasible('loaded_command_profile_mismatch:'+field)
        native=pose_matrix(self.api.ee_pose(self.arm));metrics=self.monitor.measure(s,native,reference)
        self.log.append(dict(kind='live_tick',variant=self.variant,arm=self.arm,target=target,phase=self.phase,index=self.index,command=command,state=s,native_ee=native.tolist(),metrics=metrics,controls_sha256=digest(c),controls=c,contacts=self.api.contacts(),**({} if self.tracking is None else dict(tracking=self.issued))))
        self.index+=1
        if self.tracking is not None:
            self.previous_reference_v=self.issued['decision']['reference_v']
            self.earlyhold.accept(self.issued['decision'])
            self.tracking.commit(self.issued['decision']);self.issued=None
            self.slip_packet=observation(self.index,s,native,metrics,self.arm,target)
        if metrics['violations']:raise CandidateInfeasible('empirical_bound_violation:'+','.join(metrics['violations']))
    @timing.measured('placement')
    def move(self,phase,p,r,joint_goal,original):
        self.phase=phase;self.armed=True;self.index=0;completed=False;failure=None
        self.goal_position=np.asarray(p).tolist();self.goal_rotation=np.asarray(r).tolist()
        try:
            if self.variant=='A_original' or phase=='lowering':
                completed=original(self.arm,p,r,self.target,self.api.relative(),joint_goal=joint_goal,allow_plan_failure=phase=='transport')
                return completed
            s=self.api.state();c=self.api.controls();b=self.pending
            if b is None or digest(s)!=b['state'] or digest(c)!=b['controls'] or np.asarray(p).tolist()!=b['above'] or np.asarray(r).tolist()!=b['rotation'] or list(joint_goal)!=b['up_q']:
                raise CandidateInfeasible('loaded_stale_transport_route')
            self.api.execute(self.arm,b['path'],self.target,True)
            for extra in range(0,1001,50):
                actual=pose_matrix(self.api.ee_pose(self.arm));pe=float(np.linalg.norm(actual[:3,3]-p));re=float(np.linalg.norm(planner.so3_log(np.asarray(r)@actual[:3,:3].T)))
                self.api.event('physical_tracking',target=self.target,extra_hold_steps=extra,position_error_m=pe,rotation_error_rad=re,loaded_live=True)
                if pe<=.005 and re<=.04 and (self.tracking is None or self.slip.mode!='hold'):
                    completed=True;return True
                if extra<1000:
                    for _ in range(50):self.api.tick(self.target,True)
            raise CandidateInfeasible('physical_ee_pose_not_settled')
        except Exception as exc:
            failure=type(exc).__name__+':'+str(exc);raise
        finally:
            self.log.append(dict(kind='live_motion_terminal',variant=self.variant,arm=self.arm,target=self.target,phase=phase,ticks=self.index,completed=bool(completed),failure=failure,native_ee=pose_matrix(self.api.ee_pose(self.arm)).tolist(),state=self.api.state(),controls=self.api.controls(),contacts=self.api.contacts() if hasattr(self.api,'contacts') else None,**({} if self.tracking is None else dict(tracking_terminal=dict(cursor=self.tracking.cursor,index=self.tracking.index,budget=self.tracking.budget,pending_issue=self.issued,rejected_command=self.rejected)))))
            self.api.event('loaded_live_terminal',object_id=self.target,arm=self.arm,loaded_phase=phase,ticks=self.index,completed=bool(completed),failure=failure)
            self.armed=False;self.monitor=None;self.pending=None;self.tracking=None;self.issued=None
