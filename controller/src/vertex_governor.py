"""v13 bounded vertex dwell. Original dense commands and clocks are immutable.

No equilibrium or C2 claim. OFF only emits compact prefix receipts; no extra
state reads, command, guard or tick. The first checkpoint is AFTER command 0's
completed tick; FD starts at the pre-command measurement, never at native qvel.
"""
from dataclasses import dataclass, asdict
import hashlib
import time
import numpy as np
import astra_planner as planner
from provenance import canonical, digest
from preview_errors import CandidateInfeasible
from return_clearance import DT, MAX_VELOCITY, MAX_ACCELERATION, ReturnClearance


@dataclass(frozen=True)
class Policy:
    whole_link_m: float = .003
    arm_error_rad: float = .006
    arm_fd_rad_s: float = .005
    streak_ticks: int = 25
    vertex_ticks: int = 500
    motion_ticks: int = 5000
    initial_pass_extra_ticks: int = 0
    terminal_settle_ticks: int = 1000
    sample_every: int = 25


POLICY = Policy()


def finite_json(value):
    """Preserve invalid measurements explicitly, without breaking failure output."""
    if isinstance(value, dict):return {k:finite_json(v) for k,v in value.items()}
    if isinstance(value, (list,tuple,np.ndarray)):return [finite_json(v) for v in value]
    if isinstance(value, (float,np.floating)) and not np.isfinite(value):return str(value)
    if isinstance(value,np.generic):return value.item()
    return value


def vertex_indices(path, dense, scale, max_velocity=MAX_VELOCITY):
    indices=[0]
    for a,b in zip(path[:-1],path[1:]):
        n=len(planner.resample_path([a,b],max_velocity=max_velocity,dt=DT,max_acceleration=MAX_ACCELERATION))-1
        indices.append(indices[-1]+n*scale)
    if indices[-1]!=len(dense)-1 or any(not np.array_equal(dense[i],q) for i,q in zip(indices,path)):
        raise CandidateInfeasible('governor_original_vertex_mapping')
    return indices


def splice_bounds(path,dense,velocity,scale=1,max_velocity=MAX_VELOCITY):
    """Every optional insertion, both target-qd and position finite differences.

    Also covers adjacent duplicate endpoints, singleton paths and terminal hold.
    No interpolation, new vertex, time scaling or path reconstruction is used.
    """
    indices=vertex_indices(path,dense,scale,max_velocity);v=np.asarray(velocity)
    if v.shape!=np.asarray(dense).shape or not np.isfinite(v).all():
        raise CandidateInfeasible('governor_nonfinite_profile')
    zero=np.zeros(7);rows=[]
    peak_v=float(np.max(np.abs(v)));peak_a=float(np.max(np.abs(np.diff(np.vstack((zero,v,zero)),axis=0)/DT)))
    for vertex,i in enumerate(indices):
        incoming=v[i-1] if i else zero
        resume=v[i+1] if i+1<len(v) else zero
        # qd drop/resume, and position-FD stop/start after the endpoint.
        drop=float(np.max(np.abs(v[i]))/DT);restart=float(np.max(np.abs(resume))/DT)
        position_stop=float(np.max(np.abs(incoming))/DT)
        bound=max(drop,restart,position_stop)
        peak_a=max(peak_a,bound)
        rows.append(dict(vertex=vertex,index=i,resume_index=i+1,drop_acceleration=drop,
                         resume_acceleration=restart,position_stop_acceleration=position_stop))
    safe=peak_v<=MAX_VELOCITY+1e-12 and peak_a<=MAX_ACCELERATION+1e-12
    return dict(safe=safe,vertices=rows,max_velocity_rad_s=peak_v,max_acceleration_rad_s2=peak_a,
                velocity_limit=MAX_VELOCITY,acceleration_limit=MAX_ACCELERATION,exact_C2_proven=False)


class FiniteDifference:
    def __init__(self):self.previous=None;self.velocity=None

    def observe(self,q,step,dt):
        q=np.asarray(q,float)
        if (q.shape!=(7,) or not np.isfinite(q).all() or type(step) is not int
            or dt is None or not np.isfinite(dt) or abs(dt-DT)>1e-9):
            self.previous=None;self.velocity=None;return None
        if self.previous is not None:
            before,old=self.previous
            if step==before:
                if not np.array_equal(old,q):self.velocity=None
                return self.velocity
            self.velocity=((q-old)/dt).tolist() if step==before+1 else None
        else:self.velocity=None
        self.previous=(step,q.copy())
        return self.velocity


def readiness(certifier,q,actual,fd):
    actual=np.asarray(actual,float);q=np.asarray(q,float)
    result=dict(ready=False,whole_link_m=None,max_arm_error_rad=None,max_arm_fd_rad_s=None,
                link_displacement_m={},geometry='explicit_corresponding_vertices_raw_finger_reserve_1nm')
    if actual.shape!=(9,) or q.shape!=(7,) or not np.isfinite(actual).all() or not np.isfinite(q).all():return result
    reference=np.r_[q,certifier.fingers]
    a=dict(certifier.world.robot_shapes(certifier.arm,q.tolist(),np.clip(reference[-2:],0,.04).tolist()))
    b=dict(certifier.world.robot_shapes(certifier.arm,actual[:7].tolist(),np.clip(actual[-2:],0,.04).tolist()))
    clipped=np.abs(reference[-2:]-np.clip(reference[-2:],0,.04))+np.abs(actual[-2:]-np.clip(actual[-2:],0,.04))
    links={}
    for name,c in certifier.bounds.coefficients.items():
        x,y=np.asarray(a[name][1]),np.asarray(b[name][1])
        if x.shape!=y.shape or not len(x) or not np.isfinite(x).all() or not np.isfinite(y).all():return result
        links[name]=float(np.max(np.linalg.norm(x-y,axis=1)))+float(c[-2:]@clipped)+1e-9
    if not links:return result
    result.update(link_displacement_m=links,whole_link_m=max(links.values()),max_arm_error_rad=float(np.max(np.abs(actual[:7]-q))))
    if fd is None or np.asarray(fd).shape!=(7,) or not np.isfinite(fd).all():return result
    result['max_arm_fd_rad_s']=float(np.max(np.abs(fd)))
    result['ready']=bool(result['whole_link_m']<=POLICY.whole_link_m and result['max_arm_error_rad']<=POLICY.arm_error_rad
                         and result['max_arm_fd_rad_s']<=POLICY.arm_fd_rad_s)
    return result


class VertexGovernor:
    def __init__(self,api,arm,target,phase,path,dense,vel,scale):
        self.api=api;self.arm=arm;self.target=target;self.phase=phase
        self.enabled=getattr(api,'vertex_governor',False);self.fd=FiniteDifference()
        nominal=getattr(api,'home_nominal_speed_rad_s',MAX_VELOCITY) if phase=='home' and getattr(api,'home_coarse',False) else MAX_VELOCITY
        self.indices=vertex_indices(path,dense,scale,nominal);self.vertices={i:n for n,i in enumerate(self.indices)}
        self.added=0;self.intervened=False;self.first_change=None;self.started=time.monotonic()
        self.splices=splice_bounds(path,dense,vel,scale,nominal) if self.enabled else None
        if self.enabled and not self.splices['safe']:
            api.event('vertex_governor_rejected',arm=arm,object_id=target,motion_phase=phase,splices=self.splices)
            raise CandidateInfeasible('governor_unsupported_splice_before_motion')
        if self.enabled:
            api.event('vertex_governor_policy',arm=arm,object_id=target,motion_phase=phase,policy=asdict(POLICY),
                      policy_sha256=digest(asdict(POLICY)),splices=self.splices,
                      timing=dict(dt=DT,per_vertex_s=POLICY.vertex_ticks*DT,per_motion_s=POLICY.motion_ticks*DT,
                                  terminal_separate_s=POLICY.terminal_settle_ticks*DT))

    def checkpoint(self,index,row,trace,certifier):
        if self.enabled:
            rb=row['readback'];self.fd.observe(row['measured_q'][:7],rb['scene_step_count'],rb['scene_timestep_s'])
        i=index-1
        if i not in self.vertices:return False
        # Exact original trace prefix, stopped BEFORE the first changed command.
        checkpoint=dict(vertex=self.vertices[i],reference_index=i,resume_index=index,
                        prefix_packets=trace.count,prefix_sha256=trace.hash.hexdigest(),packet_sha256=digest(finite_json(row)))
        self.api.event('vertex_governor_checkpoint',arm=self.arm,object_id=self.target,motion_phase=self.phase,
                       comparable_prefix=not self.intervened,**checkpoint)
        if not self.enabled:return False
        return self.hold(np.asarray(row['monitor_reference_q']),i,index,certifier,checkpoint)

    def hold(self,q,i,resume,certifier,checkpoint):
        feedback=getattr(self.api,'velocity_feedback_controller',None)
        api=self.api;initial_step=api.telemetry()['scene_step_count'];started=time.monotonic()
        attempted=issued=tick_attempts=tick_returns=completed=streak=0
        samples=[];last=None;reason=None;rolling=hashlib.sha256();sample_count=0;status='checking'
        def retained_controls():
            return {a:[{k:v for k,v in j.items() if not (a==self.arm and j['name'] in
                [f'panda_joint{n}' for n in range(1,8)] and k in ('target','velocity_target'))}
                for j in js] for a,js in api.controls().items()}
        controls_before=digest(retained_controls())
        def capture(stage):
            s=api.state();rb=api.telemetry();actual=s['robots'][self.arm]['q']
            fd=self.fd.observe(actual[:7],rb['scene_step_count'],rb['scene_timestep_s'])
            monitor=certifier.monitor(s,q)
            live=ReturnClearance(api.world(s),self.arm,self.target,self.phase=='retreat')
            clearance=live.point(actual[:7],envelopes=False)
            return dict(stage=stage,reference_q=q.tolist(),reference_index=i,resume_index=resume,
                measured_q=actual,finite_difference_velocity=fd,readiness=readiness(certifier,q,actual,fd),
                monitor=monitor,clearance=clearance,scene_step=rb['scene_step_count'],
                readback=rb,controls_sha256=digest(api.controls()),state_sha256=digest(finite_json(s)))
        def guard(row):
            if not row['monitor']['safe']:raise CandidateInfeasible('governor_hard_envelope:'+','.join(row['monitor']['reasons']))
            if row['clearance'].get('safe') is not True:raise CandidateInfeasible('governor_hard_clearance')
            # Original contact, continuation/pose/world, and deadline guards,
            # read-only and before any command; tick retains its own guards.
            if digest(retained_controls())!=controls_before:raise CandidateInfeasible('governor_retained_controls_changed')
            api.governor_guard(self.target,self.phase=='retreat')
        def record(row):
            nonlocal sample_count,last
            last=finite_json(row);rolling.update(canonical(last));sample_count+=1
            if sample_count==1 or row['stage']=='posthold' and completed%POLICY.sample_every==0:
                samples.append(dict(last,rolling_sha256=rolling.hexdigest()))
        try:
            last=capture('admission');record(last);guard(last)
            if last['readiness']['ready']:
                last=capture('preresume');record(last);guard(last)
                if last['readiness']['ready']:
                    status='initial_pass';return False
            if feedback is not None:feedback.begin_recovery()
            while True:
                last=capture('prehold');record(last);guard(last)
                if completed>=POLICY.vertex_ticks:raise CandidateInfeasible('governor_vertex_timeout')
                if self.added+getattr(self,'progress_paused',0)>=POLICY.motion_ticks:raise CandidateInfeasible('governor_motion_timeout')
                if feedback is not None:feedback.observe_ready(last['readiness']['ready'])
                before=last['scene_step']
                try:
                    self.intervened=True
                    if self.first_change is None:self.first_change=dict(checkpoint)
                    attempted+=1;api.command(self.arm,q,np.zeros(7));issued+=1
                    tick_attempts+=1;api.tick(self.target,False,self.phase=='retreat');tick_returns+=1
                finally:
                    after=api.telemetry()['scene_step_count']
                    delta=after-before if type(after) is int and type(before) is int else 0
                    completed+=max(0,delta);self.added+=max(0,delta)
                    last=capture('posthold');last.update(hold_ticks=completed,streak_before=streak,
                        command_attempts=attempted,commands_issued=issued,tick_attempts=tick_attempts,tick_returns=tick_returns)
                    record(last)
                guard(last)
                if delta!=1:raise CandidateInfeasible('governor_tick_discontinuity')
                if feedback is not None:
                    feedback.observe_ready(last['readiness']['ready'],completed=True)
                    streak=feedback.recovery.streak
                else:streak=streak+1 if last['readiness']['ready'] else 0
                if streak>=POLICY.streak_ticks:
                    last=capture('preresume');record(last);guard(last)
                    if last['readiness']['ready']:
                        if feedback is not None:feedback.resume()
                        status='resumed';return True
                    streak=0
                    if feedback is not None:feedback.recovery.streak=0
        except Exception as exc:
            reason=type(exc).__name__+':'+str(exc);status='failed';raise
        finally:
            api.event('vertex_governor_vertex',schema='vertex-governor/13',arm=self.arm,object_id=self.target,motion_phase=self.phase,
                **checkpoint,status=status,reason=reason,reference_q=q.tolist(),**(dict(nominal_command_qdot=[0.]*7,last_issued_velocity_proof=feedback.last,
                        feedback_recovery=None if status=='initial_pass' else (vars(feedback.recovery).copy() if feedback.recovery is not None else feedback.last_recovery),
                        actual_velocity_targets=__import__('velocity_feedback').targets(api.controls(),self.arm)[1].tolist())
                   if feedback is not None else dict(command_qdot=[0.]*7)),
                command_attempts=attempted,commands_issued=issued,tick_attempts=tick_attempts,tick_returns=tick_returns,
                completed_hold_ticks=completed,motion_added_ticks=self.added,streak=streak,first_change=self.first_change,
                initial_scene_step=initial_step,elapsed_s=time.monotonic()-started,motion_elapsed_s=time.monotonic()-self.started,
                samples=samples,last_packet=finite_json(last),sample_count=sample_count,all_packets_sha256=rolling.hexdigest(),
                retained_controls_sha256=controls_before,
                failure_contacts=api.contacts() if reason else [],formal_credit=False)
