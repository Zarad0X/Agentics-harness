"""Explicit native-float32 goal formation and fail-closed command validation.

Goal quantization happens BEFORE IK pose acceptance and geometry planning.
Emission never clips a route or repairs an existing retained control.
"""
import numpy as np
from preview_errors import CandidateInfeasible


def inward_box(bounds):
    b=np.asarray(bounds,float)
    if b.ndim!=2 or b.shape[1]!=2 or not len(b) or not np.isfinite(b).all() or np.any(b[:,0]>=b[:,1]):
        raise ValueError('actuator_invalid_bounds')
    with np.errstate(over='ignore',invalid='ignore'):
        lo=b[:,0].astype(np.float32);hi=b[:,1].astype(np.float32)
        lo=np.where(lo.astype(float)<b[:,0],np.nextafter(lo,np.float32(np.inf)),lo)
        hi=np.where(hi.astype(float)>b[:,1],np.nextafter(hi,np.float32(-np.inf)),hi)
    result=np.column_stack((lo,hi)).astype(float)
    if not np.isfinite(result).all() or np.any(result[:,0]>result[:,1]) or np.any(result[:,0]<b[:,0]) or np.any(result[:,1]>b[:,1]):
        raise ValueError('actuator_no_finite_float32_interval')
    return result


def goal(q,bounds):
    """Form an inward representable candidate, never alter a certified route."""
    b=np.asarray(bounds,float);box=inward_box(b);q=np.asarray(q,float)
    if q.shape!=(len(b),) or not np.isfinite(q).all() or np.any(q<b[:,0]) or np.any(q>b[:,1]):
        raise ValueError('actuator_goal_outside_original_bounds')
    with np.errstate(over='ignore'):native=q.astype(np.float32).astype(float)
    if not np.isfinite(native).all():raise ValueError('actuator_goal_float32_overflow')
    result=np.clip(native,box[:,0],box[:,1])
    if not np.array_equal(result,result.astype(np.float32).astype(float)):
        raise ValueError('actuator_goal_not_representable')
    return result


def emitted(q,bounds):
    """Native representation of an already planned drive, with NO clipping."""
    b=np.asarray(bounds,float);inward_box(b);q=np.asarray(q,float)
    if q.ndim<1 or q.shape[-1]!=len(b) or not np.isfinite(q).all():
        raise CandidateInfeasible('actuator_command_nonfinite_or_shape')
    if np.any(q<b[:,0]) or np.any(q>b[:,1]):
        raise CandidateInfeasible('actuator_reference_outside_joint_margin')
    result=q.astype(np.float32).astype(float)
    if not np.isfinite(result).all() or np.any(result<b[:,0]) or np.any(result>b[:,1]):
        raise CandidateInfeasible('actuator_emitted_outside_joint_margin')
    return result


def issue(robot,arm,q,velocity,bounds):
    """Validate before issuing, then bind actual readback before any tick."""
    if arm not in ('left','right'):raise ValueError('actuator_invalid_arm')
    desired=emitted(q,bounds);v=np.asarray(velocity,float)
    if desired.ndim!=1 or v.shape!=desired.shape or not np.isfinite(v).all():
        raise CandidateInfeasible('actuator_velocity_nonfinite_or_shape')
    with np.errstate(over='ignore'):native_v=v.astype(np.float32).astype(float)
    if not np.isfinite(native_v).all():raise CandidateInfeasible('actuator_velocity_overflow')
    joints=getattr(robot,arm+'_arm_joints')
    if len(joints)!=len(desired):raise CandidateInfeasible('actuator_joint_mapping')
    robot.set_arm_joints(desired,v,arm)
    actual=np.asarray([float(j.get_drive_target()[0]) for j in joints])
    read_v=np.asarray([float(j.get_drive_velocity_target()[0]) for j in joints])
    if (actual.shape!=desired.shape or read_v.shape!=native_v.shape or
            not np.array_equal(actual,desired) or not np.array_equal(read_v,native_v) or
            actual.astype('<f4').tobytes()!=desired.astype('<f4').tobytes() or
            read_v.astype('<f4').tobytes()!=native_v.astype('<f4').tobytes()):
        raise CandidateInfeasible('actuator_native_readback_mismatch')
    # Do not use measured q or overwrite retained controls on disagreement.
    emitted(actual,bounds)
    return actual.tolist()


def handoff(model,arm,state,controls):
    """The existing loaded constructor's admission rules, without any command.

    Two identical measured references are sufficient to exercise its initial
    drive/velocity/bias/metadata predicates. This is not a route certificate.
    """
    import types
    from loaded_tracking import Tracking
    from provenance import digest
    q=state['robots'][arm]['q'][:7]
    Tracking(types.SimpleNamespace(arm=arm,world=types.SimpleNamespace(model=model)),
             [dict(q=q)]*1102,state,controls)
    row=dict(schema='retained-actuator-handoff/010',arm=arm,state=state,controls=controls,
             state_sha256=digest(state),controls_sha256=digest(controls),
             checked='unchanged_loaded_tracking_constructor_only',physical_completion_proved=False)
    return dict(row,sha256=digest(row))


def command_bytes(q,v):
    return np.r_[q,np.asarray(v,dtype=np.float32).astype(float)].astype('<f4').tobytes()


class Route:
    """Full native command-list admission; no command or physical state setter.

    Existing world.free predicate checks every actual float32 target, plus
    the retained-drive prefix at the unchanged planner .025rad sampling.
    This validates the command route; it is NOT a proof of native tracking.
    """
    def __init__(self,world,arm,target,path,dense,bias,previous,allow,event,profile,bridge=None,separation=None,caller=None):
        import hashlib
        from provenance import digest
        self.event=event;self.arm=arm;self.target=target;self.verified=0;self.attempted=0
        self.observed=hashlib.sha256();self.expected=hashlib.sha256()
        self.bounds=np.asarray(world.model.bounds)+[.03,-.03]
        if bridge is None and list(path[0])!=world.state['robots'][arm]['q'][:7]:
            raise CandidateInfeasible('actuator_route_reference_start')
        if bridge is not None and (bridge['state_sha256']!=digest(world.state) or bridge['path_sha256']!=digest(path) or bridge['arm']!=arm or bridge['object_id']!=target or bridge['reused_approval'] is not False):
            raise CandidateInfeasible('actuator_route_bridge_binding')
        self.q=emitted(np.asarray(dense)+bias,self.bounds)
        self.v=np.vstack((np.diff(dense,axis=0)/.004,np.zeros((1,7))))
        if not np.isfinite(self.v).all():raise CandidateInfeasible('actuator_velocity_nonfinite_or_shape')
        if np.max(np.abs(self.v))>profile['max_velocity']+1e-5:raise RuntimeError('interpolation_velocity_limit')
        previous=emitted(previous,self.bounds)
        if not len(self.q):raise CandidateInfeasible('actuator_empty_route')
        fingers=np.clip(world.state['robots'][arm]['q'][-2:],0,.04).tolist()
        resolution=.025 if separation is None else .002
        prefix=np.linspace(previous,self.q[0],max(2,int(np.ceil(np.max(np.abs(self.q[0]-previous))/resolution))+1))
        checks=0
        # Use the existing separating-contact contract, not the ordinary 1mm
        # world.free default, for this explicitly identified grasp phase.
        separating=None
        if separation is not None:
            separating={k:separation[k] for k in ('path','initial_clearance','initial_contacts')}
            from copy import deepcopy
            separating=deepcopy(separating)
            if separating['path']!=np.asarray(path).tolist():raise CandidateInfeasible('actuator_separation_path')
            previous_clearance=separating['initial_clearance'].copy()
            touching={k for k,v in previous_clearance.items() if v<=2e-5}
            evidence=separating['initial_contacts']
            if not previous_clearance or any(k not in evidence or not np.isfinite([evidence[k]['separation_min'],evidence[k]['impulse_max']]).all() or evidence[k]['separation_min']<-.00005 or evidence[k]['impulse_max']>.002 for k in touching):
                raise CandidateInfeasible('actuator_separation_initial_contact')
        for q in list(prefix)+list(self.q):
            checks+=1
            kwargs={}
            if separating is not None:
                thresholds={k:max(1e-5,min(v,.001)-1e-6) for k,v in previous_clearance.items()}
                at_start=np.max(np.abs(q-np.asarray(path[0])))<1e-10
                kwargs=dict(payload_clearance=thresholds,start_contact_pairs=touching if at_start else ())
            if not world.free(arm,q.tolist(),target,allow,fingers,**kwargs):
                raise CandidateInfeasible('actuator_emitted_route_collision:'+str(world.last_collision))
            if separating is not None:
                measured=world.payload_clearances(arm,q.tolist(),previous_clearance)
                previous_clearance={k:max(previous_clearance[k],measured[k]) for k in previous_clearance}
        if separating is not None:
            final=world.payload_clearances(arm,self.q[-1].tolist(),previous_clearance)
            if min(final.values())<.003 or not world.free(arm,self.q[-1].tolist(),target,allow,fingers):raise CandidateInfeasible('actuator_separation_final_clearance')
        for q,v in zip(self.q,self.v):self.expected.update(command_bytes(q,v))
        relative=None if world.held is None else np.asarray(world.held[2]).tolist()
        row=dict(schema='native-actuator-route/010',caller_sha256=caller,arm=arm,object_id=int(target),state=world.state,
                 state_sha256=digest(world.state),path=np.asarray(path).tolist(),bias=np.asarray(bias).tolist(),
                 previous=previous.tolist(),profile=profile,bridge=bridge,separation=separating,allow_target_contact=allow,held_relative=relative,
                 native_targets_sha256=digest(self.q.tolist()),commands_sha256=self.expected.hexdigest(),
                 command_count=len(self.q),geometry_checks=checks,physical_completion_proved=False)
        self.receipt=dict(row,sha256=digest(row));event('actuator_route_admission',receipt=self.receipt)

    def issue(self,robot,index):
        self.attempted+=1
        q=issue(robot,self.arm,self.q[index],self.v[index],self.bounds)
        self.observed.update(command_bytes(q,self.v[index]));self.verified+=1

    def finish(self,failed):
        self.event('actuator_route_readback',route_sha256=self.receipt['sha256'],arm=self.arm,
                   object_id=int(self.target),attempted=self.attempted,verified_readbacks=self.verified,readback_sha256=self.observed.hexdigest(),
                   emission_complete=bool(not failed and self.verified==len(self.q)),failed=failed,
                   physical_completion_proved=False)
