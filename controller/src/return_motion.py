"""Bounded, measured and instrumented post-release motion. No physics reset."""
from collections import deque
import copy
import hashlib
import time
import numpy as np
import astra_planner as planner
from collision_world import pose_matrix
from provenance import canonical,digest
from preview_errors import CandidateInfeasible
from return_clearance import (ReturnClearance,DT,MAX_VELOCITY,MAX_ACCELERATION,
                              TRACK_Q,TRACK_FINGER,LINK_TUBE_M,WORLD_DRIFT_M,SPEED_BOX)


class MotionTrace:
    """Same packet schema in shadows/formal; hash every tick, keep bounded detail."""
    def __init__(self,matched_scale=None):
        self.hash=hashlib.sha256();self.count=0;self.samples=[];self.window=deque(maxlen=64)
        self.sample_every=100*(matched_scale or 1);self.sample_offset=2 if matched_scale else 1

    def add(self,packet):
        self.hash.update(canonical(packet));self.count+=1;self.window.append(packet)
        if (self.count-self.sample_offset)%self.sample_every==0:
            self.samples.append(dict(packet,rolling_sha256=self.hash.hexdigest(),packet_number=self.count))
            if len(self.samples)>256:
                self.samples=self.samples[::2];self.sample_every*=2

    def finish(self,failed):
        return dict(ticks=self.count,measured_trace_sha256=self.hash.hexdigest(),samples=self.samples,
                    ticks_field_meaning='legacy packet count; use motion_completed_scene_steps for physics ticks',
                    failure_window=list(self.window) if failed else [],window_capacity=64,
                    sample_capacity=256,sample_every=self.sample_every,packet_count=self.count)


def dense_profile(path,time_scale=1):
    if type(time_scale) is not int or time_scale not in (1,2):raise ValueError('retreat_time_scale')
    dense=np.asarray(planner.resample_path(path,max_velocity=MAX_VELOCITY,dt=DT,max_acceleration=MAX_ACCELERATION))
    if time_scale==2:
        # Double each integer segment duration exactly, including the bridge.
        # Evaluate the SAME quintic clock on the SAME certified segment.
        expanded=[np.asarray(path[0],float)]
        for a,b in zip(np.asarray(path[:-1],float),np.asarray(path[1:],float)):
            n=len(planner.resample_path([a,b],max_velocity=MAX_VELOCITY,dt=DT,max_acceleration=MAX_ACCELERATION))-1
            for i in range(1,2*n+1):
                u=i/(2*n);expanded.append(b if i==2*n else a+(b-a)*u**3*(10-15*u+6*u*u))
        dense=np.asarray(expanded)
    verify_interpolation(path,dense)
    velocity=np.vstack((np.diff(dense,axis=0)/DT,np.zeros((1,7))))
    acceleration=np.diff(np.vstack((np.zeros((1,7)),velocity)),axis=0)/DT
    if np.max(np.abs(velocity))>MAX_VELOCITY/time_scale+1e-8 or np.max(np.abs(acceleration))>MAX_ACCELERATION/time_scale**2+1e-5:
        raise RuntimeError('return_command_profile_bound')
    return dense,velocity


def motion_readback(env,step):
    """Getters only: qf is the last applied force, never recomputed here.

    Scene step count is the runner's completed tick counter (all shadows or
    formal-episode), excluding scene setup. Within this motion all ticks refer
    to the same scene; subtract its initial counter for an exact local count.
    A throwing post-step guard retains
    the incremented counter. These reads do not issue commands or scene steps.
    """
    return dict(scene_timestep_s=float(env.scene.get_timestep()),scene_step_count=step,
        scene_step_scope='completed runner ticks: cumulative shadows or formal episode; excludes setup',
        applied_passive_qf={a:getattr(env.robot,a+'_entity').get_qf().tolist() for a in ('left','right')})


def path_progress(path,q):
    """Segment plus linear fraction: independent of dense command count."""
    vertices=np.asarray(path,float);q=np.asarray(q,float)
    for i,(a,b) in enumerate(zip(vertices[:-1],vertices[1:])):
        d=b-a;u=float((q-a)@d/(d@d)) if d@d else 0.
        if -1e-10<=u<=1+1e-10 and np.max(np.abs(q-a-u*d))<1e-10:return i+float(np.clip(u,0,1))
    return float(max(0,len(vertices)-1))


def verify_interpolation(path,dense):
    """Check every emitted command belongs, in order, to the certified curve."""
    vertices=np.asarray(path,float);dense=np.asarray(dense,float)
    if dense.ndim!=2 or dense.shape[1]!=7 or not len(dense) or not np.isfinite(dense).all():raise ValueError('invalid dense commands')
    if not np.array_equal(dense[0],vertices[0]) or not np.array_equal(dense[-1],vertices[-1]):
        raise ValueError('dense endpoints differ')
    segment=0;previous=0.
    for q in dense:
        while segment<len(vertices)-2 and np.max(np.abs(q-vertices[segment+1]))<=1e-12:
            segment+=1;previous=0.
        if len(vertices)==1:
            if np.max(np.abs(q-vertices[0]))>1e-12:raise ValueError('dense curve changed')
            continue
        a,b=vertices[segment:segment+2];d=b-a;length=float(d@d)
        u=0. if length==0 else float((q-a)@d/length)
        if u<previous-1e-12 or not -1e-12<=u<=1+1e-12 or np.max(np.abs(q-a-u*d))>1e-12:
            raise ValueError('dense curve changed')
        previous=u


def cartesian_escape(api,arm,q,height):
    start=api.model.evaluate(arm,list(q));p=np.asarray(start['ee_position_m']);r=start['ee_rotation_world'];path=[list(q)]
    for rise in np.linspace(0,height,int(np.ceil(height/.01))+1)[1:]:
        ik=planner.solve_ik(api.model,arm,p+[0,0,rise],r,path[-1],rng=api.rng,max_iters=120,restarts=0)
        if not ik['success']:return None
        path.append(ik['q'])
    return path


def actuator_start(api,arm,allow_velocity=False):
    joints={j['name']:j for j in api.controls()[arm]}
    selected=[joints[f'panda_joint{i}'] for i in range(1,8)]
    if not allow_velocity and any(abs(j['velocity_target'][0])>1e-10 for j in selected):
        raise CandidateInfeasible('postrelease_nonzero_start_drive_velocity')
    return [float(j['target'][0]) for j in selected]


class RetainedHold:
    """Read-only per-tick guard for idle arms or the two-arm settled hold.

    Never writes q, targets, gains or finger state. The moving arm retains its
    ordinary loaded planner/contact checks; its live geometry is checked here
    against the idle arm on both sides of every physics tick.
    """
    def __init__(self,api,arms,target,phase):
        self.api=api;self.arms=tuple(arms);self.target=target;self.phase=phase
        self.controls=copy.deepcopy(api.controls());self.trace=MotionTrace();self.finished=False;self.started=time.monotonic()
        self.reference=copy.deepcopy(api.state());self.certifiers={};self.commands={}
        for arm in self.arms:
            c=ReturnClearance(api.world(self.reference),arm,target,False)
            joints={j['name']:j for j in self.controls[arm]}
            c.fingers=np.clip([joints['panda_finger_joint'+str(i)]['target'][0] for i in (1,2)],0,.04)
            self.commands[arm]=actuator_start(api,arm);self.certifiers[arm]=c
            if len(self.arms)==2:
                certificate=c.certify([self.commands[arm]])
                api.event('return_hold_certificate',arm=arm,object_id=target,certificate=certificate)
                if not certificate['certified']:raise CandidateInfeasible('retained_hold_uncertified:'+arm)
        api.event('return_hold_admitted',motion_phase=phase,object_id=target,arms=list(self.arms),
                  state=self.reference,controls=self.controls,rng_sha256=digest(api.rng.bit_generator.state))

    def released(self):
        # The released target becomes stationary at its actual release pose.
        # No other obstacle or idle-arm reference is rebased.
        observed=copy.deepcopy(self.api.state()['objects'][str(self.target)])
        self.reference['objects'][str(self.target)]=observed
        self.api.event('return_hold_release_reference',object_id=self.target,observation=observed)

    def check(self,stage,held=False):
        api=self.api;s=api.state();controls=api.controls();reports={};reason=None
        for arm,c in self.certifiers.items():
            monitor=c.monitor(s,self.commands[arm],monitor_other=len(self.arms)==2,
                              moving_object=self.target if held else None)
            live=ReturnClearance(api.world(s),arm,self.target,False)
            # Actual other-arm and payload shapes, never a cached path verdict.
            clearance=live.point(s['robots'][arm]['q'][:7],envelopes=False)
            reports[arm]=dict(monitor=monitor,clearance=clearance)
            if controls[arm]!=self.controls[arm]:reason='retained_controls_changed:'+arm
            elif not monitor['safe']:reason='retained_envelope:'+arm+':'+','.join(monitor['reasons'])
            elif not clearance['safe']:reason='retained_live_clearance:'+arm
        self.trace.add(dict(stage=stage,state=s,controls=controls,reports=reports,held=held,reason=reason,
            contacts=api.contacts(),readback=getattr(api,'telemetry',lambda:None)()))
        if reason:
            self.finish(reason)
            raise CandidateInfeasible(reason)

    def finish(self,reason=None):
        if self.finished:return
        self.api.event('return_hold_trace',object_id=self.target,motion_phase=self.phase,
                       arms=list(self.arms),failed=reason is not None,reason=reason,elapsed_s=time.monotonic()-self.started,
                       **self.trace.finish(reason is not None))
        self.finished=True


def bridge_to_measured(api,arm,target):
    """Certify and physically execute retained drive -> measured approach start.

    The subsequent grasp planner must re-read measured q after this execution.
    This certificate is not reusable for any subsequent approach or segment.
    """
    s=api.state();drive=actuator_start(api,arm,__import__('velocity_feedback').enabled(api,'handoff'));measured=s['robots'][arm]['q'][:7]
    c=ReturnClearance(api.world(s),arm,target,False)
    admission=c.monitor(s,drive)
    if not admission['safe']:raise CandidateInfeasible('handoff_initial_tracking:'+','.join(admission['reasons']))
    path=[drive,measured];certificate=c.certify(path)
    api.event('return_handoff_bridge',arm=arm,object_id=target,drive_q=drive,measured_q=measured,
              certificate=certificate,state_sha256=digest(s),controls_sha256=digest(api.controls()),
              rng_sha256=digest(api.rng.bit_generator.state),reusable=False)
    if not certificate['certified']:raise CandidateInfeasible('handoff_bridge_uncertified')
    execute_motion(api,arm,target,'handoff',path,c,certificate,[],dict(state_sha256=digest(s)))


def plan_motion(api,arm,target,phase,home=None):
    s=api.state();world=api.world(s);allow=phase=='retreat'
    certifier=ReturnClearance(world,arm,target,allow)
    q0=s['robots'][arm]['q'][:7];bounds=np.array(api.model.bounds)+[.03,-.03]
    drive_q=actuator_start(api,arm,__import__('velocity_feedback').enabled(api,phase))
    admission=certifier.monitor(s,drive_q)
    if not admission['safe']:
        api.event('return_motion_rejected',object_id=target,arm=arm,motion_phase=phase,
                  reason='initial_drive_tracking',monitor=admission,drive_q=drive_q,measured_robots=s['robots'])
        raise CandidateInfeasible('postrelease_initial_drive_tracking')
    def bridge(path):
        return path if np.array_equal(drive_q,path[0]) else [drive_q]+path
    attempts=[]
    # Home: direct first, then two bounded orientation-preserving lifts before
    # any wrist turn. Plan and certify the WHOLE composite before moving.
    heights=(.12,) if allow else (0.,.08,.16)
    for height in heights:
        prefix=[q0] if height==0 else cartesian_escape(api,arm,q0,height)
        if prefix is None:attempts.append(dict(height=height,reason='escape_ik'));continue
        if allow:paths=[prefix]
        else:
            # A point's robust clearance guides existing custom RRT. Complete
            # edge certification below is authoritative, not its .025 sampling.
            paths=[]
            for retry in range(2):
                plan=planner.plan_joint_path(prefix[-1],home,bounds,
                    lambda q:bool(certifier.point(q)['safe']),rng=api.rng,max_nodes=80,resolution=.025)
                if not plan['success']:
                    attempts.append(dict(height=height,retry=retry,reason=plan['reason']));continue
                path=bridge(prefix+plan['path'][1:])
                certificate=certifier.certify(path)
                attempts.append(dict(height=height,retry=retry,certificate=certificate))
                if certificate['certified']:
                    if getattr(api,'home_coarse',False) and getattr(api,'joint_step_refinement',False):
                        from home_admission import admit
                        try:
                            resource_admission=admit(path,getattr(api,'home_nominal_speed_rad_s',.1))
                        except CandidateInfeasible as exc:
                            attempts[-1]['computational_rejection']=str(exc)
                            # Continue only the existing bounded search. Each
                            # replacement curve needs its own full certificate.
                            continue
                        api.event('home_command_admission',object_id=target,arm=arm,
                                  stage='certified_measured_route',**resource_admission)
                    return path,certifier,certificate,attempts
            continue
        for path in paths:
            path=bridge(path)
            certificate=certifier.certify(path);attempts.append(dict(height=height,certificate=certificate))
            if certificate['certified']:return path,certifier,certificate,attempts
    api.event('return_motion_rejected',object_id=target,arm=arm,motion_phase=phase,attempts=attempts,
              state_sha256=digest(s),controls_sha256=digest(api.controls()),rng_sha256=digest(api.rng.bit_generator.state))
    raise CandidateInfeasible('postrelease_clearance_plan_failed:'+phase)


def execute_motion(api,arm,target,phase,path,certifier,certificate,attempts,planning_start):
    if not getattr(api,'home_coarse',False):
        return _execute_motion(api,arm,target,phase,path,certifier,certificate,attempts,planning_start)
    from home_profile import PhaseReceipt
    receipt=PhaseReceipt(api,arm,target,phase)
    reason=None
    try:
        return _execute_motion(receipt,arm,target,phase,path,certifier,certificate,attempts,planning_start)
    except Exception as exc:
        reason=type(exc).__name__+':'+str(exc)
        raise
    finally:
        receipt.finish(reason)


def _execute_motion(api,arm,target,phase,path,certifier,certificate,attempts,planning_start):
    from velocity_feedback import enabled, FeedbackAPI
    feedback_on=enabled(api,phase)
    feedback=None
    scale=getattr(api,'retreat_time_scale',1) if phase=='retreat' else 1
    refinement=None
    if getattr(api,'joint_step_refinement',False):
        from motion_refinement import prepare,ExecutionBudget
        if getattr(api,'home_coarse',False) and phase=='home':
            from home_profile import prepare_coarse
            path,certifier,certificate,dense,vel=prepare_coarse(api,arm,target,phase,path,certificate)
        else:
            path,certifier,certificate,dense,vel=prepare(api,arm,target,phase,path,certificate)
        refinement=ExecutionBudget(api,arm,target,phase,len(dense));api=refinement
    else:
        dense,vel=dense_profile(path,scale)
    probe=getattr(api,'fixed_setpoint_diagnostic',None)
    hold_checkpoint=None
    calibration=getattr(api,'feedback_scope','retreat')=='calibration'
    if calibration:
        if probe!='hold' or not getattr(api,'independent_diagnostic',False):raise ValueError('calibration_requires_independent_fixed_hold')
        api.feedback_prefix=[]
    if probe is not None and getattr(api,'vertex_governor',False):
        raise ValueError('fixed_setpoint_incompatible_vertex_governor')
    if probe is not None:
        from fixed_setpoint_diagnostic import validate_vertex
        validate_vertex(api,arm,target,phase,path,dense,certificate)
    if feedback_on and (np.asarray(dense).ndim!=2 or np.asarray(dense).shape[1]!=7 or not np.isfinite(dense).all() or np.asarray(vel).shape!=np.asarray(dense).shape or not np.isfinite(vel).all() or np.max(np.abs(vel))>.1+1e-12 or len(dense)>30000 or len(path)>512 or certificate.get('certified') is not True):
        raise CandidateInfeasible('velocity_feedback_profile_preflight')
    if not np.array_equal(actuator_start(api,arm,feedback_on),dense[0]):
        raise CandidateInfeasible('postrelease_start_controls_changed')
    if feedback_on:
        feedback=FeedbackAPI(api,arm,target,phase,api.velocity_feedback,getattr(api,'feedback_telemetry',False));api=feedback
        if hasattr(api,'feedback_metadata'):api.event('feedback_metadata',metadata=api.feedback_metadata())
    start=api.state();controls=api.controls();trace=MotionTrace(scale);failed=True;reason=None
    last_clearance=certificate['witness'];clearance_index=-1
    telemetry=getattr(api,'telemetry',lambda:dict(scene_timestep_s=None,scene_step_count=None,applied_passive_qf=None))
    initial_readback=telemetry()
    if initial_readback['scene_timestep_s'] is not None and abs(initial_readback['scene_timestep_s']-DT)>1e-9:
        raise CandidateInfeasible('postrelease_scene_timestep_mismatch')
    from vertex_governor import VertexGovernor
    governor=VertexGovernor(api,arm,target,phase,path,dense,vel,scale) if probe is None else None
    progress=None;terminal_ticks=0
    if phase=='home' and getattr(api,'home_progress',False):
        from home_progress import ProgressGovernor
        progress=ProgressGovernor(api,arm,target,path,certifier,list(governor.vertices))
    api.event('return_motion_plan',schema='postrelease-motion/9',object_id=target,arm=arm,motion_phase=phase,vertices=path,
        pre_retreat_observation_prefix_sha256=getattr(api,'prefix_sha256',None),
        path_sha256=digest(path),certificate_sha256=digest(certificate),
        dense_command_sha256=digest(dict(q=dense.tolist(),qdot=vel.tolist())),dense_commands=len(dense),
        start_state_sha256=digest(start),controls_sha256=digest(controls),rng_sha256=digest(api.rng.bit_generator.state),
        start_robots=start['robots'],start_controls=controls,planning_start=planning_start,initial_readback=initial_readback,
        certificate=certificate,attempts=attempts,profile=dict(dt=DT,time_scale=scale,
            configured_retreat_time_scale=getattr(api,'retreat_time_scale',1),home_time_scale=1,
            **(dict(home_progress=True,dense_hash_semantics='nominal comparison profile; executed reference governed',
                    actual_dt_s=progress.dt) if progress is not None else {}),
            **(dict(home_clock_changed=True,home_nominal_speed_rad_s=.06) if phase=='home' and getattr(api,'home_nominal_speed_rad_s',.1)==.06 else {}),
            max_velocity=(getattr(api,'home_nominal_speed_rad_s',MAX_VELOCITY) if phase=='home' and getattr(api,'home_coarse',False) else MAX_VELOCITY/scale),max_acceleration=MAX_ACCELERATION/scale**2,
            track_joint_rad=TRACK_Q,track_finger_m=TRACK_FINGER,link_tube_m=LINK_TUBE_M,world_drift_m=WORLD_DRIFT_M,
            speed_envelope=SPEED_BOX.tolist(),conditional_physical_envelope=True,
            step_geometry='axis radii plus downstream tracking/step/interval displacement bounds'))
    previous=dense[0];last_issued=None;issued_index=None;last_index=-1;q=dense[0];v=vel[0]
    fd_previous=None;command_attempted=False;command_issued=False;tick_attempted=False;tick_completed=False
    def packet(index,q,v,s,monitor,reference,reference_index,stage):
        nonlocal fd_previous
        rb=telemetry();now=rb['scene_step_count'];measured=np.asarray(s['robots'][arm]['q'])
        fd=None
        if fd_previous is not None and now is not None and now>fd_previous[0]:
            fd=((measured-fd_previous[1])/((now-fd_previous[0])*rb['scene_timestep_s'])).tolist()
        if now is not None and (fd_previous is None or now!=fd_previous[0]):fd_previous=(now,measured.copy())
        return dict(command_index=index,command_q=q.tolist(),command_qdot=v.tolist(),packet_stage=stage,
                    motion_completed_scene_steps=None if now is None else now-initial_readback['scene_step_count'],
                    upcoming_command_index=index,upcoming_command_q=q.tolist(),upcoming_command_qdot=v.tolist(),
                    monitor_reference_q=reference.tolist(),monitor_reference_index=reference_index,
                    monitor_reference_kind='initial_drive_target' if reference_index is None else 'last_issued_command',
                    last_issued_q=None if last_issued is None else last_issued.tolist(),last_issued_index=issued_index,
                    monitor_path_progress=path_progress(path,reference),upcoming_path_progress=path_progress(path,q),
                    measured_q=measured.tolist(),measured_qdot=s['robots'][arm]['qdot'],
                    finite_difference_velocity=fd,readback=rb,drive_readback=api.controls(),
                    fingers=s['robots'][arm]['q'][-2:],monitor=monitor,clearance=last_clearance,
                    clearance_measured_at_command_index=clearance_index,contacts=api.contact_summary())
    def issuance(row):
        after=telemetry()['scene_step_count'];before=row['readback']['scene_step_count']
        result=dict(command_attempted=command_attempted,command_issued=command_issued,
                    command_completion_uncertain=command_attempted and not command_issued,
                    tick_attempted=tick_attempted,tick_completed=tick_completed,
                    physics_tick_issued=None if after is None or before is None else after>before,
                    # Actual completed physics is observable even if a post-step guard throws.
                    scene_step_after_attempt=after)
        if governor is not None and governor.enabled and not tick_attempted:
            # Dwell owns its ticks even when it fails before the pending dense
            # command. Never attribute those ticks to that unissued command.
            result.update(physics_tick_issued=False,governor_completed_scene_steps=
                None if after is None or before is None else after-before)
        return result
    try:
        for index in range(36000 if progress is not None else len(dense)+1000):
            q=dense[min(index,len(dense)-1)] if progress is None else progress.q.copy()
            v=(vel[index] if index<len(dense) else np.zeros(7)) if progress is None else progress.v.copy()
            last_index=index;command_attempted=command_issued=tick_attempted=tick_completed=False
            s=api.state();monitor=certifier.monitor(s,previous)
            if index%100==0 or not monitor['safe']:
                live=ReturnClearance(api.world(s),arm,target,phase=='retreat')
                last_clearance=live.point(s['robots'][arm]['q'][:7],envelopes=False);clearance_index=index
            row=packet(index,q,v,s,monitor,previous,issued_index,'precommand')
            try:
                if not monitor['safe']:raise CandidateInfeasible('postrelease_precommand_envelope:'+','.join(monitor['reasons']))
                if last_clearance is not None and last_clearance.get('safe') is False:
                    raise CandidateInfeasible('postrelease_measured_clearance')
                if progress is not None and index==0 and getattr(api,'home_entry_hold',False):
                    from home_entry import prepare as prepare_entry
                    if prepare_entry(api,governor,progress,certifier,row,trace):
                        s=api.state();monitor=certifier.monitor(s,previous)
                        row=packet(index,q,v,s,monitor,previous,issued_index,'precommand')
                checkpoint_index=index
                if progress is not None:
                    governor.fd.observe(row['measured_q'][:7],row['readback']['scene_step_count'],row['readback']['scene_timestep_s'])
                    checkpoint_index=progress.checkpoint()
                    governor.progress_paused=progress.paused
                if governor is not None and checkpoint_index is not None and governor.checkpoint(checkpoint_index,row,trace,certifier):
                    # Dwell does not consume a dense index or terminal settling tick.
                    # Refresh the ordinary packet at the continuous resumed state.
                    if progress is not None:progress.resumed()
                    s=api.state();monitor=certifier.monitor(s,previous)
                    row=packet(index,q,v,s,monitor,previous,issued_index,'precommand')
                terminal_index=index-len(dense)
                if progress is not None:
                    terminal_index=terminal_ticks if progress.done else -1
                    if terminal_index>=1000:raise CandidateInfeasible('postrelease_measured_settling_failed')
                    q,v,decision=progress.choose(s,telemetry(),governor.added)
                    row=packet(index,q,v,s,monitor,previous,issued_index,'precommand')
                    row['home_progress_decision']=decision
                    if terminal_index>=0:terminal_ticks+=1
                if probe is not None and index==201:
                    # Prefix hash ends AFTER issued command 200 and its tick.
                    # This packet still has the ordinary upcoming command 201.
                    checkpoint=dict(schema='fixed-setpoint-checkpoint/11',packet=copy.deepcopy(row),
                        state=s,controls=api.controls(),rng=copy.deepcopy(api.rng.bit_generator.state),
                        prefix_packets=trace.count,prefix_sha256=trace.hash.hexdigest(),
                        path_sha256=digest(path),certificate_sha256=digest(certificate),
                        formal_credit=False,checkpoint_eligible=False)
                    if issued_index!=200 or not np.array_equal(previous,path[2]):
                        raise ValueError('fixed_setpoint_wrong_last_issued_reference')
                    api.event('fixed_setpoint_checkpoint',**checkpoint)
                    if probe=='hold':hold_checkpoint=checkpoint;break
                if feedback is not None and terminal_index>=0:
                    from vertex_governor import readiness
                    feedback.begin_recovery()
                    feedback.observe_ready(readiness(certifier,q,s['robots'][arm]['q'],governor.fd.velocity)['ready'],completed=terminal_index>0)
                if terminal_index>=100 and (feedback is None or (feedback.zero() and feedback.recovery.streak>=25)):
                    robot=s['robots'][arm]
                    if max(abs(np.asarray(robot['q'][:7])-dense[-1]))<=.005 and max(abs(np.asarray(robot['qdot'][:7])))<=.005:
                        desired=api.model.evaluate(arm,dense[-1].tolist());actual=pose_matrix(api.ee_pose(arm))
                        pe=float(np.linalg.norm(np.asarray(desired['ee_position_m'])-actual[:3,3]))
                        re=float(np.linalg.norm(planner.so3_log(np.asarray(desired['ee_rotation_world'])@actual[:3,:3].T)))
                        if pe<=.005 and re<=.04:
                            if feedback is not None:feedback.read()
                            api.event('return_motion_settled',object_id=target,arm=arm,motion_phase=phase,
                                      position_error_m=pe,rotation_error_rad=re,measured_q=robot['q'],measured_qdot=robot['qdot'])
                            failed=False;break
                prefix_pre=None
                if calibration and 189<=index<=200:
                    from feedback_telemetry import packet as numeric_packet
                    prefix_pre=numeric_packet(api,arm)
                command_attempted=True;api.command(arm,q,v);command_issued=True
                if feedback is not None:
                    row['nominal_command_qdot']=row['command_qdot'];row['command_qdot']=feedback.last['issued_qdot']
                    row['velocity_feedback_proof']=feedback.last
                last_issued=q;issued_index=index;previous=q
                tick_attempted=True;api.tick(target,False,phase=='retreat');tick_completed=True
                if prefix_pre is not None:
                    from feedback_telemetry import pair
                    api.feedback_prefix.append(pair(prefix_pre,numeric_packet(api,arm),q,v))
            finally:
                row.update(issuance(row));trace.add(row)
        else:raise CandidateInfeasible('postrelease_measured_settling_failed')
        if hold_checkpoint is not None:
            if getattr(api,'feedback_scope','retreat')=='calibration':
                from feedback_probe import run_hold
            else:
                from fixed_setpoint_diagnostic import run_hold
            run_hold(api,arm,target,path,certifier,hold_checkpoint)
    except Exception as exc:
        reason=type(exc).__name__+':'+str(exc)
        if hold_checkpoint is not None:
            # Hold owns all attempted/completed and post-throw packets. Never
            # relabel its final state as ordinary trajectory command 201.
            raise
        # Precommand failure: previous is the last ISSUED command (or initial
        # drive). Throwing tick: previous was updated on successful command,
        # and this packet reads the actual post-throw state without stepping.
        s=api.state();monitor=certifier.monitor(s,previous)
        try:
            live=ReturnClearance(api.world(s),arm,target,phase=='retreat')
            last_clearance=live.point(s['robots'][arm]['q'][:7],envelopes=False);clearance_index=last_index
        except Exception as measurement_error:
            last_clearance=dict(measurement_error=type(measurement_error).__name__+':'+str(measurement_error))
        exception_row=packet(last_index,q,v,s,monitor,previous,issued_index,'exception')
        exception_row.update(issuance(row) if 'row' in locals() else issuance(exception_row));trace.add(exception_row)
        raise
    finally:
        api.event('return_motion_trace',schema='postrelease-motion/9',object_id=target,arm=arm,motion_phase=phase,failed=failed,reason=reason,
                  failure_contacts=api.contacts() if failed else [],**trace.finish(failed))
        if progress is not None:progress.finish(failed)
        if feedback is not None:feedback.finish()
        if refinement is not None:refinement.finish()


def run_motion(api,arm,target,phase,home=None):
    if phase not in ('retreat','home'):raise ValueError('postrelease phase')
    planning_start=dict(state_sha256=digest(api.state()),controls_sha256=digest(api.controls()),
                        rng_sha256=digest(api.rng.bit_generator.state))
    path,certifier,certificate,attempts=plan_motion(api,arm,target,phase,home)
    execute_motion(api,arm,target,phase,path,certifier,certificate,attempts,planning_start)


def run_supplied_retreat(api,arm,target,measured_path):
    """Execute the existing extraction route with the audited unloaded executor.

    Keep its exact endpoint. The explicit retained-drive -> measured-start
    prefix is newly certified with the entire route, never a target reset.
    """
    state=api.state();controls=api.controls();path=np.asarray(measured_path,float)
    if path.ndim!=2 or path.shape[1]!=7 or not len(path) or not np.isfinite(path).all():
        raise CandidateInfeasible('extraction_retreat_path')
    if not np.array_equal(path[0],state['robots'][arm]['q'][:7]):
        raise CandidateInfeasible('extraction_retreat_stale_start')
    drive=actuator_start(api,arm,__import__('velocity_feedback').enabled(api,'retreat'))
    certifier=ReturnClearance(api.world(state),arm,target,True)
    admission=certifier.monitor(state,drive)
    route=path.tolist() if np.array_equal(drive,path[0]) else [drive]+path.tolist()
    certificate=certifier.certify(route) if admission['safe'] else None
    api.event('extraction_retreat_binding',object_id=target,arm=arm,state=state,controls=controls,
        state_sha256=digest(state),controls_sha256=digest(controls),measured_path=path.tolist(),
        route=route,route_sha256=digest(route),drive_q=drive,admission=admission,certificate=certificate,
        endpoint_semantics='unchanged_original_retreat_IK_endpoint',reference_semantics='explicit_retained_drive_prefix_then_original_route')
    if not admission['safe']:raise CandidateInfeasible('extraction_retreat_initial_drive_tracking')
    if not certificate['certified']:raise CandidateInfeasible('extraction_retreat_uncertified')
    execute_motion(api,arm,target,'retreat',route,certifier,certificate,[],
        dict(state_sha256=digest(state),controls_sha256=digest(controls),rng_sha256=digest(api.rng.bit_generator.state)))
