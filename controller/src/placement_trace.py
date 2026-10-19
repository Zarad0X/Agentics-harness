"""Optional bounded placement witness. No control, solver, RNG or collision calls.

Only selected actual lowering/release/first-retreat ticks enter the formal file.
Foreign scene observations are segregated and invalidate the requested witness.
Instrumentation errors disable recording, never retry or restore physical state.
"""
import hashlib
import sys
from pathlib import Path
import numpy as np
from collision_world import pose_matrix
from provenance import canonical,digest,write_once
from live_sleep_readback import SystemReferences
_system_references=SystemReferences()

MAX_ROWS=100000
MAX_BYTES=1024**3
STAGES=('lowering','lowering_hold','lowering_settle','detach_before','detach_after',
        'opening_after','postrelease_wait','first_retreat','retreat_complete')


def native_observation(env,formal_env,model,component_type,arm,target,relative,held_id,drive,phase,step,preview_step,preview):
    """Read public native getters once per field; no mesh/clearance queries."""
    robots={};body_roles={}
    for side in ('left','right'):
        entity=getattr(env.robot,side+'_entity')
        q=entity.get_qpos().tolist();qdot=entity.get_qvel().tolist()
        controls=[]
        for joint in entity.get_active_joints():
            controls.append(dict(name=joint.get_name(),target=joint.get_drive_target().tolist(),
                velocity_target=joint.get_drive_velocity_target().tolist(),stiffness=joint.get_stiffness(),
                damping=joint.get_damping(),force_limit=joint.get_force_limit(),mode=str(joint.get_drive_mode())))
        robots[side]=dict(q=q,qdot=qdot,fingers=q[-2:],finger_velocities=qdot[-2:],controls=controls)
        for link in entity.get_links():body_roles[link]=dict(kind='robot_link',arm=side,link=link.get_name())
    objects={}
    for oid,obj in env.objects.items():
        pose=obj.get_pose();body=obj.actor.find_component_by_type(component_type)
        objects[str(oid)]=dict(pose=pose.p.tolist()+pose.q.tolist(),linear_velocity=body.get_linear_velocity().tolist(),
            angular_velocity=body.get_angular_velocity().tolist(),actor_name=obj.actor.get_name())
        body_roles[body]=dict(kind='object',object_id=int(oid))
    native=pose_matrix(env.get_arm_pose(arm));fk=model.evaluate(arm,robots[arm]['q'][:7])
    measured=np.linalg.inv(native)@pose_matrix(objects[str(target)]['pose'])
    contacts=[]
    for contact in env.scene.get_contacts():
        bodies=[dict(body_roles.get(b,dict(kind='static_or_unmapped')),python_body_token=str(id(b)),
            actor_name=b.entity.get_name(),native_type=type(b).__name__) for b in contact.bodies]
        contacts.append(dict(bodies=bodies,points=[dict(separation=float(p.separation),impulse=p.impulse.tolist(),
            position=p.position.tolist(),normal=p.normal.tolist()) for p in contact.points]))
    return dict(physics_step=int(step),preview_step=int(preview_step),episode_phase=phase,
        scene_role='independent_preview' if preview or env is not formal_env else 'formal',
        scene_token=str(id(_system_references.retain(env))),environment_token=str(id(env)),
        arm=arm,object_id=int(target),held_id=held_id,drive_active=drive is not None,
        magnetic_drive_token=None if drive is None else str(id(drive)),robots=robots,objects=objects,
        native_ee=native.tolist(),fk_ee=dict(position=fk['ee_position_m'],rotation=fk['ee_rotation_world']),
        stored_held_transform=np.asarray(relative).tolist(),current_measured_transform=measured.tolist(),
        contacts=contacts)


class PlacementTrace:
    def __init__(self,output,target,binding,formal_scene_token,formal_environment_token,observe):
        self.output=Path(output);self.target=target;self.binding=binding;self.observe=observe
        self.scene_token=formal_scene_token;self.environment_token=formal_environment_token
        self.active=False;self.reached=False;self.completed=False;self.error=None;self.closed=False
        self.stage=None;self.context=None;self.arm=None;self.relative=None
        self.rows=0;self.bytes=0;self.hash=hashlib.sha256();self.previous=None
        self.file=(self.output/'placement-transition.jsonl').open('xb')
        self.header=dict(schema='placement-transition/30',kind='header',selected_object_id=target,binding=binding,
            formal_scene_token=formal_scene_token,formal_environment_token=formal_environment_token,
            max_measurement_rows=MAX_ROWS,max_measurement_bytes=MAX_BYTES,
            scope='actual extraction lowering including hold, release, wait, first retreat; no native success credit')
        self._write(self.header,force=True)
    def _fail(self,exc):
        if self.error is None:
            self.error=type(exc).__name__+':'+str(exc)[:1024]
            print('placement_transition_trace_failed:'+self.error,file=sys.stderr)
        self.active=False
    def _write(self,row,force=False):
        row=dict(row,sequence=self.rows,previous_sha256=self.previous)
        row['row_sha256']=digest(row);data=canonical(row)
        if not force and (self.rows>=MAX_ROWS or self.bytes+len(data)>MAX_BYTES):
            raise ValueError('placement_trace_recording_cap')
        self.file.write(data);self.file.flush();self.hash.update(data)
        self.rows+=1;self.bytes+=len(data);self.previous=row['row_sha256']
    def begin(self,target,arm,relative,context):
        if target!=self.target:return
        try:
            if self.reached:raise ValueError('placement_trace_repeated_actual_transition')
            self.reached=True;self.active=True;self.arm=arm;self.relative=np.asarray(relative).copy()
            self.context=dict(context,stored_held_transform=self.relative.tolist());self.mark('lowering')
        except Exception as exc:self._fail(exc)
    def _record(self,kind,**extra):
        if not self.active:return
        try:
            observed=self.observe(self.arm,self.target,self.relative)
            if (observed['scene_role']!='formal' or observed['scene_token']!=self.scene_token
                    or observed['environment_token']!=self.environment_token):
                # This separate file cannot be interpreted as formal tick history.
                write_once(self.output/'placement-transition-foreign-scene.json',dict(kind='foreign_scene_rejected',observation=observed,formal_credit=False))
                raise ValueError('placement_trace_foreign_scene')
            self._write(dict(kind=kind,stage=self.stage,context_sha256=digest(self.context),observation=observed,**extra))
        except Exception as exc:self._fail(exc)
    def mark(self,stage):
        if not self.active:return
        if stage not in STAGES:self._fail(ValueError('placement_trace_stage'));return
        self.stage=stage;self._record('stage',context=self.context if stage=='lowering' else None)
        if stage=='retreat_complete' and self.error is None:self.completed=True;self.active=False
    def tick(self,target,held):
        if not self.active:return
        if target!=self.target:self._fail(ValueError('placement_trace_tick_target'));return
        self._record('tick',tick_held_argument=bool(held))
    def exception(self,reason):
        self._record('exception',reason=reason)
    def finish(self,phase,step,failure):
        if self.closed:return
        self.closed=True
        try:
            if self.active:self._record('finalization',reason=failure or 'unclosed_transition')
            self._write(dict(kind='footer',episode_phase=phase,physics_step=step,episode_failure=failure,
                requested_object_reached=self.reached,transition_completed=self.completed,error=self.error),force=True)
            self.file.close()
            write_once(self.output/'placement-transition-receipt.json',dict(schema='placement-transition-receipt/30',
                selected_object_id=self.target,binding=self.binding,rows=self.rows,bytes=self.bytes,
                sha256=self.hash.hexdigest(),last_row_sha256=self.previous,header_sha256=digest(self.header),
                requested_object_reached=self.reached,transition_completed=self.completed,
                recording_complete=self.error is None and self.reached,error=self.error,
                episode_failure=failure,formal_credit=False,physical_completion_proved=False))
        except Exception as exc:
            self._fail(exc)
            self.file.close()
