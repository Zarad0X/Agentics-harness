"""Native capture adapter. Reads current physics; writes finger drives only."""
import numpy as np
from types import SimpleNamespace
from capture_feedback import run as capture, raw_touch, need
from provenance import digest


def contact_points(scene,fingers,target_entity):
    links=[joint.child_link for joint in fingers]
    need(len(links)==2 and links[0]!=links[1],'finger_identity')
    result=[[],[]]
    for contact in scene.get_contacts():
        if len(contact.bodies)!=2:continue
        for index,link in enumerate(links):
            for a,b in ((contact.bodies[0],contact.bodies[1]),(contact.bodies[1],contact.bodies[0])):
                # Link handles select the requested arm, never shared names.
                # Target entity identity selects this object, never actor name.
                if a==link and b.entity==target_entity:
                    for p in contact.points:
                        result[index].append(dict(separation=float(p.separation),impulse=np.asarray(p.impulse).tolist()))
    return result


def run(*,env,arm,target,candidate,state,world,measurement,tick,guard,event,phase,returning,context):
    entity=getattr(env.robot,arm+'_entity');active=entity.get_active_joints()
    fingers=[j[0] for j in getattr(env.robot,arm+'_gripper')]
    need(all(j.child_link in entity.get_links() for j in fingers),'selected_arm_links')
    offset=0 if arm=='left' else len(env.robot.left_entity.get_active_joints())
    slots=[offset+active.index(j) for j in fingers]
    need(slots==([7,8] if arm=='left' else [16,17]),'joint_layout')
    def controls():
        return tuple((j.get_stiffness(),j.get_damping(),j.get_force_limit(),j.get_drive_mode(),
                      tuple(j.get_drive_target()),tuple(j.get_drive_velocity_target()))
                     for side in ('left','right') for j in getattr(env.robot,side+'_entity').get_active_joints())
    def observe():
        s=state();m=measurement();o=s['objects'][str(target)]
        points=contact_points(env.scene,fingers,env.objects[target].actor)
        q=s['robots'][arm]['q'];v=s['robots'][arm]['qdot']
        return dict(finger_position=[float(q[active.index(j)]) for j in fingers],
                    finger_velocity=[float(v[active.index(j)]) for j in fingers],points=points,touching=[any(-.0003<=p['separation']<=.0001 and np.isfinite(p['impulse']).all() and np.linalg.norm(p['impulse'])>1e-6 for p in ps) for ps in points],
                    position_error_m=m['position_error_m'],rotation_error_rad=m['rotation_error_rad'],
                    object_pose=o['pose'],object_linear_velocity=o['linear_velocity'],object_angular_velocity=o['angular_velocity'],
                    held_id=context()['held_id'],drive_active=context()['drive_active'])
    def issue(values):
        for j,value in zip(fingers,values):
            j.set_drive_target(value);j.set_drive_velocity_target(0.)
    def geometry(values):
        s=state();w=world(s);q=s['robots'][arm]['q'][:7]
        measured=[s['robots'][arm]['q'][active.index(j)] for j in fingers]
        previous=[float(j.get_drive_target()[0]) for j in fingers]
        # Sample the native finger-position to target interval and the command
        # interval at <=40 micrometres. Target-finger contact is the sole allowed
        # target exception; all other objects/robot links use existing geometry.
        samples=[];seen=set()
        for start in (measured,previous):
            count=max(2,int(np.ceil(np.max(np.abs(np.asarray(values)-start))/.00004))+1)
            if count>1002:return None
            for f in np.linspace(start,values,count):
                key=tuple(f.tolist())
                if key in seen:continue
                seen.add(key)
                if not w.free(arm,q,target,True,f.tolist()):return None
                samples.append(f.tolist())
        compact=dict(robots=s['robots'],objects={k:dict(pose=o['pose']) for k,o in s['objects'].items()})
        return dict(state=compact,state_sha256=digest(compact),samples=samples,
                    previous_targets=previous,targets=list(values))
    source=dict(arm=arm,object_id=target,slots=slots,candidate_sha256=digest(candidate),
                phase=phase,returning=returning,context=context(),
                finger_names=[j.child_link.get_name() for j in fingers],
                contact_identity='selected_native_child_links_and_target_entity',
                desired_position=candidate['ee_position_m'],desired_rotation=candidate['ee_rotation_world'])
    def closure_source():return dict(arm=arm,object_id=target,candidate_sha256=digest(candidate),state=state(),
        desired_position=candidate['ee_position_m'],desired_rotation=candidate['ee_rotation_world'])
    api=SimpleNamespace(dt=env.scene.get_timestep(),joints=fingers,controls=controls,observe=observe,
                        issue=issue,geometry=geometry,guard=guard,tick=tick,closure_source=closure_source,
                        emit=event,legacy_emit=lambda kind,**kw:event(kind,object_id=target,arm=arm,**kw))
    from capture_handoff import handoff
    receipt=handoff(env,arm,target,candidate,state,event,context(),guard)
    source['gripper_handoff_sha256']=receipt['sha256']
    return capture(api,source)
