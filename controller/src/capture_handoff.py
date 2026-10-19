"""Unloaded approach admission and explicit native capture command handoff.

Never steps physics or writes q/qvel, pose, force, or a scene. The retained
position command is reissued unchanged; only the selected drive's declared
capture properties and velocity command change. Failures never roll back.
"""
import copy
import math
from provenance import digest
from capture_feedback import need, probe
from closure_regulation import wire_controls

CAPTURE=(1000.,200.,4.,'force')
INITIAL=(2000.,280.,3.4028234663852886e38,'force')

def controls(env):
    return tuple((j.get_stiffness(),j.get_damping(),j.get_force_limit(),j.get_drive_mode(),
                  tuple(j.get_drive_target()),tuple(j.get_drive_velocity_target()))
                 for side in ('left','right') for j in getattr(env.robot,side+'_entity').get_active_joints())

def selected(env,arm):
    need(arm in ('left','right'),'handoff_arm')
    entity=getattr(env.robot,arm+'_entity');active=entity.get_active_joints()
    fingers=[j[0] for j in getattr(env.robot,arm+'_gripper')]
    offset=0 if arm=='left' else len(env.robot.left_entity.get_active_joints())
    need(len(fingers)==2 and fingers[0]!=fingers[1] and
         all(j in active and j.child_link in entity.get_links() for j in fingers),'handoff_identity')
    slots=[offset+active.index(j) for j in fingers]
    need(slots==([7,8] if arm=='left' else [16,17]),'handoff_layout')
    return fingers,slots

def validate(rows,slots,context):
    need(context['held_id'] is None and context['drive_active'] is False,'handoff_unloaded')
    need(len(rows)==18,'handoff_control_shape')
    for slot in slots:
        row=rows[slot]
        need(row[:4] in (CAPTURE,INITIAL),'handoff_properties')
        need(len(row[4])==len(row[5])==1 and
             all(math.isfinite(x) for x in (*row[4],*row[5])),'handoff_command_finite')
        need(0<=row[4][0]<=.04,'handoff_target_travel')
        # Bound set_gripper's 0.05 * delta over 40 mm travel (float32).
        need(abs(row[5][0])<=.0020000001,'handoff_velocity_envelope')

def preflight(env,arm,target,candidate,state,event,context):
    """Read-only rejection before IK/path execution, in formal and shadow."""
    receipt=dict(schema='capture-preflight/018-1',arm=arm,object_id=target,
                 candidate_sha256=digest(candidate),context=copy.deepcopy(context))
    try:
        _,slots=selected(env,arm);rows=controls(env)
        receipt.update(slots=slots,controls=probe(wire_controls(rows)),state_sha256=digest(state()))
        validate(rows,slots,context)
        receipt['status']='accepted'
    except Exception as exc:
        receipt.update(status='failed',failure=type(exc).__name__+':'+str(exc));raise
    finally:
        receipt['sha256']=digest(receipt);event('gripper_capture_preflight',receipt=receipt)
    return receipt

def handoff(env,arm,target,candidate,state,event,context,guard):
    receipt=dict(schema='capture-handoff/018-1',arm=arm,object_id=target,
                 candidate_sha256=digest(candidate),context=copy.deepcopy(context),
                 status='failed',writes_attempted=False)
    try:
        fingers,slots=selected(env,arm);before=controls(env);physical=copy.deepcopy(state())
        receipt.update(slots=slots,before_controls=probe(wire_controls(before)),before_state=probe(physical))
        validate(before,slots,context)
        guard()  # original native contact check before ANY drive write
        expected=list(before)
        for slot in slots:expected[slot]=(*CAPTURE,before[slot][4],(0.,))
        receipt['intended_controls']=probe(wire_controls(tuple(expected)))
        receipt['writes_attempted']=True
        for joint,slot in zip(fingers,slots):
            joint.set_drive_property(stiffness=1000.,damping=200.,force_limit=4.)
            joint.set_drive_target(before[slot][4][0])
            joint.set_drive_velocity_target(0.)
        need(controls(env)==tuple(expected),'handoff_readback')
        need(state()==physical,'handoff_physical_state_changed')
        guard()  # no tick, contact relaxation, or velocity reset
        receipt['status']='accepted'
    except Exception as exc:
        receipt['failure']=type(exc).__name__+':'+str(exc);raise
    finally:
        # Preserve partial writes and invalid inputs in failure evidence.
        receipt['after_controls']=probe(wire_controls(controls(env)))
        receipt['after_state']=probe(state())
        receipt['sha256']=digest(receipt)
        event('gripper_capture_handoff',receipt=receipt)
    return receipt
