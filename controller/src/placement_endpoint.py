"""Candidate-only placement endpoint filter; no physical execution or recovery.

Preserve MeshWorld's 1 mm static margin while reserving the EXISTING 5 mm /
0.04 rad endpoint tracking allowance for panda_link7, rigidly fixed to the EE.
This proves only a static-obstacle endpoint statement in the frozen geometry.
It does not certify the lowering path, upstream links, release dynamics, HOME,
or the tracking bound during/after release. Existing checks remain required.
"""
import math
import numpy as np
from astra_planner import so3_log, POSITION_TOLERANCE_M, ROTATION_TOLERANCE_RAD
from return_clearance import separation_lower

# Literal existing physics_runner.plan_to tolerances and MeshWorld static margin.
POSITION_TRACKING_M = .005
ROTATION_TRACKING_RAD = .04
STATIC_MARGIN_M = .001

def assess_link7_endpoint(world, arm, q, position, rotation):
    p=np.asarray(position,float);r=np.asarray(rotation,float)
    if p.shape!=(3,) or r.shape!=(3,3) or not np.isfinite(p).all() or not np.isfinite(r).all():
        raise ValueError('placement_endpoint_pose')
    # so3_log validates proper orthonormal rotation.
    so3_log(r)
    # This derivation requires the link7 -> hand chain to remain rigid.
    for name,parent,child in (('panda_joint8','panda_link7','panda_link8'),
                              ('panda_hand_joint','panda_link8','panda_hand')):
        joint=world.model.root.find("joint[@name='%s']"%name)
        if joint is None or joint.get('type')!='fixed' or joint.find('parent').get('link')!=parent or joint.find('child').get('link')!=child:
            raise ValueError('placement_endpoint_rigid_chain')
    fk=world.model.evaluate(arm,list(q))
    center=np.asarray(fk['ee_position_m']);nominal_r=np.asarray(fk['ee_rotation_world'])
    pe=float(np.linalg.norm(center-p));re=float(np.linalg.norm(so3_log(r@nominal_r.T)))
    if pe>POSITION_TOLERANCE_M or re>ROTATION_TOLERANCE_RAD:
        raise ValueError('placement_endpoint_ik_target_mismatch')
    shapes=world.robot_shapes(arm,q,[.04,.04])
    matches=[shape for name,shape in shapes if name=='panda_link7']
    if len(matches)!=1:raise ValueError('placement_endpoint_link7_coverage')
    vertices=matches[0][1]
    radius=float(np.max(np.linalg.norm(vertices-center,axis=1)))
    # Triangle inequality for actual-to-target-to-nominal translation/rotation.
    reserve=POSITION_TRACKING_M+pe+2*radius*math.sin((ROTATION_TRACKING_RAD+re)/2)
    required=float(np.nextafter(STATIC_MARGIN_M+reserve,math.inf))
    rows=[]
    for name,shape in world.static_shapes:
        lower=separation_lower(vertices,shape[1],required)
        rows.append(dict(pair=['panda_link7',name],clearance_lower_m=lower,
                         required_m=required,safe=bool(lower>required)))
    if not rows:raise ValueError('placement_endpoint_static_coverage')
    return dict(schema='placement-link7-endpoint-candidate/1',safe=all(x['safe'] for x in rows),
                radius_m=radius,nominal_ik_position_error_m=pe,nominal_ik_rotation_error_rad=re,
                tracking_position_m=POSITION_TRACKING_M,tracking_rotation_rad=ROTATION_TRACKING_RAD,
                static_margin_m=STATIC_MARGIN_M,displacement_reserve_m=reserve,pairs=rows,
                conditional_on_measured_endpoint_tracking=True,physical_completion_proved=False)
