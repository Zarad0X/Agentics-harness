"""Pure pre-transport unloaded retreat forecast using the execution certificate.

Nominal and inherited compensation are hypotheses, not predicted native states.
The live retained controls and release-state certificate remain authoritative.
"""
import copy
import numpy as np
from loaded_common import digest
from return_clearance import ReturnClearance
from loaded_tracking import POLICY
import astra_planner as planner


def assess(world_factory,state,controls,arm,target,low_q,retreat_q,transform,path,rng):
    q=np.asarray(low_q,float);goal=np.asarray(retreat_q,float);t=np.asarray(transform,float)
    if q.shape!=(7,) or goal.shape!=(7,) or t.shape!=(4,4) or not all(np.isfinite(x).all() for x in (q,goal,t)):
        raise ValueError('placement_retreat_inputs')
    if not np.allclose(t[3],[0,0,0,1],rtol=0,atol=1e-12) or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),rtol=0,atol=1e-6) or abs(np.linalg.det(t[:3,:3])-1)>1e-6:
        raise ValueError('placement_retreat_transform')
    route=np.asarray(path,float)
    if route.ndim!=2 or route.shape[1]!=7 or not len(route) or not np.isfinite(route).all() or not np.array_equal(route[0],q) or not np.array_equal(route[-1],goal):
        raise ValueError('placement_retreat_path_binding')
    drive=np.array([j['target'][0] for j in controls[arm][:7]],float)
    offset=drive-np.asarray(state['robots'][arm]['q'][:7])
    if not np.isfinite(offset).all() or np.max(np.abs(offset))>POLICY['bias_limit_rad']:
        raise ValueError('placement_retreat_source_bias')
    future=copy.deepcopy(state);future['robots'][arm]['q']=q.tolist()+[.04,.04]
    world=world_factory(future)
    if world.held is not None:raise ValueError('placement_retreat_unloaded')
    world.set_object_transform(target,t)
    c=ReturnClearance(world,arm,target,True)
    limits=np.asarray(world.model.bounds)+[.03,-.03]
    forecasts=[]
    # Float32 control readback is what the live prefix actually retains.
    for label,bias in (('nominal',np.zeros(7)),('inherited_compensation',offset)):
        predicted=(q+bias).astype(np.float32).astype(float)
        valid=bool(np.all(predicted>=limits[:,0]) and np.all(predicted<=limits[:,1]))
        point=c.point(predicted.tolist()) if valid else None
        forecasts.append(dict(hypothesis=label,drive_q=predicted.tolist(),drive_bounds_valid=valid,point=point))
    original=route.tolist();replanned=False
    certificate=c.certify(original)
    # Search with the SAME reserve-aware predicate only if endpoints are viable.
    # Endpoint failures must advance placement/IK selection, never path retries.
    if (not certificate['certified'] and certificate['reason']!='endpoint_clearance'
            and all(f['drive_bounds_valid'] and f['point']['safe'] for f in forecasts)):
        candidate=planner.plan_joint_path(q,goal,limits,lambda x:c.point(x)['safe'],rng=rng,max_nodes=80,resolution=.025)
        if candidate['success']:
            route=np.asarray(candidate['path']);certificate=c.certify(route);replanned=True
    for f in forecasts:
        prefix=[f['drive_q']]+route.tolist()
        f['route']=prefix
        f['certificate']=c.certify(prefix) if f['drive_bounds_valid'] and f['point']['safe'] and certificate['certified'] else None
    safe=bool(certificate['certified'] and all(f['certificate'] is not None and f['certificate']['certified'] for f in forecasts))
    record=dict(schema='placement-unloaded-retreat/007',safe=safe,arm=arm,object_id=int(target),
        state=copy.deepcopy(state),state_sha256=digest(state),controls=copy.deepcopy(controls),controls_sha256=digest(controls),
        low_q=q.tolist(),retreat_q=goal.tolist(),predicted_transform=t.tolist(),fingers=[.04,.04],
        original_path=original,path=route.tolist(),path_sha256=digest(route.tolist()),replanned=replanned,
        geometry_sha256=digest(dict(geometry=world.geometry,meshes=world.meshes,robot_vertices=world.robot_vertices)),
        certificate=certificate,forecasts=forecasts,physical_completion_proved=False,
        scope='conditional unloaded whole route and retained-drive prefix forecasts; actual release revalidation required')
    return dict(record,sha256=digest(record))
