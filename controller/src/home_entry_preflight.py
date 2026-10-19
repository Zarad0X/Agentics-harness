"""Necessary nominal retreat-to-HOME condition, before committed transport.

Use the existing HOME contact envelope and full world. This is not a swept
retreat/HOME certificate, physical forecast, or measured tracking guarantee.
No solver, RNG, execution, margin change, or geometry exclusion is introduced.
"""
import copy
import numpy as np
from provenance import digest
from return_clearance import ReturnClearance


def assess(world_factory, state, envelope, arm, target, retreat_q, transform):
    if arm not in ('left','right') or set(state['robots']) != {'left','right'}:
        raise ValueError('placement_HOME_entry_arms')
    q=np.asarray(retreat_q,float);t=np.asarray(transform,float)
    if q.shape!=(7,) or not np.isfinite(q).all():
        raise ValueError('placement_HOME_entry_q')
    if (str(target) not in state['objects'] or t.shape!=(4,4) or not np.isfinite(t).all()
            or not np.allclose(t[3],[0,0,0,1],rtol=0,atol=1e-12)
            or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),rtol=0,atol=1e-6)
            or abs(np.linalg.det(t[:3,:3])-1)>1e-6):
        raise ValueError('placement_HOME_entry_transform')
    future=copy.deepcopy(state)
    # Only the moving arm is hypothesized; retain actual parked-arm geometry.
    future['robots'][arm]['q']=q.tolist()+[.04,.04]
    world=world_factory(future)
    if world.held is not None:raise ValueError('placement_HOME_entry_requires_unloaded_world')
    envelope.attach(world)
    world.set_object_transform(target,t)
    point=ReturnClearance(world,arm,target,False).point(q.tolist())
    record=dict(schema='placement-HOME-entry-preflight/1',safe=point['safe'],
        arm=arm,object_id=int(target),retreat_q=q.tolist(),fingers=[.04,.04],
        state=copy.deepcopy(state),state_sha256=digest(state),
        predicted_transform=t.tolist(),home_contact_model_sha256=envelope.sha256,
        point=point,scope='necessary nominal HOME start condition only',
        conditional_on_predicted_placement=True,physical_completion_proved=False)
    return dict(record,sha256=digest(record))
