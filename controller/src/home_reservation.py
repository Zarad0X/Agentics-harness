"""Pure temporary-slot admission using the SAME live HOME contact model.

Only hypothetical collision worlds are changed. No scene, controls, object pose
or velocity setters. Both canonical HOME poses must admit either active role,
including its 6 mm tube and all existing step/world/parked reserves. The normal
loaded, lowering and release-retreat preflights are still independently needed.
"""
import copy
import numpy as np
from provenance import digest
from return_clearance import ReturnClearance


def assess(world_factory, state, homes, envelope, target=None, transform=None):
    if set(homes)!={'left','right'}:raise ValueError('reservation_home_coverage')
    future=copy.deepcopy(state)
    for arm in ('left','right'):
        q=np.asarray(homes[arm],float)
        if q.shape!=(7,) or not np.isfinite(q).all():raise ValueError('reservation_home_q')
        future['robots'][arm]['q']=q.tolist()+[.04,.04]
    if transform is not None:
        t=np.asarray(transform,float)
        if (str(target) not in future['objects'] or t.shape!=(4,4) or not np.isfinite(t).all()
                or not np.allclose(t[3],[0,0,0,1],rtol=0,atol=1e-12)
                or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),rtol=0,atol=1e-6)
                or abs(np.linalg.det(t[:3,:3])-1)>1e-6):raise ValueError('reservation_transform')
    world=world_factory(future)
    if world.held is not None:raise ValueError('reservation_world_must_be_unloaded_hypothesis')
    envelope.attach(world)
    if transform is not None:world.set_object_transform(target,t)
    roles={a:ReturnClearance(world,a,target).point(homes[a]) for a in ('left','right')}
    return dict(schema='home-workspace-reservation/22',safe=all(r['safe'] for r in roles.values()),
        roles=roles,homes=copy.deepcopy(homes),state=copy.deepcopy(state),state_sha256=digest(state),
        predicted_object_id=None if transform is None else int(target),
        predicted_transform=None if transform is None else t.tolist(),model_sha256=envelope.sha256,
        conditional_on_measured_placement=True,formal_credit=False)


class Reservation:
    def __init__(self,env,world_factory,state,homes,event):
        from home_interaction import RuntimeEnvelope
        self.runtime=RuntimeEnvelope(env,world_factory(state));self.world_factory=world_factory
        self.homes=copy.deepcopy(homes);self.event=event
        event('home_workspace_model',model=self.runtime.envelope.record,
              model_sha256=self.runtime.envelope.sha256,homes=self.homes,state_sha256=digest(state),
              scope='temporary placement reserves both canonical HOME roles; full existing reserves',formal_credit=False)

    def check(self,state,target,stage,transform=None):
        self.runtime.check()
        receipt=assess(self.world_factory,state,self.homes,self.runtime.envelope,target,transform)
        self.event('home_workspace_reservation',object_id=target,stage=stage,**receipt)
        return receipt['safe']
