"""Read-only additional observation, identical on control and candidate.
The old scoped filter remains unchanged. Impulses are not force readbacks.
"""
def observe_target_contacts(env,target,contacts):
    try:actor=env.objects[target].actor
    except (KeyError,AttributeError):
        return dict(schema='target-identity-contacts/1',available=False,reason='scene_actor_unavailable',target=int(target),pairs=None)
    pairs=[]
    for ordinal,c in enumerate(contacts):
        identities=[b.entity is actor or b.entity==actor for b in c.bodies]
        if not any(identities):continue
        pairs.append(dict(scene_pair_index=ordinal,bodies=[b.entity.get_name() for b in c.bodies],target_body_indices=[i for i,x in enumerate(identities) if x],
            points=[dict(position=p.position.tolist(),normal=p.normal.tolist(),separation=float(p.separation),impulse=p.impulse.tolist()) for p in c.points]))
    return dict(schema='target-identity-contacts/1',available=True,target=int(target),actor_name=actor.get_name(),
        identity_basis='env.objects[target].actor entity identity/equality; names never select',
        total_scene_pairs=len(contacts),target_pairs=len(pairs),target_points=sum(len(c['points']) for c in pairs),pairs=pairs,
        measurement='native contact impulses and geometry; no threshold or point truncation',
        force_readback=None,force_readback_status='unavailable; impulse is not instantaneous force',
        configured_limits_are_measurements=False)
