"""Short, strictly separating extraction from an initially narrow positive gap.

Existing sub-millimeter gaps may use their measured clearance temporarily.
An unresolved touching start additionally requires actual physics contacts
within 50 microns penetration and the unchanged 0.002 impulse gate. This
exception applies only to the exact initial joint state; every subsequent
sample must be disjoint. Other mesh intersections are rejected. Samples increase
or preserve clearance until the normal 1mm margin is reached; the final pose
must have at least 3mm clearance to those neighbors. Other collision rules are
unchanged. Physical contacts still have to be checked during execution.
"""
import numpy as np
import astra_planner as planner


def plan_separation(model,world,arm,q0,target,finger,*,rng,initial_contacts=None,validate_candidate=None):
    q0=np.asarray(q0,float)
    if world.free(arm,q0.tolist(),target,True,finger):
        return dict(success=True,needed=False,path=[q0.tolist()])
    collision=world.last_collision
    if collision[0]!='payload' or not collision[1].startswith('object_'):
        return dict(success=False,needed=True,reason='not_a_neighbor_clearance',collision=collision)
    near={k:v for k,v in world.payload_clearances(arm,q0.tolist()).items() if v<.001}
    touching={k for k,v in near.items() if v<=2e-5}
    evidence=initial_contacts or {}
    if not near or any(k not in evidence or not np.isfinite([evidence[k]['separation_min'],evidence[k]['impulse_max']]).all()
        or evidence[k]['separation_min']<-.00005
        or evidence[k]['impulse_max']>.002 for k in touching):
        return dict(success=False,needed=True,reason='initial_intersection_or_unresolved_gap',near=near)
    fk=model.evaluate(arm,q0.tolist());p=np.array(fk['ee_position_m']);r=np.array(fk['ee_rotation_world'])
    offsets=[np.array([0.,0.,.015])]
    for z in (.004,0.,.012):
        offsets.extend(np.array([.012*np.cos(a),.012*np.sin(a),z]) for a in np.arange(8)*np.pi/4)
    rejected=[]
    for offset in offsets:
        positions=[p+f*offset for f in np.linspace(0,1,7)[1:]]
        ik=planner.solve_ik_sequence(model,arm,positions,r,q0,rng=rng,max_iters=100)
        if not ik['success']:
            rejected.append('ik');continue
        path=[q0.tolist()]+ik['solutions'];previous=near.copy();good=True
        for a,b in zip(path,path[1:]):
            a,b=np.array(a),np.array(b)
            count=max(1,int(np.ceil(np.max(np.abs(b-a))/.002)))
            for f in np.linspace(0,1,count+1):
                q=(a+f*(b-a)).tolist()
                thresholds={k:max(1e-5,min(v,.001)-1e-6) for k,v in previous.items()}
                at_start=np.max(np.abs(np.asarray(q)-q0))<1e-10
                if not world.free(arm,q,target,True,finger,payload_clearance=thresholds,
                                  start_contact_pairs=touching if at_start else ()):
                    good=False;break
                measured=world.payload_clearances(arm,q,near)
                previous={k:max(previous[k],measured[k]) for k in previous}
            if not good:break
        if not good:
            rejected.append('nonseparating_or_collision');continue
        final=world.payload_clearances(arm,path[-1],near)
        if min(final.values())<.003 or not world.free(arm,path[-1],target,True,finger):
            rejected.append('insufficient_final_clearance');continue
        candidate=dict(success=True,needed=True,path=path,initial_clearance=near,
                    final_clearance=final,offset_m=offset.tolist(),position=(p+offset).tolist(),
                    rotation=r.tolist(),rejected=rejected,initial_contacts={k:evidence[k] for k in touching})
        if validate_candidate is not None and not validate_candidate(candidate):
            rejected.append('physics_preview_rejected');continue
        return candidate
    return dict(success=False,needed=True,reason='no_short_separating_motion',near=near,rejected=rejected)
