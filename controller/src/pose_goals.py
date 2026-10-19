"""Payload-frame goals, raw SE(3) errors, conservative support/insertion graph."""
import math
import numpy as np
from collision_world import pose_matrix


def pose_errors(actual,goal):
    a=np.asarray(actual,float);g=np.asarray(goal,float)
    pose_matrix(a);pose_matrix(g)
    qa=a[3:]/np.linalg.norm(a[3:]);qg=g[3:]/np.linalg.norm(g[3:])
    # q and -q represent the same rotation. atan2 remains stable near zero/pi.
    d=min(np.linalg.norm(qa-qg),np.linalg.norm(qa+qg))
    angle=4*math.asin(min(1.,float(d)/2))
    return float(np.linalg.norm(a[:3]-g[:3])),math.degrees(angle)


def ee_for_payload(goal_pose,measured_ee_to_payload):
    rel=np.asarray(measured_ee_to_payload,float)
    if rel.shape!=(4,4) or not np.isfinite(rel).all(): raise ValueError('invalid grasp transform')
    if not np.allclose(rel[3],[0,0,0,1]) or not np.allclose(rel[:3,:3].T@rel[:3,:3],np.eye(3),atol=1e-6) or np.linalg.det(rel[:3,:3])<.999:
        raise ValueError('invalid rigid transform')
    return pose_matrix(goal_pose)@np.linalg.inv(rel)


def mesh_bounds(pose,parts):
    t=pose_matrix(pose)
    v=np.concatenate([np.asarray(p['vertices']) for p in parts])@t[:3,:3].T+t[:3,3]
    return v.min(0),v.max(0)


def dependencies(objects,meshes,contacts):
    """Support before supported object; lower overlapping footprint before upper.

    Conservative vertical insertion only. Cycles/interlocking are infeasible,
    not permission to omit objects or move their frozen goals.
    """
    bounds={i:mesh_bounds(o['pose'],meshes[i]) for i,o in objects.items()}
    edges={i:set() for i in objects};supports={i:set() for i in objects}
    names={o['actor_name']:i for i,o in objects.items()}
    for c in contacts:
        a,b=(names.get(n) for n in c['bodies'])
        if a is None or b is None or a==b: continue
        # Only a near-vertical normal establishes gravity support.
        if not any(abs(p['normal'][2])>=.65 and p['separation']<=.001 for p in c.get('points',[])): continue
        low,high=sorted((a,b),key=lambda i:objects[i]['pose'][2])
        if abs(objects[low]['pose'][2]-objects[high]['pose'][2])<.002: continue
        edges[high].add(low);supports[high].add(low)
    ids=sorted(objects)
    for k,a in enumerate(ids):
        for b in ids[k+1:]:
            al,ah=bounds[a];bl,bh=bounds[b]
            if np.all(np.minimum(ah[:2],bh[:2])-np.maximum(al[:2],bl[:2])>.001):
                low,high=sorted((a,b),key=lambda i:(bounds[i][0][2],i))
                if bounds[high][0][2]>bounds[low][0][2]+.002: edges[high].add(low)
    return {i:sorted(v) for i,v in edges.items()},{i:sorted(v) for i,v in supports.items()}


def insertion_order(deps,selected,already_present=()):
    pending=set(map(str,selected));present=set(map(str,already_present));order=[]
    while pending:
        ready=sorted(i for i in pending if set(deps[i])<=present)
        if not ready: raise RuntimeError('infeasible_support_or_insertion_cycle:'+str(sorted(pending)))
        oid=ready[0];pending.remove(oid);present.add(oid);order.append(oid)
    return order
