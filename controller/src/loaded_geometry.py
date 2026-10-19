"""Exact named-shape binding, independent of JSON object insertion order."""
import numpy as np
from loaded_common import digest
SCHEMA='named-loaded-obstacles/1'
def obstacle_rows(obstacles):
    rows=[];seen=set()
    for name,vertices in obstacles:
        if not isinstance(name,str) or not name or name in seen:raise ValueError('loaded_duplicate_obstacle_identity')
        seen.add(name);v=np.asarray(vertices)
        if v.ndim!=2 or v.shape[1]!=3 or not len(v) or not np.isfinite(v).all():raise ValueError('loaded_obstacle_vertices')
        rows.append((name,v.tolist()))
    return rows

def binding(obstacles):
    rows=obstacle_rows(obstacles)
    return dict(obstacle_schema=SCHEMA,obstacle_order=[n for n,v in rows],obstacles=digest(sorted(rows,key=lambda r:r[0])))

def verify(admission,b,legacy=False):
    rows=obstacle_rows(admission.obstacles)
    if legacy:
        if any(k in b for k in ('obstacle_schema','obstacle_order')):raise ValueError('loaded_legacy_obstacle_schema')
        if digest(rows)!=b['obstacles']:raise ValueError('loaded_obstacle_hash')
        return
    if b.get('obstacle_schema')!=SCHEMA:raise ValueError('loaded_obstacle_schema')
    order=b.get('obstacle_order');names=[n for n,v in rows]
    if not isinstance(order,list) or len(order)!=len(names) or len(set(order))!=len(order) or set(order)!=set(names):raise ValueError('loaded_obstacle_identity_coverage')
    if digest(sorted(rows,key=lambda r:r[0]))!=b['obstacles']:raise ValueError('loaded_obstacle_hash')
    lookup=dict(admission.obstacles);admission.obstacles=[(n,lookup[n]) for n in order]
    admission.obstacle_bounds=[(n,v,v.min(0),v.max(0)) for n,v in admission.obstacles]
