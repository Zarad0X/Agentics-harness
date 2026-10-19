"""Read the cooked convex hulls actually used by the physics engine.

Source asset triangles are not necessarily identical to PhysX's cooked hull.
Keep scaling and per-shape local poses in actor coordinates for all queries.
This module reads shapes only; it never changes a collision shape or actor.
"""
import numpy as np


def convex_parts(shapes):
    parts=[]
    for shape in shapes:
        vertices=np.asarray(shape.get_vertices(),dtype=float)
        faces=np.asarray(shape.get_triangles(),dtype=int).reshape(-1,3)
        scale=np.asarray(shape.get_scale(),dtype=float)
        local=np.asarray(shape.get_local_pose().to_transformation_matrix(),dtype=float)
        if (vertices.ndim!=2 or vertices.shape[1]!=3 or len(vertices)<4
                or scale.shape!=(3,) or local.shape!=(4,4)
                or not np.isfinite(vertices).all() or not np.isfinite(scale).all()
                or not np.isfinite(local).all() or np.any(scale<=0)
                or not len(faces) or faces.min()<0 or faces.max()>=len(vertices)):
            raise ValueError('invalid_physics_convex_shape')
        points=(vertices*scale)@local[:3,:3].T+local[:3,3]
        parts.append(dict(vertices=points.tolist(),faces=faces.tolist()))
    if not parts:raise ValueError('missing_physics_convex_shapes')
    return parts


def bounds_for_parts(parts):
    return [dict(min=np.asarray(p['vertices']).min(0).tolist(),
                 max=np.asarray(p['vertices']).max(0).tolist()) for p in parts]
