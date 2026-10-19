"""Astra-authored grasp hypotheses from physical triangle-mesh cross sections.

Only NumPy geometry. No annotations, grasp pool, pretrained grasp model, or
robotics solver. A candidate is provisional until actual finger contact.
"""
import math
import numpy as np


def horizontal_section(vertices,faces,z):
    v=np.asarray(vertices,float);f=np.asarray(faces,int)
    tri=v[f];pieces=[]
    for a,b in ((0,1),(1,2),(2,0)):
        p,q=tri[:,a],tri[:,b];dz=q[:,2]-p[:,2]
        good=(np.abs(dz)>1e-10)&((p[:,2]-z)*(q[:,2]-z)<=0)
        if np.any(good):pieces.append(p[good]+((z-p[good,2])/dz[good])[:,None]*(q[good]-p[good]))
    if not pieces:return np.empty((0,3))
    return np.unique(np.round(np.concatenate(pieces),8),axis=0)


def mesh_grasps(pose,parts,depths=(.014,.025,.04,.06)):
    pose=np.asarray(pose,float)
    world=[(np.asarray(p['vertices'])@pose[:3,:3].T+pose[:3,3],p['faces']) for p in parts]
    top=max(v[:,2].max() for v,f in world)
    candidates=[];seen=set()
    for depth in depths:
        z=top-depth;sections=[horizontal_section(v,f,z) for v,f in world]
        nonempty=[s for s in sections if len(s)>=3]
        if not nonempty:continue
        # Whole slice first, then component slices (rim/handle/neck contacts).
        groups=[np.concatenate(nonempty)]+nonempty
        for group in groups:
            for k in range(8):
                angle=k*math.pi/8;closing=np.array([math.cos(angle),math.sin(angle),0.])
                approach=np.array([0.,0.,-1.]);across=np.cross(approach,closing)
                u=group@closing;v=group@across;width=float(np.ptp(u))
                if not .006<=width<=.076:continue
                center=closing*(u.min()+u.max())/2+across*(v.min()+v.max())/2
                center[2]=z
                key=tuple(np.round(center/.004).astype(int))+(k,)
                if key in seen:continue
                seen.add(key)
                rotation=np.column_stack((approach,closing,across))
                candidates.append(dict(ee_position_m=(center-.13*approach).tolist(),
                    ee_rotation_world=rotation.tolist(),closing_width_m=width,
                    whole_obb_fits=True,assisted_contact_hypothesis=True,
                    surface_depth_m=depth,section_center=center.tolist(),
                    source='physical_triangle_cross_section'))
    return candidates
