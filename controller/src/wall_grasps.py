"""Bounded extraction-only physical-mesh sections normal to an inward-tilted approach.

No change to source meshes, target pose, tool offset, width filter or physical
closure. Returned hypotheses still require complete robot/world and native tests.
"""
import copy,math
import numpy as np
from surface_grasps import mesh_grasps
def tilted_mesh_grasps(pose,parts,container_center,angle_degrees,depths=(.014,)):
 pose=np.asarray(pose,float);inward=np.asarray(container_center,float)-pose[:2,3]
 if np.linalg.norm(inward)<1e-9:return []
 inward/=np.linalg.norm(inward);theta=math.radians(angle_degrees)
 normal=np.r_[inward*math.sin(theta),math.cos(theta)]
 x=np.r_[-inward[1],inward[0],0.];y=np.cross(normal,x);frame=np.column_stack((x,y,normal))
 virtual=np.eye(4);virtual[:3,:3]=frame.T@pose[:3,:3];virtual[:3,3]=frame.T@pose[:3,3]
 result=[]
 for c in mesh_grasps(virtual,parts,depths):
  v=copy.deepcopy(c)
  v['ee_position_m']=(frame@np.asarray(c['ee_position_m'])).tolist();v['section_center']=(frame@np.asarray(c['section_center'])).tolist();v['ee_rotation_world']=(frame@np.asarray(c['ee_rotation_world'])).tolist()
  v['source']='physical_triangle_oblique_cross_section';v['approach_tilt_deg']=float(angle_degrees)
  result.append(v)
 return result


def append_inward_tilt(candidates, pose, parts, container_center):
    """Keep the complete existing prefix, then at most eight 20-degree sections.

    The tilt points the wrist toward the container interior, using the unchanged
    physical triangle section generator and its tool offset/width filter. This
    is only additional orientation coverage: every existing admission, native
    bilateral contact, closure, lift and return requirement remains mandatory.
    """
    pose = np.asarray(pose, dtype=float)
    center = np.asarray(container_center, dtype=float)
    if pose.shape != (4, 4) or center.shape != (2,):
        raise ValueError('invalid wall-grasp frame dimensions')
    if not np.isfinite(pose).all() or not np.isfinite(center).all():
        raise ValueError('nonfinite wall-grasp frame')
    additions = tilted_mesh_grasps(pose, parts, center, 20., (.014,))[:8]
    return list(candidates) + additions
