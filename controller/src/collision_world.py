"""Conservative per-mesh OBB checks using an explicit SAT implementation.

No robotics collision library. This checks sampled configurations, not swept
continuous collision or physical force closure. Initial overlaps fail closed.
"""
import math
import xml.etree.ElementTree as ET
import numpy as np
from direct_core import origin


def pose_matrix(pose):
    p = np.asarray(pose, dtype=float)
    if p.shape != (7,) or not np.isfinite(p).all():
        raise ValueError('invalid pose')
    norm = np.linalg.norm(p[3:])
    if norm < 1e-12:
        raise ValueError('zero quaternion')
    w,x,y,z = p[3:]/norm
    t=np.eye(4)
    t[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
              [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
              [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    t[:3,3]=p[:3]
    return t


def box(minimum, maximum, transform):
    lo,hi,t=np.asarray(minimum),np.asarray(maximum),np.asarray(transform)
    return (t[:3,:3]@((lo+hi)/2)+t[:3,3], t[:3,:3], (hi-lo)/2)


def overlaps(a,b,margin=0.):
    ca,ra,ha=a;cb,rb,hb=b
    # Necessary world-axis overlap test before the more expensive 15-axis SAT.
    # These are enclosing AABBs, so a rejection cannot hide an OBB collision.
    if np.any(np.abs(ca-cb)>np.abs(ra)@ha+np.abs(rb)@hb+margin+1e-12):
        return False
    axes=[ra[:,i] for i in range(3)]+[rb[:,i] for i in range(3)]
    axes += [np.cross(ra[:,i],rb[:,j]) for i in range(3) for j in range(3)]
    for axis in axes:
        n=np.linalg.norm(axis)
        if n < 1e-10:continue
        axis=axis/n
        reach=np.abs(ra.T@axis)@ha + np.abs(rb.T@axis)@hb
        if abs(axis@(cb-ca)) > reach+margin:return False
    return True


def cuboid(center,full):
    return np.asarray(center,dtype=float),np.eye(3),np.asarray(full,dtype=float)/2


class CollisionWorld:
    def __init__(self,model,geometry,state):
        self.model,self.geometry,self.state=model,geometry,state
        self.disabled={frozenset(p) for p in geometry['disabled_self_pairs']}
        self.objects={oid:[box(v['min'],v['max'],pose_matrix(info['pose']))
                           for v in geometry['objects'][oid]['parts']]
                      for oid,info in state['objects'].items()}
        # Same physical container and catch tray walls as Real10, including table.
        cx,cy=-.04,-.10;hx,hy=.76*.7/2,.61*.7/2
        self.static=[('table',cuboid([0,0,.72],[1.2,.7,.04])),
                     ('floor',cuboid([cx,cy,.746],[2*hx+.024,2*hy+.024,.012]))]
        for name,c,d in [('wall_x-',[cx-hx-.006,cy,.827],[.012,2*hy+.024,.15]),
                         ('wall_x+',[cx+hx+.006,cy,.827],[.012,2*hy+.024,.15]),
                         ('wall_y-',[cx,cy-hy-.006,.782],[2*hx,.012,.06]),
                         ('wall_y+',[cx,cy+hy+.006,.827],[2*hx,.012,.15])]:
            self.static.append((name,cuboid(c,d)))
        for side,x,width in [('left',-.58,.56),('right',.48,.40)]:
            self.static.append((side+'_tray',cuboid([x,-.20,.725],[width,1.10,.03])))
            for name,c,d in [('inner',[x+(width/2 if side=='left' else -width/2),-.20,.78],[.02,1.10,.08]),
                             ('outer',[x+(-width/2 if side=='left' else width/2),-.20,.78],[.02,1.10,.08]),
                             ('front',[x,-.75,.78],[width,.02,.08]),
                             ('back',[x,.35,.78],[width,.02,.08])]:
                self.static.append((side+'_'+name,cuboid(c,d)))
        self.last_collision=None
        self.held=None

    def robot_boxes(self,arm,q,finger_q=.04):
        transforms=self.model.link_transforms(arm,list(q),finger_q)
        return [(r['link'],box(r['min'],r['max'],np.asarray(transforms[r['link']])@
                 np.asarray(origin(ET.Element('origin',r['origin']))))) for r in self.geometry['robot_boxes']]

    def free(self,arm,q,target_id=None,allow_target_fingers=False,finger_q=.04):
        boxes=self.robot_boxes(arm,q,finger_q)
        objects=self.objects
        payload=None
        if self.held is not None:
            held_arm,oid,relative=self.held
            if held_arm!=arm:raise ValueError('only held arm can move')
            p=self.model.evaluate(arm,list(q));t=np.eye(4)
            t[:3,:3]=p['ee_rotation_world'];t[:3,3]=p['ee_position_m']
            payload=[box(v['min'],v['max'],t@relative) for v in self.geometry['objects'][oid]['parts']]
            objects={k:v for k,v in self.objects.items() if k!=oid}
        other='left' if arm=='right' else 'right'
        other_q=self.state['robots'][other]['q'][:7]
        other_boxes=self.robot_boxes(other,other_q)
        def fail(a,b):
            self.last_collision=[a,b]
            return False
        for i,(name,a) in enumerate(boxes):
            for name2,b in boxes[i+1:]:
                if frozenset((name,name2)) not in self.disabled and overlaps(a,b):return fail(name,name2)
            for name2,b in other_boxes:
                if overlaps(a,b,.001):return fail(arm+':'+name,other+':'+name2)
            for name2,b in self.static:
                if name=='panda_link0' and name2=='table':continue # fixed mount only
                if overlaps(a,b,.001):return fail(name,name2)
            for oid,parts in objects.items():
                if allow_target_fingers and str(target_id)==oid and name in ('panda_leftfinger','panda_rightfinger'):continue
                if any(overlaps(a,b,.001) for b in parts):return fail(name,'object_'+oid)
        if payload is not None:
            for a in payload:
                for name2,b in self.static+other_boxes:
                    if overlaps(a,b,.001):return fail('payload',name2)
                for oid,parts in objects.items():
                    if any(overlaps(a,b,.001) for b in parts):return fail('payload','object_'+oid)
                for name2,b in boxes:
                    if name2 not in ('panda_leftfinger','panda_rightfinger') and overlaps(a,b):
                        return fail('payload',name2)
        self.last_collision=None
        return True
