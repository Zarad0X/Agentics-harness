"""Conservative horizontal support contact and exact-key query acceleration."""
from collections import OrderedDict
import numpy as np
from mesh_world import MeshWorld
from collision_world import pose_matrix
from astra_planner import so3_log


class ReturnWorld(MeshWorld):
    def __init__(self,*args,supports=None,goals=None,optimize=False,**kwargs):
        super().__init__(*args,**kwargs)
        self.supports=supports or {};self.goals=goals or {};self.optimize=optimize
        self._shape_cache=OrderedDict();self.cache_hits=0

    def robot_shapes(self,arm,q,finger_q):
        if not self.optimize:return super().robot_shapes(arm,q,finger_q)
        key=(arm,tuple(q),tuple(np.atleast_1d(finger_q)))
        if key in self._shape_cache:
            self.cache_hits+=1;return self._shape_cache[key]
        result=super().robot_shapes(arm,q,finger_q);self._shape_cache[key]=result
        if len(self._shape_cache)>256:self._shape_cache.popitem(last=False)
        return result

    def intentional_support_contact(self,arm,q,oid,payload):
        if not self.held:return False
        _,target,relative=self.held
        if oid not in self.supports.get(target,[]) or target not in self.goals:return False
        fk=self.model.evaluate(arm,list(q));ee=np.eye(4);ee[:3,:3]=fk['ee_rotation_world'];ee[:3,3]=fk['ee_position_m']
        actual=ee@relative;goal=pose_matrix(self.goals[target])
        if np.linalg.norm(actual[:3,3]-goal[:3,3])>.003 or np.linalg.norm(so3_log(actual[:3,:3]@goal[:3,:3].T))>.04:return False
        # No blanket pair exclusion: only above a supporting horizontal plane.
        bottom=min(a[1][:,2].min() for a in payload)
        top=max(b[1][:,2].max() for b in self.object_shapes[oid])
        return bool(top-.0005<=bottom<=top+.001)
