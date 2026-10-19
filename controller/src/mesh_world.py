"""Sampled robot/world collision checks with custom GJK narrow phase."""
import xml.etree.ElementTree as ET
import numpy as np
from collision_world import CollisionWorld,box,pose_matrix,overlaps
from convex_geometry import convex_distance,box_vertices
from direct_core import origin

class MeshWorld(CollisionWorld):
    def __init__(self,model,geometry,state,meshes,robot_vertices):
        super().__init__(model,geometry,state)
        self.meshes,self.robot_vertices=meshes,robot_vertices
        self.object_shapes={}
        for oid,info in state['objects'].items():
            t=pose_matrix(info['pose'])
            self.object_shapes[oid]=[(b,np.asarray(v['vertices'])@t[:3,:3].T+t[:3,3]) for b,v in zip(self.objects[oid],meshes[oid])]
        self.static_shapes=[(n,(b,box_vertices(b))) for n,b in self.static]

    def robot_shapes(self,arm,q,finger_q):
        transforms=self.model.link_transforms(arm,list(q),finger_q)
        result=[]
        for r in self.geometry['robot_boxes']:
            t=np.asarray(transforms[r['link']])@np.asarray(origin(ET.Element('origin',r['origin'])))
            vertices=np.asarray(self.robot_vertices[r['link']])@t[:3,:3].T+t[:3,3]
            result.append((r['link'],(box(r['min'],r['max'],t),vertices)))
        return result

    def set_object_transform(self,oid,transform):
        """Change only this collision query's hypothetical object placement."""
        oid=str(oid);t=np.asarray(transform,float)
        self.object_shapes[oid]=[(box(g['min'],g['max'],t),
            np.asarray(m['vertices'])@t[:3,:3].T+t[:3,3])
            for g,m in zip(self.geometry['objects'][oid]['parts'],self.meshes[oid])]

    def payload_shapes(self,arm,q):
        ha,oid,rel=self.held
        if ha!=arm:raise ValueError('held arm mismatch')
        fk=self.model.evaluate(arm,list(q));t=np.eye(4)
        t[:3,:3]=fk['ee_rotation_world'];t[:3,3]=fk['ee_position_m'];t=t@rel
        return [(box(g['min'],g['max'],t),np.asarray(m['vertices'])@t[:3,:3].T+t[:3,3])
            for g,m in zip(self.geometry['objects'][oid]['parts'],self.meshes[oid])]

    def payload_clearances(self,arm,q,ids=None):
        """Actual convex distances, capped at 1cm outside the broad phase."""
        payload=self.payload_shapes(arm,q);held_id=self.held[1];result={}
        for oid,parts in self.object_shapes.items():
            if oid==held_id or (ids is not None and oid not in ids):continue
            result[oid]=min([.01]+[convex_distance(a[1],b[1])
                for a in payload for b in parts if overlaps(a[0],b[0],.01)])
        return result

    @staticmethod
    def hit(a,b,margin=0.):
        if not overlaps(a[0],b[0],margin):return False
        return convex_distance(a[1],b[1])<=max(margin,1e-7)

    def free(self,arm,q,target_id=None,allow_target_fingers=False,finger_q=None,payload_clearance=None,start_contact_pairs=()):
        if finger_q is None:finger_q=np.clip(self.state['robots'][arm]['q'][-2:],0.,.04).tolist()
        robot=self.robot_shapes(arm,q,finger_q)
        other='right' if arm=='left' else 'left'
        other_q=self.state['robots'][other]['q']
        other_robot=self.robot_shapes(other,other_q[:7],np.clip(other_q[-2:],0.,.04).tolist())
        objects=self.object_shapes;payload=None
        if self.held:
            ha,oid,rel=self.held
            payload=self.payload_shapes(arm,q)
            objects={k:v for k,v in objects.items() if k!=oid}
        def fail(a,b):self.last_collision=[a,b];return False
        for i,(name,a) in enumerate(robot):
            for n,b in robot[i+1:]:
                if frozenset((name,n)) not in self.disabled and self.hit(a,b):return fail(name,n)
            for n,b in other_robot:
                if self.hit(a,b,.001):return fail(arm+':'+name,other+':'+n)
            for n,b in self.static_shapes:
                if name=='panda_link0' and n=='table':continue
                if self.hit(a,b,.001):return fail(name,n)
            for oid,parts in objects.items():
                if allow_target_fingers and str(target_id)==oid and 'finger' in name:continue
                if any(self.hit(a,b,.001) for b in parts):return fail(name,'object_'+oid)
        if payload is not None:
            for a in payload:
                for n,b in self.static_shapes:
                    # Resting on a horizontal support is an intentional contact;
                    # use actual hull vertices, never a shrunken box. Side/rim and
                    # deep floor intersection remain forbidden.
                    if n in ('table','floor','left_tray','right_tray'):
                        top=b[0][0][2]+b[0][2][2]
                        if a[1][:,2].min()>=top-.0005:continue
                    if self.hit(a,b,.001):return fail('payload',n)
                for n,b in other_robot:
                    if self.hit(a,b,.001):return fail('payload',other+':'+n)
                for oid,parts in objects.items():
                    if oid in start_contact_pairs:continue
                    if self.intentional_support_contact(arm,q,oid,payload):continue
                    margin=.001 if payload_clearance is None else payload_clearance.get(oid,.001)
                    if any(self.hit(a,b,margin) for b in parts):return fail('payload','object_'+oid)
                for n,b in robot:
                    if 'finger' not in n and self.hit(a,b):return fail('payload',n)
        self.last_collision=None;return True

    def intentional_support_contact(self,arm,q,oid,payload):
        return False
