"""Conservative geometry certificates for unloaded return retreat/home ONLY.

For each link/joint, R bounds every collision vertex's distance to that axis
at EVERY allowed configuration: sum downstream origin lengths, maximum
prismatic travel, and maximum local collision-vertex norm. Thus displacement
is <= sum R_j*abs(dq_j), with unit coefficients for prismatic translations.
Common ancestors cancel for self-pair distances. A convex support projection
gives a distance LOWER bound (the ordinary GJK distance estimate is not used
as a certificate). Adaptive interval subdivision proves the whole joint-linear
polyline, hence its monotone quintic time parameterization, with tracking and
one-step motion reserves. Unknown/over-budget intervals fail closed.

This is a geometry certificate CONDITIONAL on the declared tracking, obstacle
drift and speed envelopes. Checking velocities at physics boundaries is not a
proof of unknown between-step PD acceleration or hidden solver behavior.
"""
import runtime_profile as timing
import math
import xml.etree.ElementTree as ET
import numpy as np
from convex_geometry import closest_simplex
from direct_core import origin
from collision_world import pose_matrix

DT=.004
MAX_VELOCITY=.1
MAX_ACCELERATION=.4
TRACK_Q=.01
TRACK_FINGER=.001
LINK_TUBE_M=.006
WORLD_DRIFT_M=.003
OBS_SPEED=.03
OBS_SPIN=.15
MEASURED_SPEED=.15
FINGER_SPEED=.01
TRACK_BOX=np.array([TRACK_Q]*7+[TRACK_FINGER]*2)
SPEED_BOX=np.array([MEASURED_SPEED]*7+[FINGER_SPEED]*2)
# Additional HOME-only parked-arm/object contract. Existing pairs and guards
# retain their original 6mm/.15rad/s reserves. No actuator settings change.
PARKED_TUBE_M=.0005
PARKED_SPEED_BOX=np.array([.01]*7+[.001]*2)


def separation_lower(a,b,required=0.):
    """Any returned positive value is a separating-projection lower bound."""
    a=np.asarray(a,float);b=np.asarray(b,float)
    if a.ndim!=2 or b.ndim!=2 or a.shape[1]!=3 or b.shape[1]!=3 or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError('invalid convex vertices')
    gap=np.maximum(np.maximum(a.min(0)-b.max(0),b.min(0)-a.max(0)),0)
    lower=max(0.,float(np.linalg.norm(gap))-1e-9)
    if lower>required:return lower
    def support(d):return a[np.argmax(a@d)]-b[np.argmin(b@d)]
    v=a.mean(0)-b.mean(0)
    if np.linalg.norm(v)<1e-12:v=np.array([1.,0,0])
    simplex=np.array([support(v)]);v=simplex[0]
    for _ in range(48):
        norm=float(np.linalg.norm(v))
        if norm<1e-12:return lower
        axis=v/norm
        lower=max(lower,float(np.min(a@axis)-np.max(b@axis))-1e-9,0.)
        if lower>required:return lower
        w=support(-v)
        if np.min(np.linalg.norm(simplex-w,axis=1))<1e-10:return lower
        v,simplex=closest_simplex(np.vstack((simplex,w)))
    return lower


class _PointShapeBounds:
    """One synchronous point query only; never memoize a collision verdict.

    Hold converted ndarray references and exact C-order byte snapshots. A
    resumed pair generator may mutate/alias/replace arrays: every finite()
    lookup compares shape and bytes, invalidating all reductions on change.
    Thus identity is only an index, never evidence of immutable contents.
    NumPy conversion strips subclass callbacks before snapshots/reductions.
    There are no yields/callbacks between validation and bound() use. As in
    the reference predicate, concurrent writes during a NumPy operation are
    unsupported; actual runtime worlds are synchronous owned query geometry.
    """
    def __init__(self):
        self.rows={}

    def finite(self,a):
        key=id(a);raw=a.tobytes(order='C');row=self.rows.get(key)
        if row is None or row[0] is not a or row[1]!=a.shape or row[2]!=raw:
            row=(a,a.shape,raw,bool(np.isfinite(a).all()),{})
            self.rows[key]=row
        return row[3]

    def bound(self,a,kind):
        bounds=self.rows[id(a)][4]
        if kind not in bounds:
            bounds[kind]=a.min(0) if kind=='min' else a.max(0)
        return bounds[kind]


def _separation_lower_cached(a,b,required,cache):
    """Any returned positive value is a separating-projection lower bound."""
    a=np.asarray(a,float);b=np.asarray(b,float)
    if a.ndim!=2 or b.ndim!=2 or a.shape[1]!=3 or b.shape[1]!=3 or not cache.finite(a) or not cache.finite(b):
        raise ValueError('invalid convex vertices')
    gap=np.maximum(np.maximum(cache.bound(a,'min')-cache.bound(b,'max'),cache.bound(b,'min')-cache.bound(a,'max')),0)
    lower=max(0.,float(np.linalg.norm(gap))-1e-9)
    if lower>required:return lower
    def support(d):return a[np.argmax(a@d)]-b[np.argmin(b@d)]
    v=a.mean(0)-b.mean(0)
    if np.linalg.norm(v)<1e-12:v=np.array([1.,0,0])
    simplex=np.array([support(v)]);v=simplex[0]
    for _ in range(48):
        norm=float(np.linalg.norm(v))
        if norm<1e-12:return lower
        axis=v/norm
        lower=max(lower,float(np.min(a@axis)-np.max(b@axis))-1e-9,0.)
        if lower>required:return lower
        w=support(-v)
        if np.min(np.linalg.norm(simplex-w,axis=1))<1e-10:return lower
        v,simplex=closest_simplex(np.vstack((simplex,w)))
    return lower


class LinkMotionBounds:
    def __init__(self,model,geometry,vertices,dt=DT):
        parents={j.find('child').get('link'):j for j in model.root.findall('joint')}
        joints=[f'panda_joint{i}' for i in range(1,8)]+['panda_finger_joint1','panda_finger_joint2']
        self.coefficients={}
        for shape in geometry['robot_boxes']:
            link=shape['link'];t=np.asarray(origin(ET.Element('origin',shape['origin'])))
            v=np.asarray(vertices[link])@t[:3,:3].T+t[:3,3]
            reach=float(np.max(np.linalg.norm(v,axis=1)));coeff=np.zeros(9);cursor=link
            while cursor in parents:
                j=parents[cursor];kind=j.get('type');name=j.get('name')
                if kind=='prismatic':
                    travel=max(abs(float(j.find('limit').get(k))) for k in ('lower','upper'))
                    axis=np.array(list(map(float,j.find('axis').get('xyz').split())))
                    if name not in joints:raise ValueError('unknown prismatic joint')
                    coeff[joints.index(name)]=np.linalg.norm(axis)
                    # Include permitted raw finger tracking excursion and its
                    # one-step speed reserve, not just clipped URDF positions.
                    reach+=(travel+TRACK_FINGER+FINGER_SPEED*dt)*np.linalg.norm(axis)
                elif kind=='revolute':
                    if name not in joints:raise ValueError('unknown revolute joint')
                    coeff[joints.index(name)]=reach
                elif kind!='fixed':raise ValueError('unsupported movement bound joint')
                reach+=np.linalg.norm(np.asarray(origin(j.find('origin')))[:3,3])
                cursor=j.find('parent').get('link')
            self.coefficients[link]=coeff

    def pair_coefficients(self,a,b=None):
        ca=self.coefficients[a].copy()
        if b is None:return ca
        cb=self.coefficients[b].copy();common=(ca>0)&(cb>0)
        ca[common]=0;cb[common]=0
        return ca+cb


class ReturnClearance:
    def __init__(self,world,arm,target=None,allow_target_fingers=False):
        if world.held is not None:raise ValueError('return clearance is unloaded only')
        self.world=world;self.arm=arm;self.target=str(target);self.allow=allow_target_fingers
        self.contact_envelope=getattr(world,'home_contact_envelope',None)
        if self.contact_envelope is not None and allow_target_fingers:raise ValueError('home_contact_unloaded_home_only')
        self.dt=DT if self.contact_envelope is None else self.contact_envelope.dt
        self.bounds=LinkMotionBounds(world.model,world.geometry,world.robot_vertices,self.dt)
        self.fingers=np.clip(world.state['robots'][arm]['q'][-2:],0,.04)
        self.other='left' if arm=='right' else 'right'
        o=world.state['robots'][self.other]['q']
        self.other_shapes=world.robot_shapes(self.other,o[:7],np.clip(o[-2:],0,.04).tolist())
        self.radii={oid:max(float(np.max(np.linalg.norm(p['vertices'],axis=1))) for p in parts)
                    for oid,parts in world.meshes.items()}
        self.last=None

    def tracking_displacement(self,arm,reference,actual):
        """A correspondence of all vertices bounds convex-hull Hausdorff motion.

        Use the cheap joint-sum bound first; refine only when it exceeds the
        tube. Refinement captures cancellation of measured joint errors without
        assuming that cancellation between future interval configurations.
        """
        reference=np.asarray(reference,float);actual=np.asarray(actual,float)
        delta=np.abs(actual-reference)
        bounds={n:float(c@delta) for n,c in self.bounds.coefficients.items()}
        if max(bounds.values())<=LINK_TUBE_M:return bounds
        a=dict(self.world.robot_shapes(arm,reference[:7].tolist(),np.clip(reference[-2:],0,.04).tolist()))
        b=dict(self.world.robot_shapes(arm,actual[:7].tolist(),np.clip(actual[-2:],0,.04).tolist()))
        clipped=np.abs(reference[-2:]-np.clip(reference[-2:],0,.04))+np.abs(actual[-2:]-np.clip(actual[-2:],0,.04))
        return {n:min(bounds[n],float(np.max(np.linalg.norm(a[n][1]-b[n][1],axis=1)))+
                      float(self.bounds.coefficients[n][-2:]@clipped)+1e-9) for n in bounds}

    def step_bounds(self,arm,q,fingers,robot):
        """Tighten ONLY the geometric radius, without reducing any rate limit.

        Distance of a downstream vertex to joint j's axis is invariant under
        upstream and j rotations. Its change from downstream joints is bounded
        by their global motion coefficients. Pad for the entire tracking box
        AND the next speed-limited step, including raw finger excursions.
        Interval-center variation is padded separately in point().
        """
        transforms=self.world.model.link_transforms(arm,list(q),list(fingers))
        axes=[]
        for joint in self.world.model.chain[:7]:
            frame=np.asarray(transforms[joint.find('child').get('link')])
            axis=frame[:3,:3]@np.array(list(map(float,joint.find('axis').get('xyz').split())))
            axes.append((frame[:3,3],axis/np.linalg.norm(axis)))
        result={}
        excursion=TRACK_BOX+SPEED_BOX*self.dt
        for name,shape in robot:
            c=self.bounds.coefficients[name];local=c.copy()
            for j,(center,axis) in enumerate(axes):
                if c[j]==0:continue
                delta=shape[1]-center
                radius=float(np.max(np.linalg.norm(delta-np.outer(delta@axis,axis),axis=1)))
                local[j]=min(c[j],radius+float(c[j+1:]@excursion[j+1:])+1e-9)
            result[name]=float(local@SPEED_BOX)*self.dt
        return result

    def _pairs(self,q,*,include_reserves=True,include_coefficients=True):
        robot=self.world.robot_shapes(self.arm,list(q),self.fingers.tolist())
        if include_reserves:
            steps=self.step_bounds(self.arm,q,self.fingers,robot)
        else:
            # Keep input validation even on a shape-cache hit; FK is pure and
            # its transforms are used only by the discarded step reserve.
            self.world.model.check_q(list(q))
            fingers=list(map(float,self.fingers))
            if len(fingers)!=2 or any(not math.isfinite(f) or not 0<=f<=.04 for f in fingers):
                raise ValueError('invalid arm or finger position')
        other_q=self.world.state['robots'][self.other]['q']
        if include_reserves:
            other_steps=self.step_bounds(self.other,other_q[:7],np.clip(other_q[-2:],0,.04),self.other_shapes)
        else:
            # The world state is mutable; preserve validation after construction.
            self.world.model.check_q(list(other_q[:7]))
            fingers=list(map(float,np.clip(other_q[-2:],0,.04)))
            if len(fingers)!=2 or any(not math.isfinite(f) or not 0<=f<=.04 for f in fingers):
                raise ValueError('invalid arm or finger position')
        for i,(name,a) in enumerate(robot):
            c=self.bounds.pair_coefficients(name) if include_coefficients else None
            if include_reserves:tracking=min(float(c@TRACK_BOX),LINK_TUBE_M);step=steps[name]
            for n,b in robot[i+1:]:
                if frozenset((name,n)) in self.world.disabled:continue
                pair=self.bounds.pair_coefficients(name,n) if include_coefficients else None
                reserve=min(float(pair@TRACK_BOX),2*LINK_TUBE_M)+float(pair@SPEED_BOX)*self.dt if include_reserves else 0.
                # MeshWorld.hit uses a 1e-7 m floor even when margin=0.
                yield [name,n],a[1],b[1],pair,reserve,1e-7
            for n,b in self.world.static_shapes:
                if name=='panda_link0' and n=='table':continue
                yield [name,n],a[1],b[1],c,tracking+step if include_reserves else 0.,.001
            for n,b in self.other_shapes:
                reserve=0.
                if include_reserves:
                    other=self.bounds.pair_coefficients(n)
                    reserve=tracking+step+min(float(other@TRACK_BOX),LINK_TUBE_M)+other_steps[n]
                yield [name,self.other+':'+n],a[1],b[1],c,reserve,.001
            for oid,parts in self.world.object_shapes.items():
                if self.allow and oid==self.target and 'finger' in name:continue
                reserve=tracking+step+WORLD_DRIFT_M+(OBS_SPEED+self.radii[oid]*OBS_SPIN)*self.dt if include_reserves else 0.
                for k,b in enumerate(parts):
                    margin=.001 if self.contact_envelope is None else self.contact_envelope.margin(self.arm,name,oid,k,.001)
                    yield [name,'object_'+oid+':'+str(k)],a[1],b[1],c,reserve,margin
        if self.contact_envelope is not None:
            # Parked-arm motion is covered by its unchanged tracking/step
            # envelope, independent of the active arm's path interval.
            for name,a in self.other_shapes:
                c=self.bounds.pair_coefficients(name)
                for oid,parts in self.world.object_shapes.items():
                    reserve=(min(float(c@TRACK_BOX),PARKED_TUBE_M)+float(c@PARKED_SPEED_BOX)*self.dt+WORLD_DRIFT_M+
                             (OBS_SPEED+self.radii[oid]*OBS_SPIN)*self.dt) if include_reserves else 0.
                    for k,b in enumerate(parts):
                        yield [self.other+':'+name,'object_'+oid+':'+str(k)],a[1],b[1],np.zeros(9),reserve,self.contact_envelope.margin(self.other,name,oid,k,.001)

    def _zero_coefficients_ok(self):
        # Check the current owned arrays every call, with no memoized eligibility.
        # Finite nonnegative coefficients make pair cancellation/addition finite;
        # all zero products and the unchanged finite speed factor yield +0.
        c=self.bounds.coefficients
        if type(c) is not dict or type(self.bounds) is not LinkMotionBounds:return False
        if set(c)!={r['link'] for r in self.world.geometry['robot_boxes']}:return False
        if any(n not in c for n,_ in self.other_shapes):return False
        return all(type(a) is np.ndarray and a.dtype==np.dtype(float) and a.shape==(9,)
                   and np.isfinite(a).all() and (a>=0).all() for a in c.values())

    @timing.measured('geometry_point')
    def point(self,q,half_span=None,envelopes=True):
        half=np.zeros(9) if half_span is None else np.r_[np.asarray(half_span,float),0.,0.]
        zero=(not envelopes and half.shape==(9,) and not np.any(half) and self._zero_coefficients_ok())
        worst=None
        # Coefficients still bound half_span when envelopes are disabled.
        pairs=self._pairs(q) if envelopes else self._pairs(q,include_reserves=False,include_coefficients=not zero)
        cache=_PointShapeBounds()
        for pair,a,b,c,reserve,margin in pairs:
            if not envelopes:reserve=0.
            # Local step radii vary over an interval by at most c@half for
            # each ancestor. Retain the old whole-link interval bound too.
            movement=0. if zero else float(c@half)*(1+float(SPEED_BOX[:7].sum())*self.dt)
            required=reserve+movement+margin
            lower=_separation_lower_cached(a,b,required,cache)
            row=dict(pair=pair,clearance_lower_m=lower,required_m=required,
                     intersample_bound_m=movement,tracking_and_step_reserve_m=reserve,margin_m=margin,
                     slack_m=lower-required)
            if self.contact_envelope is not None:row['home_contact_model_sha256']=self.contact_envelope.sha256
            if worst is None or row['slack_m']<worst['slack_m']:worst=row
            if lower<=required:self.last=row;return dict(safe=False,**row)
        self.last=worst;return dict(safe=True,**worst)

    @timing.measured('geometry_certificate')
    def certify(self,path,max_nodes=4096):
        """Full edge certificate. Subdivision does not change commanded path."""
        q=np.asarray(path,float);nodes=0;leaves=0;worst=None
        binding={} if self.contact_envelope is None else dict(home_contact_model_sha256=self.contact_envelope.sha256)
        if q.ndim!=2 or q.shape[1]!=7 or not len(q) or not np.isfinite(q).all():raise ValueError('invalid return path')
        for v in q:
            row=self.point(v.tolist());nodes+=1
            if not row['safe']:return dict(**binding,certified=False,reason='endpoint_clearance',nodes=nodes,witness=row)
        stack=[(a,b,0) for a,b in zip(q[:-1],q[1:])]
        while stack:
            a,b,depth=stack.pop();center=(a+b)/2;half=np.abs(b-a)/2
            nodes+=1
            if nodes>max_nodes:return dict(**binding,certified=False,reason='certificate_budget',nodes=nodes)
            row=self.point(center.tolist(),half)
            if row['safe']:
                leaves+=1
                if worst is None or row['slack_m']<worst['slack_m']:worst=row
            elif depth>=18 or np.max(half)<1e-6:
                return dict(**binding,certified=False,reason='interval_clearance',nodes=nodes,witness=row)
            else:stack.extend([(a,center,depth+1),(center,b,depth+1)])
        return dict(**binding,certified=True,nodes=nodes,leaves=leaves,witness=worst,
                    scope='whole joint-linear command curve plus conditional tracking/speed/world envelopes')

    def monitor(self,state,command,*,monitor_other=True,moving_object=None):
        """Before each new command: measured deviation and one-step envelopes."""
        robot=state['robots'][self.arm];actual=np.asarray(robot['q'],float)
        if not np.isfinite(actual).all() or not np.isfinite(robot['qdot']).all() or not np.isfinite(command).all():
            return dict(safe=False,reasons=['nonfinite_robot_state'])
        limits=np.asarray(self.world.model.bounds)+[.03,-.03]
        if np.any(actual[:7]<limits[:,0]) or np.any(actual[:7]>limits[:,1]):
            return dict(safe=False,reasons=['measured_joint_margin'])
        # Physical finger limits may have micrometre solver excursions. Do not
        # hide them: displacement uses RAW fingers, geometry is certified at
        # the clipped starting fingers and accounts for the raw deviation.
        dq=np.abs(actual-np.r_[command,self.fingers]);speeds=np.abs(robot['qdot'])
        displacements=self.tracking_displacement(self.arm,np.r_[command,self.fingers],actual)
        reasons=[]
        if np.any(dq>TRACK_BOX):reasons.append('joint_tracking_envelope')
        if max(displacements.values())>LINK_TUBE_M:reasons.append('link_tracking_envelope')
        if np.any(speeds>SPEED_BOX):reasons.append('measured_speed_envelope')
        other=np.asarray(state['robots'][self.other]['q']);other_ref=np.asarray(self.world.state['robots'][self.other]['q'])
        if not np.isfinite(other).all() or not np.isfinite(state['robots'][self.other]['qdot']).all():
            return dict(safe=False,reasons=['nonfinite_other_arm'])
        if np.any(other[:7]<limits[:,0]) or np.any(other[:7]>limits[:,1]):
            return dict(safe=False,reasons=['other_arm_joint_margin'])
        od=np.abs(other-other_ref)
        if monitor_other and (np.any(od>TRACK_BOX) or max(self.tracking_displacement(self.other,other_ref,other).values())>LINK_TUBE_M
                or np.any(np.abs(state['robots'][self.other]['qdot'])>SPEED_BOX)):
            reasons.append('other_arm_envelope')
        if self.contact_envelope is not None:
            parked_ref=np.r_[other_ref[:7],np.clip(other_ref[-2:],0,.04)]
            parked=self.tracking_displacement(self.other,parked_ref,other)
            if max(parked.values())>PARKED_TUBE_M or np.any(np.abs(state['robots'][self.other]['qdot'])>PARKED_SPEED_BOX):
                reasons.append('home_parked_arm_envelope')
        drift={}
        for oid,o in state['objects'].items():
            # Only the intentionally carried payload is nonstationary. It is
            # still present in live collision geometry and contact guards.
            if moving_object is not None and oid==str(moving_object):continue
            original=self.world.state['objects'][oid]['pose'];p=o['pose']
            if not all(np.isfinite(o[k]).all() for k in ('pose','linear_velocity','angular_velocity')) or np.linalg.norm(p[3:])<1e-12:
                return dict(safe=False,reasons=['nonfinite_object:'+oid])
            dot=abs(float(np.asarray(p[3:])@original[3:]))/(np.linalg.norm(p[3:])*np.linalg.norm(original[3:]))
            drift[oid]=float(np.linalg.norm(np.asarray(p[:3])-original[:3])+2*self.radii[oid]*math.sqrt(max(0.,1-min(1.,dot)**2)))
            if drift[oid]>WORLD_DRIFT_M or np.linalg.norm(o['linear_velocity'])>OBS_SPEED or np.linalg.norm(o['angular_velocity'])>OBS_SPIN:
                reasons.append('object_envelope:'+oid)
        return dict(safe=not reasons,reasons=reasons,link_tracking_bound_m=displacements,
                    obstacle_drift_bound_m=drift,joint_tracking_abs=dq.tolist())
