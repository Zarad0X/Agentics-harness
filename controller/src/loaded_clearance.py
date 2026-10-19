"""Isolated empirical loaded-motion admission; no physics setters or legacy APIs.

Bounds are explicit experimental assumptions, NOT proven in-flight limits.
Certificates cover joint-linear nominal paths plus these declared reserves.
Native monitoring rejects violations; it cannot prove hidden between-tick motion.
"""
import runtime_profile as timing
from dataclasses import dataclass,asdict
import math
import numpy as np
from loaded_common import digest
from collision_world import pose_matrix
from astra_planner import resample_path,so3_log,solve_ik,plan_joint_path
from return_clearance import separation_lower,LinkMotionBounds
from direct_core import origin

@dataclass(frozen=True)
class Envelope:
    position_m:float=.006
    rotation_rad:float=.020
    slip_position_m:float=.0015
    slip_rotation_rad:float=.006
    obstacle_vertex_m:float=.003
    between_tick_m:float=.002
    margin_m:float=.001
    joint_rad:float=.015
    def __post_init__(self):
        if any(not math.isfinite(v) or v<0 for v in asdict(self).values()) or self.margin_m<.001:
            raise ValueError('invalid_empirical_envelope')
        if self.rotation_rad>math.pi or self.slip_rotation_rad>math.pi:raise ValueError('rotation_envelope')

PROFILE=dict(dt=.004,max_velocity=.25,max_acceleration=None,clock='joint_linear',handoff='continue_previous_drive_target',
             hold_steps=100,extra_hold_max=1000,extra_hold_stride=50,
             endpoint_position_m=.005,endpoint_rotation_rad=.04,endpoint_joint_rad=.025)

def ee(model,arm,q):
    f=model.evaluate(arm,[float(x) for x in q]); t=np.eye(4)
    t[:3,:3]=f['ee_rotation_world'];t[:3,3]=f['ee_position_m'];return t

def motion_coefficients(model,radius):
    # Reverse actual serial chain. EE transform is a fixed rotation plus .04m.
    reach=float(radius)+.04; coeff=np.zeros(7)
    for j in reversed(model.chain):
        if j.get('type')=='revolute':coeff[int(j.get('name').removeprefix('panda_joint'))-1]=reach
        elif j.get('type')!='fixed':raise ValueError('unsupported_chain')
        reach+=float(np.linalg.norm(np.array(origin(j.find('origin')))[:3,3]))
    return coeff

class PayloadAdmission:
    def __init__(self,world,arm,target,relative,envelope=Envelope()):
        self.world=world;self.arm=arm;self.target=str(target);self.relative=np.array(relative,float)
        self.env=envelope;self.world.held=(arm,self.target,self.relative)
        verts=[np.array(p['vertices']) for p in world.meshes[self.target]]
        self.local=verts
        self.radius=max(float(np.linalg.norm(v@self.relative[:3,:3].T+self.relative[:3,3],axis=1).max()) for v in verts)
        self.object_radius=max(float(np.linalg.norm(v,axis=1).max()) for v in verts)
        self.coeff=motion_coefficients(world.model,self.radius)
        link_bounds=LinkMotionBounds(world.model,world.geometry,world.robot_vertices)
        self.self_coeff={}
        for name,c in link_bounds.coefficients.items():
            if 'finger' in name:continue
            p=self.coeff.copy();r=c[:7].copy();common=(p>0)&(r>0);p[common]=0;r[common]=0
            self.self_coeff[name]=p+r
        self.self_max=np.max(list(self.self_coeff.values()),axis=0)
        self.slip=envelope.slip_position_m+2*self.object_radius*math.sin(envelope.slip_rotation_rad/2)
        self.reserve=(envelope.margin_m+envelope.position_m+2*self.radius*math.sin(envelope.rotation_rad/2)+
                      self.slip+envelope.obstacle_vertex_m+envelope.between_tick_m)
        other='right' if arm=='left' else 'left';oq=world.state['robots'][other]['q']
        self.obstacles=[(n,s[1]) for n,s in world.static_shapes]
        self.obstacles += [('idle:'+n,s[1]) for n,s in world.robot_shapes(other,oq[:7],np.clip(oq[-2:],0,.04).tolist())]
        self.obstacles += [('object:'+oid+':'+str(i),s[1]) for oid,parts in world.object_shapes.items() if oid!=self.target for i,s in enumerate(parts)]
        self.obstacle_bounds=[(n,v,v.min(0),v.max(0)) for n,v in self.obstacles]
        self.cache={};self.checks=0

    def at(self,q,extra=0.):
        q=[float(x) for x in q]; key=tuple(q)
        if key not in self.cache:
            self.checks+=1
            free=self.world.free(self.arm,q,int(self.target),True)
            if not free:self.cache[key]=(False,str(self.world.last_collision),0.,0.);return self.cache[key]
            t=ee(self.world.model,self.arm,q)@self.relative
            payload=[v@t[:3,:3].T+t[:3,3] for v in self.local]
            minimum=math.inf; pair=None
            for i,a in enumerate(payload):
                lo,hi=a.min(0),a.max(0)
                for name,b,blo,bhi in self.obstacle_bounds:
                    # Positive enclosing-AABB distance is already a lower bound.
                    gap=float(np.linalg.norm(np.maximum(np.maximum(lo-bhi,blo-hi),0)))
                    lower=gap-1e-9 if gap>self.reserve+extra else separation_lower(a,b,self.reserve+extra)
                    if lower<minimum:minimum=lower;pair=(i,name)
            # Payload versus own non-finger links needs relative-slip reserve;
            # shared rigid EE tracking does not inflate that internal distance.
            internal=math.inf
            for name,b in self.world.robot_shapes(self.arm,q,np.clip(self.world.state['robots'][self.arm]['q'][-2:],0,.04).tolist()):
                if 'finger' in name:continue
                required=self.slip+float(self.self_coeff[name].sum())*self.env.joint_rad
                for a in payload:internal=min(internal,separation_lower(a,b[1],required+extra)-required)
            self.cache[key]=(True,pair,float(minimum),float(internal))
        return self.cache[key]

    def certify(self,path,max_nodes=4096):
        path=np.asarray(path,float)
        if path.ndim!=2 or path.shape[1]!=7 or not np.isfinite(path).all():raise ValueError('path')
        stack=[(a,b,0) for a,b in zip(path[:-1],path[1:])]; visited=0; worst=math.inf
        if not stack:stack=[(path[0],path[0],0)]
        while stack:
            a,b,depth=stack.pop();visited+=1
            if visited>max_nodes:return dict(safe=False,reason='certificate_budget',nodes=visited)
            half=float(self.coeff@np.abs(b-a)/2)
            self_half=float(self.self_max@np.abs(b-a)/2)
            # Relative self-pairs use conservative sum of two moving shapes.
            ok,pair,lower,internal=self.at((a+b)/2,max(half,self_half))
            if not ok:return dict(safe=False,reason='nominal_collision',pair=pair,nodes=visited)
            if lower>self.reserve+half and internal>self_half:
                worst=min(worst,lower-self.reserve-half);continue
            if lower<=self.reserve or internal<=0:
                return dict(safe=False,reason='payload_empirical_reserve',pair=pair,lower_m=lower,required_m=self.reserve,nodes=visited)
            if depth>=18:return dict(safe=False,reason='interval_unresolved',nodes=visited)
            mid=(a+b)/2;stack.extend([(a,mid,depth+1),(mid,b,depth+1)])
        return dict(safe=True,reason='conditional_empirical_certificate',nodes=visited,minimum_slack_m=worst,reserve_m=self.reserve)

    def admit(self,path,state,controls,profile=PROFILE):
        if profile!=PROFILE:raise ValueError('unqualified_profile')
        drive=[j['target'][0] for j in controls[self.arm][:7]]
        if not np.allclose(path[0],drive,atol=2e-7,rtol=0):raise ValueError('command_handoff_changed')
        observed=ee(self.world.model,self.arm,state['robots'][self.arm]['q'][:7])
        desired=ee(self.world.model,self.arm,path[0])
        if (np.linalg.norm(observed[:3,3]-desired[:3,3])>self.env.position_m or
            np.linalg.norm(so3_log(observed[:3,:3]@desired[:3,:3].T))>self.env.rotation_rad or
            np.max(np.abs(np.array(state['robots'][self.arm]['q'][:7])-path[0]))>self.env.joint_rad):
            return dict(safe=False,reason='initial_tracking_outside_empirical_envelope')
        cert=self.certify(path)
        if not cert['safe']:return cert
        dense=np.array(resample_path(path,max_velocity=profile['max_velocity'],dt=profile['dt']))
        # First point is already commanded. No reset/bridge or new first target.
        velocities=np.vstack(((dense[1:]-dense[:-1])/profile['dt'],np.zeros(7)))
        commands=[dict(q=q.tolist(),v=v.tolist()) for q,v in zip(dense[1:],velocities[1:])]
        motion_commands=len(commands)
        commands += [dict(q=dense[-1].tolist(),v=[0.]*7) for _ in range(profile['hold_steps']+profile['extra_hold_max'])]
        binding=dict(state=digest(state),controls=digest(controls),relative=self.relative.tolist(),
                     envelope=asdict(self.env),profile=profile,commands_sha256=digest(commands),path=np.asarray(path).tolist(),
                     geometry=digest(self.world.meshes),static=digest([(n,v.tolist()) for n,v in self.obstacles]))
        return dict(safe=True,certificate=cert,binding=binding,binding_sha256=digest(binding),commands=commands,motion_commands=motion_commands)

    def validate(self,receipt,state,controls):
        b=receipt['binding']
        expected_motion=len(receipt['commands'])-PROFILE['hold_steps']-PROFILE['extra_hold_max']
        if (receipt.get('motion_commands')!=expected_motion or not receipt.get('safe') or digest(b)!=receipt['binding_sha256'] or b['state']!=digest(state) or b['controls']!=digest(controls)
            or b['relative']!=self.relative.tolist() or b['envelope']!=asdict(self.env) or b['profile']!=PROFILE
            or b['commands_sha256']!=digest(receipt['commands']) or b['geometry']!=digest(self.world.meshes)
            or b['static']!=digest([(n,v.tolist()) for n,v in self.obstacles])):raise ValueError('stale_loaded_admission')

def search(admission,q0,slots,emit=lambda x:None,max_options=48):
    """Bounded slot/yaw/high-outward route search; no physics mutation/retry."""
    w=admission.world;arm=admission.arm;rel=admission.relative;model=w.model
    import astra_planner
    astra_planner.MAX_COLLISION_CHECKS=1200
    bounds=np.array(model.bounds)+[.03,-.03];rng=np.random.default_rng(20261007)
    current=ee(model,arm,q0);base_r=current[:3,:3];tried=0
    for slot in slots:
        for yaw in (0.,-np.pi/2,np.pi/2,np.pi/4,-np.pi/4,np.pi):
            tried+=1
            if tried>max_options:raise RuntimeError('loaded_alternative_budget')
            c,s=np.cos(yaw),np.sin(yaw);r=np.array([[c,-s,0],[s,c,0],[0,0,1]])@base_r
            offset=r@rel[:3,3];p=np.r_[np.array(slot[:2])-offset[:2],max(current[2,3],1.21)]
            obj=np.eye(4);obj[:3,:3]=r@rel[:3,:3];obj[:3,3]=p+r@rel[:3,3]
            bottom=min((v@obj[:3,:3].T+obj[:3,3])[:,2].min() for v in admission.local)
            low=p-np.array([0,0,bottom-.77])
            # Cheap reachability screening before spending on a high route.
            endpoint=solve_ik(model,arm,low,r,q0,rng=rng,max_iters=150,restarts=2)
            if not endpoint['success']:
                emit(dict(kind='alternative',slot=slot,yaw=yaw,accepted=False,reason='low_ik'));continue
            ok,pair,lower,internal=admission.at(endpoint['q'])
            if not ok or lower<=admission.reserve or internal<=0:
                emit(dict(kind='alternative',slot=slot,yaw=yaw,accepted=False,reason='low_clearance',pair=pair));continue
            # Lift first, move outward while high, then translate/yaw, then lower.
            outward=current[:3,3].copy();outward[2]=p[2]
            exit_p=outward.copy();exit_p[0]=p[0]
            targets=[(outward,base_r),(exit_p,base_r),(p,r),(low,r)]
            path=[list(map(float,q0))];failure=None
            for stage,(xyz,rotation) in enumerate(targets):
                emit(dict(kind='search_stage',attempt=tried,slot=slot,yaw=yaw,stage=stage))
                ik=solve_ik(model,arm,xyz,rotation,path[-1],rng=rng,max_iters=150,restarts=2)
                if not ik['success']:failure='ik';break
                # Planner callback includes empirical payload reserve. Exact
                # continuous-edge certificate still runs on the final path.
                def free(q):
                    ok,_,lower,internal=admission.at(q)
                    return bool(ok and lower>admission.reserve and internal>0)
                plan=plan_joint_path(path[-1],ik['q'],bounds,free,rng=rng,max_nodes=32,resolution=.025)
                if not plan['success']:failure=plan['reason'];break
                path.extend(plan['path'][1:])
            if failure is None:
                cert=admission.certify(path)
                if cert['safe']:
                    emit(dict(kind='alternative',slot=slot,yaw=yaw,accepted=True,certificate=cert))
                    return dict(path=path,slot=slot,yaw=yaw,low=low.tolist(),rotation=r.tolist(),attempts=tried)
                failure=cert['reason']
            emit(dict(kind='alternative',slot=slot,yaw=yaw,accepted=False,reason=failure))
    raise RuntimeError('no_loaded_alternative')

def alternative_slots(state,arm,existing,target):
    """Additional physical tray centres, never changes tray/object geometry."""
    p=state['objects'][str(target)]['pose']
    sign=-1 if arm=='left' else 1
    extra=[[sign*x,float(y),.753,1,0,0,0] for x in (.40,.42,.44,.48,.54,.60,.66,.72)
           for y in (-.2,-.1,0.,.1,.2,-.3,-.4,-.5,-.6)]
    values={tuple(s):list(s) for s in extra+list(existing)}
    return sorted(values.values(),key=lambda s:(abs(s[0]-p[0])+abs(s[1]-p[1]),s[0],s[1]))

class Monitor:
    def __init__(self,admission):
        self.a=admission;self.previous=None
        self.reference_objects={oid:[p[1] for p in parts] for oid,parts in admission.world.object_shapes.items() if oid!=admission.target}
    @timing.measured('monitor')
    def measure(self,state,native_ee,command):
        a=self.a;observed=np.asarray(native_ee);planned=ee(a.world.model,a.arm,command)
        obj=pose_matrix(state['objects'][a.target]['pose']);rel=np.linalg.inv(observed)@obj
        pe=float(np.linalg.norm(observed[:3,3]-planned[:3,3]));re=float(np.linalg.norm(so3_log(observed[:3,:3]@planned[:3,:3].T)))
        sp=float(np.linalg.norm(rel[:3,3]-a.relative[:3,3]));sr=float(np.linalg.norm(so3_log(rel[:3,:3]@a.relative[:3,:3].T)))
        vertices=np.concatenate([v@obj[:3,:3].T+obj[:3,3] for v in a.local])
        motion=0. if self.previous is None else float(np.linalg.norm(vertices-self.previous,axis=1).max());self.previous=vertices
        drift=0.
        for oid,reference in self.reference_objects.items():
            t=pose_matrix(state['objects'][oid]['pose'])
            for mesh,initial in zip(a.world.meshes[oid],reference):
                actual=np.array(mesh['vertices'])@t[:3,:3].T+t[:3,3]
                drift=max(drift,float(np.linalg.norm(actual-initial,axis=1).max()))
        other='right' if a.arm=='left' else 'left';q=state['robots'][other]['q']
        actual=a.world.robot_shapes(other,q[:7],np.clip(q[-2:],0,.04).tolist())
        initial={n[5:]:v for n,v in a.obstacles if n.startswith('idle:')}
        for n,shape in actual:drift=max(drift,float(np.linalg.norm(shape[1]-initial[n],axis=1).max()))
        joint=float(np.max(np.abs(np.asarray(state['robots'][a.arm]['q'][:7])-command)))
        values=dict(position_m=pe,rotation_rad=re,slip_position_m=sp,slip_rotation_rad=sr,obstacle_vertex_m=drift,between_tick_m=motion,joint_rad=joint)
        violations=[n for n,v in values.items() if v>getattr(a.env,n)]
        return dict(values=values,violations=violations,relative=rel.tolist())
