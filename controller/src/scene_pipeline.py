"""Initial scene generation only. All placement setters run before formal start."""
import json
import math
from pathlib import Path
import numpy as np
from pose_goals import mesh_bounds,pose_errors
from provenance import file_hash,digest

BASE_IDS=(3,4,8,9,10,11,12,17,18,19)
CONTAINER=dict(center=[-.04,-.10],inner=[.532,.427],floor_z=.752,wall_height=.15,
               front_lip_height=.06,wall_thickness=.012,scene_scale=.7,collision_scale=.95)


def inventory(layout,count):
    if type(count) is not int or count not in (10,15,20):raise ValueError('inventory_count')
    ids=[o['n'] for o in layout['objects']]
    if any(type(i) is not int for i in ids) or len(set(ids))!=len(ids):
        raise ValueError('inventory_asset_ids_not_unique_integers')
    by_id={o['n']:o for o in layout['objects']}
    if not set(BASE_IDS)<=set(by_id): raise ValueError('missing frozen assets')
    chosen=[by_id[i] for i in BASE_IDS]
    # The original 10 include N12 (a small dish). Do not add the large trays.
    chosen += [o for o in layout['objects'] if o['n'] not in BASE_IDS and not o.get('is_tray',False)][:count-10]
    while len(chosen)<count: chosen.append(by_id[BASE_IDS[(len(chosen)-10)%len(BASE_IDS)]])
    used=set();records=[]
    for o in chosen:
        oid=o['n'] if o['n'] not in used else 1000+len(records)
        if oid in used:raise ValueError('inventory_instance_id_collision')
        used.add(oid);records.append(dict(o,instance_id=oid,asset_id=o['n']))
    if len(records)!=count:raise ValueError('inventory_instance_count')
    return records


def pack_layout(local_bounds,seed,max_trials=1500):
    """Randomized bottom-left height packing with 3mm separating AABB gaps.

    A conservative spawn certificate, followed by mandatory physical settling
    validation. No mesh or scale changes; unsuccessful density is a result.
    """
    rng=np.random.default_rng(seed);placed=[];result=[]
    center=np.array(CONTAINER['center']);half=np.array(CONTAINER['inner'])/2
    order=sorted(range(len(local_bounds)),key=lambda i:-np.prod(local_bounds[i][1]-local_bounds[i][0]))
    for i in order:
        lo,hi=map(np.asarray,local_bounds[i]);size=hi-lo
        low=center-half+.003-lo[:2];high=center+half-.003-hi[:2]
        if np.any(low>high): raise RuntimeError('asset_does_not_fit_container:'+str(i))
        best=None
        for _ in range(max_trials):
            xy=rng.uniform(low,high);bottom=CONTAINER['floor_z']+.002
            al=xy+lo[:2];ah=xy+hi[:2]
            for pl,ph in placed:
                if np.all(np.minimum(ah,ph[:2])-np.maximum(al,pl[:2])>-.003):bottom=max(bottom,ph[2]+.003)
            if best is None or bottom<best[0]:best=(bottom,xy)
        bottom,xy=best
        if bottom+size[2]>1.27:raise RuntimeError('packing_height_budget_exhausted')
        p=np.r_[xy,bottom-lo[2]];placed.append((p+lo,p+hi));result.append((i,p.tolist()))
    return dict(result)


def load_generated(environment,task,sapien,config):
    from physics_geometry import convex_parts
    layout=json.loads(Path(task.LAYOUT_PATH).read_text());records=inventory(layout,config.count)
    environment.objects={};environment._modelnames={};bounds=[];rng=np.random.default_rng(config.seed)
    for r in records:
        oid=r['instance_id'];actor=environment._create_inner_actor(r['modelname'],[0,0,2.],r['model_id'])
        actor.actor.set_name('roundtrip_'+str(oid))
        vol=float(np.prod(r['aabb_size']))*task.SCENE_SCALE**3
        actor.set_mass(float(np.clip(vol*400,.03,.4)))
        body=actor.actor.find_component_by_type(sapien.pysapien.physx.PhysxRigidDynamicComponent)
        body.set_linear_damping(3.);body.set_angular_damping(8.)
        # World yaw multiplied by the original scan orientation; scale unchanged.
        angle=float(rng.uniform(-np.pi,np.pi));c,s=np.cos(angle/2),np.sin(angle/2)
        w,x,y,z=task.Q_FIX;q=[c*w-s*z,c*x-s*y,c*y+s*x,c*z+s*w]
        r['spawn_quaternion']=q
        parts=convex_parts(body.get_collision_shapes());bounds.append(mesh_bounds([0,0,0]+q,parts))
        environment.objects[oid]=actor;environment._modelnames[oid]=r['modelname']
    poses=pack_layout(bounds,config.seed)
    for i,r in enumerate(records):
        actor=environment.objects[r['instance_id']]
        actor.actor.set_pose(sapien.Pose(poses[i],r['spawn_quaternion']))
        environment.add_prohibit_area(actor,padding=.02)
    environment.roundtrip_inventory=records


def identity_records(environment,task,meshes):
    layout=json.loads(Path(task.LAYOUT_PATH).read_text())
    records=getattr(environment,'roundtrip_inventory',inventory(layout,10))
    records={str(r['instance_id']):r for r in records}
    result={}
    for oid,actor in environment.objects.items():
        r=records[str(oid)];directory=Path('./assets/objects')/r['modelname']
        # Exact geometry/metadata allowlist, never copy assets or candidate pools.
        files=[directory/('model_data'+str(r['model_id'])+'.json')]
        files+=sorted((directory/'collision').glob('*.glb'))
        asset_hash=digest({str(p.relative_to(directory)):file_hash(p) for p in files})
        result[str(oid)]=dict(asset_id=r['asset_id'],modelname=r['modelname'],model_id=r['model_id'],
            asset_sha256=asset_hash,geometry_sha256=digest(meshes[str(oid)]),
            scale=np.asarray(actor.config['scale']).tolist(),actor_name=actor.actor.get_name())
    return result


def validate_layout(initial,earlier,meshes,contacts,config):
    errors=[];metrics={};lo=np.array(CONTAINER['center'])-np.array(CONTAINER['inner'])/2
    hi=np.array(CONTAINER['center'])+np.array(CONTAINER['inner'])/2
    footprints=[]
    # JSON manifests sort string IDs lexically. Use that same order before
    # serialization too, including error lists, and accurately sum binary64
    # areas. Runtime and independent audit must compare exactly, not tolerantly.
    for oid in sorted(initial['objects']):
        o=initial['objects'][oid]
        vectors=[np.asarray(o[k],float) for k in ('pose','linear_velocity','angular_velocity')]
        if any(v.shape!=shape or not np.isfinite(v).all() for v,shape in zip(vectors,((7,),(3,),(3,)))):
            errors.append('nonfinite_or_malformed_state:'+oid);metrics[oid]=dict(valid=False);continue
        a,b=mesh_bounds(o['pose'],meshes[oid]);p,r=pose_errors(o['pose'],earlier[oid])
        if not np.isfinite(np.r_[a,b,p,r]).all():
            errors.append('nonfinite_geometry_or_drift:'+oid);metrics[oid]=dict(valid=False);continue
        speed=float(np.linalg.norm(o['linear_velocity']));spin=float(np.linalg.norm(o['angular_velocity']))
        if np.any(a[:2]<lo-.001) or np.any(b[:2]>hi+.001) or a[2]<.751 or b[2]>1.28:errors.append('outside_container:'+oid)
        if p>config.layout_drift_m or r>config.layout_rotation_drift_deg or speed>config.speed_tolerance_m_s or spin>config.spin_tolerance_rad_s:errors.append('unstable:'+oid)
        footprints.append(float(np.prod(b[:2]-a[:2])))
        metrics[oid]=dict(drift_m=p,drift_deg=r,speed_m_s=speed,spin_rad_s=spin,aabb_min=a.tolist(),aabb_max=b.tolist())
    for c in contacts:
        if not np.isfinite(c['separation_min']):errors.append('nonfinite_contact:'+str(c['bodies']))
        if c['separation_min'] < -config.layout_penetration_m: errors.append('deep_initial_contact:'+str(c['bodies']))
    if len(initial['objects'])!=config.count:errors.append('object_count')
    return dict(valid=not errors,errors=errors,objects=metrics,container=CONTAINER,
                object_count=len(initial['objects']),footprint_aabb_sum_over_container_area=math.fsum(footprints)/float(np.prod(CONTAINER['inner'])),
                density_note='Sum of projected AABB areas / floor area, includes overlap; not union area or physical solid volume.')
