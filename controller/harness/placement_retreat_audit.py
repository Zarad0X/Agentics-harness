"""Independent receipt reconstruction; does not call the candidate selector."""
import copy,hashlib,json
import numpy as np
def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
from return_clearance import ReturnClearance
from loaded_tracking import POLICY

def require(value,reason):
    if not value:raise ValueError('placement_retreat_'+reason)

def predicted_placement(endpoint,world_factory):
    """Independently derive placement from actual held-source data and mesh.

    The immutable main/endpoint_receipt AST contract pins field capture to the
    original relative transform used by loaded planning, not another forecast.
    """
    from provenance import digest as source_digest
    require(source_digest({k:v for k,v in endpoint.items() if k!='sha256'})==endpoint['sha256'],'endpoint_hash')
    source=endpoint.get('placement_source');require(isinstance(source,dict) and source.get('schema')=='placement-source/008','placement_source')
    world=world_factory(endpoint['state']);parts=world.meshes[str(endpoint['object_id'])]
    require(source['physical_parts_sha256']==source_digest(parts),'physical_parts')
    rel=np.asarray(source['held_relative'],float);rotation=np.asarray(endpoint['desired_rotation'],float);above=np.asarray(source['above'],float)
    require(rel.shape==(4,4) and rotation.shape==(3,3) and above.shape==(3,) and all(np.isfinite(x).all() for x in (rel,rotation,above)),'source_shape')
    require(np.array_equal(rel[3],[0,0,0,1]) and np.allclose(rel[:3,:3].T@rel[:3,:3],np.eye(3),rtol=0,atol=1e-6) and abs(np.linalg.det(rel[:3,:3])-1)<1e-6,'source_rigid')
    require(np.allclose(rotation.T@rotation,np.eye(3),rtol=0,atol=1e-6) and abs(np.linalg.det(rotation)-1)<1e-6,'source_rotation')
    offset=rotation@rel[:3,3];slot=endpoint['slot']
    require(above[:2].tolist()==[slot[0]-offset[0],slot[1]-offset[1]],'slot_formula')
    high=np.eye(4);high[:3,:3]=rotation;high[:3,3]=above;high=high@rel
    bottom=min((np.asarray(part['vertices'])@high[:3,:3].T+high[:3,3])[:,2].min() for part in parts)
    low=above-[0,0,max(0.,bottom-.77)]
    require(low.tolist()==endpoint['desired_position'],'lowering_formula')
    settled=high.copy();settled[2,3]+=.74-bottom
    return settled.tolist()

def verify_receipt(r,world_factory):
    require(r['schema']=='placement-unloaded-retreat/007' and r['physical_completion_proved'] is False,'schema')
    require(digest({k:v for k,v in r.items() if k!='sha256'})==r['sha256'],'hash')
    s=r['state'];arm=r['arm'];q=r['low_q'];end=r['retreat_q'];path=r['path'];controls=r['controls']
    require(digest(s)==r['state_sha256'] and digest(controls)==r['controls_sha256'],'source')
    require(r['fingers']==[.04,.04] and digest(path)==r['path_sha256'],'reference')
    require(path[0]==r['original_path'][0]==q and path[-1]==r['original_path'][-1]==end,'endpoints')
    future=copy.deepcopy(s);future['robots'][arm]['q']=q+[.04,.04]
    w=world_factory(future);w.set_object_transform(r['object_id'],np.asarray(r['predicted_transform']))
    require(w.held is None and getattr(w,'home_contact_envelope',None) is None,'unloaded_contact_scope')
    require(digest(dict(geometry=w.geometry,meshes=w.meshes,robot_vertices=w.robot_vertices))==r['geometry_sha256'],'geometry')
    c=ReturnClearance(w,arm,r['object_id'],True);cert=c.certify(path)
    require(cert==r['certificate'],'certificate')
    offset=[j['target'][0]-x for j,x in zip(controls[arm][:7],s['robots'][arm]['q'][:7])]
    require(max(map(abs,offset))<=POLICY['bias_limit_rad'],'source_bias')
    require([f['hypothesis'] for f in r['forecasts']]==['nominal','inherited_compensation'],'hypotheses')
    bounds=np.asarray(w.model.bounds)+[.03,-.03];safe=cert['certified']
    for f,bias in zip(r['forecasts'],([0.]*7,offset)):
        drive=np.asarray([x+b for x,b in zip(q,bias)],np.float32).astype(float).tolist()
        require(f['drive_q']==drive and f['route']==[drive]+path,'prefix')
        valid=bool(np.all(np.asarray(drive)>=bounds[:,0]) and np.all(np.asarray(drive)<=bounds[:,1]))
        require(f['drive_bounds_valid']==valid,'drive_bounds')
        point=c.point(drive) if valid else None;require(point==f['point'],'point')
        fc=c.certify(f['route']) if valid and point['safe'] and cert['certified'] else None
        require(fc==f['certificate'],'prefix_certificate');safe=bool(safe and fc is not None and fc['certified'])
    require(r['safe']==safe,'safe')
    return safe

def verify(events,world):
    receipts={};preflights={};committed={};lowering=set();count=0
    endpoints={};endpoint_steps={};contexts={}
    for e in events:
        kind=e['kind']
        if kind=='placement_endpoint_admission':
            endpoints[e['receipt']['sha256']]=e['receipt'];endpoint_steps[e['receipt']['sha256']]=e['physics_step']
        if kind=='placement_retreat_admission':
            r=e['receipt'];h=r['sha256'];require(h not in receipts,'duplicate')
            endpoint=endpoints.get(e['endpoint_admission_sha256']);require(endpoint is not None,'missing_endpoint')
            require(endpoint['state']==r['state'] and endpoint['endpoint_q']==r['low_q'],'lowering_source')
            require(e['arm']==r['arm']==endpoint['arm'] and e['object_id']==r['object_id']==endpoint['object_id'] and e['slot']==endpoint['slot'],'identity')
            require(e['physics_step']==endpoint_steps[endpoint['sha256']],'source_step')
            require(r['predicted_transform']==predicted_placement(endpoint,world),'placement_transform_formula')
            verify_receipt(r,world)
            receipts[h]=(r,e['endpoint_admission_sha256'],e['slot']);count+=1
        if kind=='placement_home_entry_context':
            matches=[(h,r,slot) for h,(r,key,slot) in receipts.items() if key==e['endpoint_admission_sha256']]
            require(len(matches)==1,'HOME_source');h,r,slot=matches[0];endpoint=endpoints[e['endpoint_admission_sha256']]
            require(h not in contexts and r['safe'] and e['arm']==r['arm'] and e['object_id']==r['object_id'] and e['slot']==slot,'HOME_identity')
            require(e['predicted_transform']==r['predicted_transform'] and e['retreat_q']==r['retreat_q'] and e['state_sha256']==endpoint['state_sha256'] and e['binding']==endpoint['binding'] and e['physics_step']==endpoint_steps[endpoint['sha256']],'HOME_geometry')
            contexts[h]=e
        if kind in ('placement_preflight','transported'):
            h=e.get('retreat_admission_sha256');require(h in receipts,'missing_admission');r,endpoint,slot=receipts[h]
            require(r['safe'] and r['arm']==e['arm'] and r['object_id']==e['object_id'] and endpoint==e['endpoint_admission_sha256'] and slot==e['slot'],'selection')
            if kind=='placement_preflight':
                require(e['low_q']==r['low_q'] and e['retreat_q']==r['retreat_q'],'selected_endpoints')
                source=endpoints[endpoint]
                require(h in contexts and e['rotation']==source['desired_rotation'] and e['low']==source['desired_position'] and e['above']==source['placement_source']['above'] and e['physics_step']==endpoint_steps[endpoint],'selected_placement_geometry')
                preflights[h]=e
            else:
                require(h in preflights,'missing_preflight');committed[e['object_id']]=r
        if kind=='custom_plan' and e.get('target') in committed and e['target'] not in lowering:
            r=committed[e['target']]
            require(e['ik']['q']==r['low_q'],'actual_lowering_endpoint')
            if e['path']['success']:require(e['path']['path'][-1]==r['low_q'],'actual_lowering_path')
            lowering.add(e['target'])
        if kind=='extraction_retreat_binding':
            require(e['object_id'] in committed and e['object_id'] in lowering,'unbound_retreat');r=committed[e['object_id']]
            require(e['arm']==r['arm'] and e['measured_path'][-1]==r['retreat_q'],'actual_endpoint')
    return dict(valid=True,admissions=count,committed=len(committed),native_completion_proved=False)
