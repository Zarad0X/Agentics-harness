"""Recorded payloads, or narrowly bound DERIVED v2 ordinal reconstruction.

No reconstruction of RNG/IK/physics history is claimed. The v2 enumerator's
five depth batches and per-batch truncation are deliberately retained here;
the current return search has different ordering and must not substitute.
"""
import copy
from pathlib import Path
import numpy as np
from provenance import digest,file_hash
from collision_world import pose_matrix
from surface_grasps import mesh_grasps

V2_BINDING='7825ef1688eb56ccca19c528d5a205f15b4bceb3d18f32f104d5b05dd75430e3'
N11_STATE='f17a44d95736fdc1ffad43670c810f566b1b261987c92c5eac09c9064e62f01a'


def select_candidate(bundle,index,model):
    target=bundle['target'];events=bundle['events'];s=bundle['final']['state']
    matches=[(i,e) for i,e in enumerate(events) if e['kind']=='return_candidate'
             and e['object_id']==target and e['index']==index]
    if len(matches)!=1:raise ValueError('recorded_return_candidate_missing_or_ambiguous')
    offset,event=matches[0]
    if event['phase']!='return' or event['physics_step']!=bundle['final']['physics_step']:
        raise ValueError('recorded_candidate_phase_or_step')
    arm=event.get('arm','right' if s['objects'][str(target)]['pose'][0]>=0 else 'left')
    provenance=dict(ordinal=index,arm=arm,event_sha256=digest(event),
        original_line_1based=bundle['receipt']['selected_line_numbers_1based'][offset],
        original_events_sha256=bundle['receipt']['original_sha256'],source_state_sha256=digest(s),
        cooked_mesh_sha256=bundle['hashes']['artifacts/physics-meshes.json'])
    if 'candidate' in event:
        c=copy.deepcopy(event['candidate']);provenance['payload_origin']='RECORDED'
    else:
        if (target!=12 or digest(s)!=N11_STATE or bundle['config']['experience'] is not False
                or digest(bundle['manifest']['source_sha256'])!=V2_BINDING):
            raise ValueError('unsupported_unrecorded_candidate_source')
        if set(event)!={'kind','time_unix','physics_step','phase','object_id','index'}:
            raise ValueError('unsupported_old_candidate_schema')
        prefix=[e for e in events[:offset+1] if e['kind']=='return_candidate' and e['object_id']==target]
        if ([e['index'] for e in prefix]!=list(range(index+1)) or
                any(e['phase']!='return' or e['physics_step']!=167007 for e in prefix)):
            raise ValueError('old_candidate_ordinal_sequence')
        # These dependencies are byte-identical to the verified frozen v2.
        for name in ('surface_grasps.py','collision_world.py'):
            if file_hash(Path(__file__).with_name(name))!=bundle['manifest']['source_sha256'][name]:
                raise ValueError('old_candidate_algorithm_dependency:'+name)
        candidates=[];pose=pose_matrix(s['objects'][str(target)]['pose'])
        for depths in ((.014,.025,.04,.06),(.008,),(.004,),(.001,),(.0005,)):
            candidates.extend(mesh_grasps(pose,bundle['meshes'][str(target)],depths)[:24])
        if not 0<=index<len(candidates):raise ValueError('old_candidate_ordinal_out_of_range')
        c=candidates[index]
        provenance.update(payload_origin='DERIVED',algorithm_source_binding=V2_BINDING,
            enumeration='frozen v2 depth batches, first 24 per batch, experience-off identity ordering',
            candidate_count=len(candidates),recorded_payload=False,
            limitation='Fresh IK/RNG and reconstructed physics; not old trajectory replay.')
        # Candidate6 is the accepted old ordinal, followed by formal grasp.
        # Independently check its two Cartesian targets and logged IK endpoints.
        if index==6:
            starts=[i for i,e in enumerate(events) if e['kind']=='return_started' and e['object_id']==12]
            if len(starts)!=1 or starts[0]<=offset:raise ValueError('old_candidate_downstream_phase')
            between=events[offset+1:starts[0]]
            if [(e.get('index'),e.get('trial'),e.get('accepted')) for e in between]!=[(6,1,True),(6,2,True)]:
                raise ValueError('old_candidate_downstream_previews')
            downstream=events[starts[0]+1:]
            tracking=[e for e in downstream if e['kind']=='physical_tracking' and e['target']==12][:2]
            plans=[e for e in downstream if e['kind']=='custom_plan'][:2]
            if len(tracking)!=2 or len(plans)!=2:raise ValueError('old_candidate_downstream_missing')
            checks=[]
            for rise,t,p in zip((.1,0.),tracking,plans):
                wanted=np.asarray(c['ee_position_m'])+[0,0,rise];rotation=np.asarray(c['ee_rotation_world'])
                if not np.allclose(t['target_position'],wanted,rtol=0,atol=1e-12) or not np.allclose(t['target_rotation'],rotation,rtol=0,atol=1e-12):
                    raise ValueError('derived_candidate_downstream_target_mismatch')
                fk=model.evaluate(arm,p['ik']['q'])
                pe=float(np.linalg.norm(np.asarray(fk['ee_position_m'])-wanted))
                re=float(np.max(np.abs(np.asarray(fk['ee_rotation_world'])-rotation)))
                if pe>1e-4 or re>1e-3 or p['path']['path'][-1]!=p['ik']['q']:
                    raise ValueError('derived_candidate_downstream_ik_mismatch')
                checks.append(dict(target_event_sha256=digest(t),plan_event_sha256=digest(p),
                                   fk_position_error_m=pe,fk_rotation_max_error=re))
            provenance['original_downstream_checks']=checks
    p=np.asarray(c['ee_position_m']);r=np.asarray(c['ee_rotation_world'])
    if arm not in ('left','right') or p.shape!=(3,) or r.shape!=(3,3) or not np.isfinite(p).all() or not np.isfinite(r).all():
        raise ValueError('invalid_diagnostic_candidate')
    provenance['candidate_sha256']=digest(c)
    return arm,c,provenance
