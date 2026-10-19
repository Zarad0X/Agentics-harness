"""Independent nominal HOME-entry receipt replay, before placement transport.

This gate audits a necessary endpoint condition, never a physical completion.
The reviewed online helper is deliberately NOT imported or called here.
"""
import copy,json
from pathlib import Path
import numpy as np
from provenance import digest,file_hash
from direct_core import ForwardModel
from physics_geometry import bounds_for_parts
from return_world import ReturnWorld
from return_clearance import ReturnClearance
from home_interaction import ContactEnvelope
from placement_audit_v30 import expected_binding


def verify_entries(events,world_factory,binding,canonical):
    model=None;model_sha=None;reservation=None;endpoint=None;context=None
    last_entry=None;pending={};transported={};extracted=set();replayed=[];contexts=0
    def phase(e):
        if (e.get('phase')!='extraction' or type(e.get('physics_step')) is not int or e['physics_step']<0):
            raise ValueError('HOME_entry_phase_step')
    def identity(e,c):
        return all(e.get(k)==c.get(k) for k in ('object_id','arm','slot','physics_step','phase'))
    for index,e in enumerate(events):
        kind=e['kind']
        if kind=='home_workspace_model':
            if model is not None or e['model_sha256']!=digest(e['model']):raise ValueError('HOME_entry_model_binding')
            model=e['model'];model_sha=e['model_sha256']
        elif kind=='home_workspace_reservation' and e.get('stage')=='predicted_slot':
            reservation=e;endpoint=None;context=None;last_entry=None
        elif kind=='placement_endpoint_admission':
            endpoint=e;context=None;last_entry=None
        elif kind=='placement_home_entry_context':
            phase(e);contexts+=1
            if context is not None:raise ValueError('HOME_entry_incomplete_assessment')
            if model is None or e.get('binding')!=binding or e.get('home_contact_model_sha256')!=model_sha:
                raise ValueError('HOME_entry_source_model_binding')
            if endpoint is None or reservation is None:raise ValueError('HOME_entry_missing_prior_context')
            p=endpoint['receipt'];r=reservation
            if (p['sha256']!=digest({k:v for k,v in p.items() if k!='sha256'}) or p['binding']!=binding
                    or p['assessment']['safe'] is not True or p['sha256']!=e['endpoint_admission_sha256']
                    or p['arm']!=e['arm'] or p['object_id']!=e['object_id'] or p['slot']!=e['slot']
                    or endpoint['physics_step']!=e['physics_step'] or endpoint.get('phase')!='extraction'
                    or p['state_sha256']!=e['state_sha256'] or p['state_sha256']!=digest(p['state'])):
                raise ValueError('HOME_entry_endpoint_association')
            if (r.get('phase')!='extraction' or r['physics_step']!=e['physics_step'] or r['safe'] is not True
                    or r['object_id']!=e['object_id'] or r['predicted_object_id']!=e['object_id']
                    or r['model_sha256']!=model_sha or r['state_sha256']!=e['state_sha256']
                    or digest(r['state'])!=e['state_sha256'] or r['state']!=p['state']
                    or r['predicted_transform']!=e['predicted_transform']):
                raise ValueError('HOME_entry_reservation_association')
            context=(index,e);last_entry=None
        elif kind=='placement_home_entry_admission':
            phase(e)
            if context is None or index!=context[0]+1 or not identity(e,context[1]):raise ValueError('HOME_entry_assessment_association')
            c=context[1];row=e['receipt'];body={k:v for k,v in row.items() if k!='sha256'}
            if (row['sha256']!=digest(body) or row['schema']!='placement-HOME-entry-preflight/1'
                    or row['physical_completion_proved'] is not False or row['conditional_on_predicted_placement'] is not True
                    or row['scope']!='necessary nominal HOME start condition only' or type(row['safe']) is not bool
                    or row['arm']!=e['arm'] or row['object_id']!=e['object_id'] or str(row['object_id']) not in canonical
                    or row['state_sha256']!=digest(row['state']) or row['state_sha256']!=c['state_sha256']
                    or row['state']!=endpoint['receipt']['state'] or row['home_contact_model_sha256']!=model_sha
                    or row['predicted_transform']!=c['predicted_transform'] or row['retreat_q']!=c['retreat_q']
                    or row['fingers']!=[.04,.04]):raise ValueError('HOME_entry_receipt_binding')
            state=row['state'];q=np.asarray(row['retreat_q'],float);t=np.asarray(row['predicted_transform'],float)
            if (set(state['objects'])!=canonical or set(state['robots'])!={'left','right'} or row['arm'] not in ('left','right')
                    or q.shape!=(7,) or not np.isfinite(q).all() or t.shape!=(4,4) or not np.isfinite(t).all()
                    or not np.allclose(t[3],[0,0,0,1],atol=1e-12,rtol=0)
                    or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-6,rtol=0)
                    or abs(np.linalg.det(t[:3,:3])-1)>1e-6):raise ValueError('HOME_entry_replay_inputs')
            # Independent reconstruction of the hypothesis, not of an actual
            # scene or of historical commanded controls/RNG.
            future=copy.deepcopy(state);future['robots'][row['arm']]['q']=q.tolist()+[.04,.04]
            world=world_factory(future)
            if world.held is not None:raise ValueError('HOME_entry_loaded_replay')
            envelope=ContactEnvelope(model,world)
            if envelope.sha256!=model_sha:raise ValueError('HOME_entry_contact_model')
            envelope.attach(world);world.set_object_transform(row['object_id'],t)
            actual=ReturnClearance(world,row['arm'],row['object_id'],False).point(q.tolist())
            if actual!=row['point'] or actual['safe'] is not row['safe']:raise ValueError('HOME_entry_recomputed_point')
            replayed.append(dict(index=index,object_id=row['object_id'],receipt_sha256=row['sha256'],safe=actual['safe'],point=actual))
            last_entry=(index,e,c);context=None
        elif kind=='placement_rejected':
            if e.get('reason')=='retreat_endpoint_blocks_HOME_entry':
                if last_entry is None or last_entry[1]['receipt']['safe'] is not False or e.get('entry')!=last_entry[1]['receipt']:
                    raise ValueError('HOME_entry_rejection_receipt')
            last_entry=None
        elif kind=='placement_preflight':
            phase(e)
            if last_entry is None:raise ValueError('HOME_entry_missing_selected_admission')
            _,entry,c=last_entry;row=entry['receipt']
            if (not identity(e,c) or row['safe'] is not True or e.get('home_entry_admission_sha256')!=row['sha256']
                    or e['endpoint_admission_sha256']!=c['endpoint_admission_sha256'] or e['retreat_q']!=row['retreat_q']):
                raise ValueError('HOME_entry_selected_association')
            pending[str(e['object_id'])]=e;last_entry=None
        elif kind=='transported':
            phase(e);oid=str(e['object_id']);pre=pending.get(oid)
            if (pre is None or oid in transported or e.get('home_entry_admission_sha256')!=pre.get('home_entry_admission_sha256')
                    or e.get('endpoint_admission_sha256')!=pre['endpoint_admission_sha256'] or e['arm']!=pre['arm']
                    or e['slot']!=pre['slot'] or e['physics_step']<pre['physics_step']):raise ValueError('HOME_entry_transport_association')
            transported[oid]=e
        elif kind=='extracted':
            oid=str(e['object_id'])
            if oid not in transported or oid in extracted:raise ValueError('HOME_entry_extracted_coverage')
            extracted.add(oid)
    if context is not None:raise ValueError('HOME_entry_incomplete_assessment')
    return dict(valid=True,replayed=replayed,contexts=contexts,selected_objects=sorted(pending),transported_objects=sorted(transported),
                extracted_objects=sorted(extracted),scope='necessary nominal HOME entry only',physical_completion_proved=False)


def audit_entries(run,manifest,events,scene):
    run=Path(run);src=run/'snapshot/src';a=run/'artifacts'
    if file_hash(src/'home_entry_preflight.py')!='6ba913c0308321033f978da402de3a5a3864de2e3a2418d6e7376e1a347b421c':
        raise ValueError('HOME_entry_helper_source')
    geometry=json.loads((src/'fixtures/collision-boxes.json').read_text());vertices=json.loads((src/'fixtures/robot-vertices.json').read_text())
    meshes=json.loads((a/'physics-meshes.json').read_text());model=ForwardModel(src/'fixtures/panda.urdf')
    for oid,parts in meshes.items():geometry['objects'].setdefault(oid,{})['parts']=bounds_for_parts(parts)
    cfg=json.loads((run/'config.json').read_text())
    def world(state):return ReturnWorld(model,copy.deepcopy(geometry),state,meshes,vertices,supports={},goals={},optimize=cfg['optimize_geometry'])
    return verify_entries(events,world,expected_binding(run,manifest),set(scene['objects']))
