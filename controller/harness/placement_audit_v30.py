"""Independent replay of pinned extraction endpoint admissions; no native imports."""
import copy,json
from pathlib import Path
from provenance import digest,file_hash
from direct_core import ForwardModel
from return_world import ReturnWorld
from physics_geometry import bounds_for_parts
from placement_endpoint import assess_link7_endpoint
from placement_contract import geometry_binding


def expected_binding(run,manifest):
    run=Path(run);src=run/'snapshot/src'
    return dict(implementation_sha256=digest(manifest['source_sha256']),config_sha256=file_hash(run/'config.json'),
        physics_mesh_sha256=file_hash(run/'artifacts/physics-meshes.json'),
        fixtures={n:file_hash(src/'fixtures'/n) for n in ('panda.urdf','collision-boxes.json','robot-vertices.json')})


def verify_admissions(events,world_factory,binding,canonical):
    admissions={};pending={};transported={};extracted=set();replayed=[]
    for index,event in enumerate(events):
        kind=event['kind'];oid=str(event.get('object_id'))
        if kind=='placement_endpoint_admission':
            row=event['receipt'];sha=row['sha256'];body={k:v for k,v in row.items() if k!='sha256'}
            if (sha!=digest(body) or row['schema']!='placement-endpoint-admission/30'
                    or row['binding']!=binding or row['physical_completion_proved'] is not False
                    or row['arm'] not in ('left','right') or row['arm']!=event['arm']
                    or row['object_id']!=event['object_id'] or oid not in canonical
                    or row['state_sha256']!=digest(row['state'])):raise ValueError('endpoint_admission_binding')
            world=world_factory(row['state'])
            if row['geometry']!=geometry_binding(world):raise ValueError('endpoint_static_shape_identity')
            actual=assess_link7_endpoint(world,row['arm'],row['endpoint_q'],row['desired_position'],row['desired_rotation'])
            if actual!=row['assessment']:raise ValueError('endpoint_assessment_replay')
            if sha in admissions:raise ValueError('duplicate_endpoint_admission')
            admissions[sha]=(index,event);replayed.append(dict(sha256=sha,object_id=row['object_id'],safe=actual['safe']))
        elif kind=='placement_preflight':
            sha=event.get('endpoint_admission_sha256');item=admissions.get(sha)
            if item is None:raise ValueError('placement_missing_endpoint_admission')
            earlier,admission=item;row=admission['receipt']
            if (earlier>=index or row['assessment']['safe'] is not True or row['object_id']!=event['object_id']
                    or row['arm']!=event.get('arm') or row['slot']!=event['slot']
                    or row['endpoint_q']!=event['low_q'] or row['desired_position']!=event['low']
                    or row['desired_rotation']!=event['rotation'] or admission['physics_step']!=event['physics_step']):
                raise ValueError('placement_endpoint_chosen_target_mismatch')
            pending[oid]=(index,event)
        elif kind=='transported':
            if oid not in pending:raise ValueError('transport_missing_placement_preflight')
            _,pre=pending[oid]
            if (event.get('endpoint_admission_sha256')!=pre['endpoint_admission_sha256']
                    or event.get('arm')!=pre['arm'] or event['slot']!=pre['slot']
                    or event['physics_step']<pre['physics_step'] or oid in transported):
                raise ValueError('transport_endpoint_binding')
            transported[oid]=event
        elif kind=='extracted':
            if oid not in transported or oid in extracted:raise ValueError('extracted_missing_unique_endpoint_admission')
            extracted.add(oid)
    return dict(valid=True,replayed=replayed,preflight_objects=sorted(pending),transported_objects=sorted(transported),
                extracted_objects=sorted(extracted),physical_completion_proved=False)


def audit_admissions(run,manifest,events,scene):
    src=Path(run)/'snapshot/src';a=Path(run)/'artifacts'
    geometry=json.loads((src/'fixtures/collision-boxes.json').read_text());vertices=json.loads((src/'fixtures/robot-vertices.json').read_text())
    meshes=json.loads((a/'physics-meshes.json').read_text());model=ForwardModel(src/'fixtures/panda.urdf')
    for oid,parts in meshes.items():geometry['objects'].setdefault(oid,{})['parts']=bounds_for_parts(parts)
    def world(state):return ReturnWorld(model,copy.deepcopy(geometry),state,meshes,vertices,supports={},goals={},optimize=False)
    return verify_admissions(events,world,expected_binding(run,manifest),set(scene['objects']))
