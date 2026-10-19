"""DIAGNOSTIC ONLY: initialize a new scene from an immutable measured barrier.

This is not continuation/replay of a formal episode. Solver/contact caches,
original drive targets, RNG checkpoint and articulation root poses were not
recorded. Asset-defined robot roots and measured-q zero-velocity drive targets
are used explicitly. No whole-episode acceptance can originate here.
"""
import json
from pathlib import Path
import numpy as np
from provenance import file_hash,digest
from pose_goals import pose_errors

FILES=('config.json','run-manifest.json','event-subset-receipt.json',
       'artifacts/final-state.json','artifacts/scene-manifest.json',
       'artifacts/physics-meshes.json','artifacts/return-events-subset.jsonl')
LIMITATIONS=['Hidden solver/contact caches unavailable; reconstruction is not exact physical replay.',
             'Original drive targets/properties, RNG checkpoint and articulation roots absent from frozen state; '
             'measured q targets, zero drive velocity, freshly initialized drive properties and asset-defined roots used.',
             'Fresh continuous full10 extraction-then-return required before any validity/performance claim.']


def load_bundle(path,target=10):
    path=Path(path).resolve(strict=True)
    if target not in (10,12):raise ValueError('diagnostic_target')
    event_file='artifacts/return-events-subset.jsonl' if target==10 else 'artifacts/return-phase-events.jsonl'
    files=FILES[:-1]+(event_file,)
    hashes={n:file_hash(path/n) for n in files}
    read=lambda n:json.loads((path/n).read_text())
    final=read('artifacts/final-state.json');manifest=read('artifacts/scene-manifest.json')
    config=read('config.json');receipt=read('event-subset-receipt.json')
    events=[json.loads(l) for l in (path/event_file).read_text().splitlines()]
    if receipt.get('schema')!='immutable-event-subset/1' or len(events)!=len(receipt['selected_line_numbers_1based']):
        raise ValueError('diagnostic_subset_receipt')
    lines=receipt['selected_line_numbers_1based']
    if lines!=sorted(set(lines)) or not lines or lines[0]<1 or lines[-1]>receipt['original_lines']:
        raise ValueError('diagnostic_subset_lines')
    if config!=manifest['config'] or config['split']!='development' or config['layout']!='original' or config['count']!=10:
        raise ValueError('diagnostic_scope_requires_original10_development')
    if final['scene_manifest_sha256']!=hashes['artifacts/scene-manifest.json'] or manifest['physics_mesh_sha256']!=hashes['artifacts/physics-meshes.json']:
        raise ValueError('diagnostic_evidence_hash_mismatch')
    if final.get('drive_active') is not False or final.get('held_id') is not None or final.get('active_magnetic_ids')!=[]:
        raise ValueError('diagnostic_active_assistance')
    ids=set(manifest['objects'])
    if (set(final['state']['objects'])!=ids or set(map(str,final['completed_ids']))!=ids or set(manifest['selected_ids'])!=ids):
        raise ValueError('diagnostic_requires_complete_extraction')
    source_event=None
    if target==10:
        if final['returned_ids']:raise ValueError('diagnostic_requires_no_returns')
        barriers=[e for e in events if e['kind']=='extraction_barrier']
        if len(barriers)!=1 or barriers[0]['state']!=final['state']:
            raise ValueError('diagnostic_source_not_frozen_barrier')
    else:
        # Narrow extension: the COMPLETE supplied return-phase interval has
        # N10/N11 returned snapshots and a later N12 release/home failure.
        # Reconstruct ONLY the last completed N11 snapshot, never the collided
        # N12 terminal pose. No fictitious extraction/return ledger is created.
        if lines!=list(range(882,1155)) or set(map(int,final['returned_ids']))!={10,11}:
            raise ValueError('n12_requires_complete_recorded_return_phase')
        returned=[(i,e) for i,e in enumerate(events) if e['kind']=='returned']
        if [e['object_id'] for i,e in returned]!=[10,11]:raise ValueError('n12_completed_return_order')
        i,e=returned[-1]
        if not any(x['kind']=='return_release' and x['object_id']==11 for x in events[:i]):
            raise ValueError('n12_source_missing_release')
        if not any(x['kind']=='return_started' and x['object_id']==12 and x['physics_step']==e['physics_step'] for x in events[i+1:]):
            raise ValueError('n12_source_not_next_return_start')
        source_event=dict(kind='returned',object_id=11,original_line_1based=lines[i],
                          original_events_sha256=receipt['original_sha256'],event_sha256=digest(e),
                          physics_step=e['physics_step'],assistance_state='inferred from completed return and verified release code; controls/history absent')
        final=dict(state=e['state'],physics_step=e['physics_step'],homes=final['homes'],
                   completed_ids=final['completed_ids'],returned_ids=[10,11],held_id=None,drive_active=False,
                   active_magnetic_ids=[],contacts=None,source_event=source_event)
    for robot in final['state']['robots'].values():
        for k in ('q','qdot'):
            v=np.asarray(robot[k]);
            if v.shape!=(9,) or not np.isfinite(v).all():raise ValueError('invalid_diagnostic_robot_state')
    for o in final['state']['objects'].values():
        for k,n in (('pose',7),('linear_velocity',3),('angular_velocity',3)):
            v=np.asarray(o[k])
            if v.shape!=(n,) or not np.isfinite(v).all():raise ValueError('invalid_diagnostic_object_state')
        if abs(np.linalg.norm(o['pose'][3:])-1)>1e-5:raise ValueError('invalid_diagnostic_quaternion')
    return dict(path=path,hashes=hashes,final=final,manifest=manifest,config=config,target=target,source_event=source_event,
                meshes=read('artifacts/physics-meshes.json'),events=events,receipt=receipt,
                run=read('run-manifest.json'),source_fingerprint=digest(final['state']))


def check_bundle_unchanged(bundle):
    if {n:file_hash(bundle['path']/n) for n in bundle['hashes']}!=bundle['hashes']:
        raise RuntimeError('diagnostic_input_modified')


def reconstruction_errors(expected, actual):
    rows={};errors=[]
    if set(expected['objects'])!=set(actual['objects']) or set(actual['robots'])!={'left','right'}:
        raise ValueError('diagnostic_reconstructed_ids')
    for oid,o in expected['objects'].items():
        a=actual['objects'][oid];pe,re=pose_errors(a['pose'],o['pose'])
        lv=float(np.max(np.abs(np.asarray(a['linear_velocity'])-o['linear_velocity'])))
        av=float(np.max(np.abs(np.asarray(a['angular_velocity'])-o['angular_velocity'])))
        keys=('actor_name','asset_id','asset_sha256','geometry_sha256','model_id','modelname','scale','extents')
        same=all(a[k]==o[k] for k in keys)
        rows[oid]=dict(position_error_m=pe,rotation_error_deg=re,linear_velocity_max_error=lv,
                       angular_velocity_max_error=av,identity_geometry_equal=same)
        if pe>1e-6 or re>1e-4 or max(lv,av)>1e-7 or not same:errors.append('object:'+oid)
    robots={}
    for arm,o in expected['robots'].items():
        robots[arm]={k:float(np.max(np.abs(np.asarray(actual['robots'][arm][k])-o[k]))) for k in ('q','qdot')}
        if max(robots[arm].values())>1e-7:errors.append('robot:'+arm)
    return dict(objects=rows,robots=robots,errors=errors,validated=not errors)


def reconstruct(bundle, environment, sapien, state, meshes):
    """Only call on a freshly created diagnostic scene, before any previews."""
    if meshes!=bundle['meshes']:raise RuntimeError('diagnostic_cooked_geometry_mismatch')
    expected=bundle['final']['state']
    # Verify assets/identity before changing any pose in the new scene.
    before=state(environment)
    for oid,o in expected['objects'].items():
        for k in ('asset_sha256','geometry_sha256','actor_name','scale','model_id'):
            if before['objects'][oid][k]!=o[k]:raise RuntimeError('diagnostic_asset_mismatch:'+oid+':'+k)
    for arm,o in expected['robots'].items():
        entity=getattr(environment.robot,arm+'_entity');entity.set_qpos(o['q']);entity.set_qvel(o['qdot'])
        joints=entity.get_active_joints()
        if len(joints)!=len(o['q']):raise RuntimeError('diagnostic_joint_layout')
        for joint,q in zip(joints,o['q']):
            joint.set_drive_target(float(q));joint.set_drive_velocity_target(0.)
    cls=sapien.pysapien.physx.PhysxRigidDynamicComponent
    for oid,o in expected['objects'].items():
        actor=environment.objects[int(oid)].actor
        actor.set_pose(sapien.Pose(o['pose'][:3],o['pose'][3:]))
        body=actor.find_component_by_type(cls)
        body.set_linear_velocity(o['linear_velocity']);body.set_angular_velocity(o['angular_velocity'])
    actual=state(environment);report=reconstruction_errors(expected,actual)
    report.update(schema='return-reconstruction/7',diagnostic_only=True,source_fingerprint=bundle['source_fingerprint'],
                  reconstructed_fingerprint=digest(actual),input_sha256=bundle['hashes'],limitations=LIMITATIONS,
                  source_event=bundle.get('source_event'),diagnostic_target=bundle.get('target',10),
                  active_magnetic_ids=[],drive_active=False,steps_before_measurement=0)
    return report


def diagnostic_result(**values):
    # Fixed flags are assigned last; even a successful pair of previews cannot
    # masquerade as a smoke, dataset result or continuous whole10 episode.
    return dict(values, schema='return-diagnostic/7',status='DIAGNOSTIC ONLY',mode='return_diagnostic',
                accepted=False,whole_episode_success=False,full_ten_success=False,dataset_success=False,
                completed_ids=[],returned_ids=[],limitations=LIMITATIONS)
