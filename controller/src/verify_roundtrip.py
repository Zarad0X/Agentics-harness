"""Independent file/trajectory verifier. No planner or runner imports.

This checks recorded evidence, not an adversarially forged simulator. Source
hashes and read-only snapshots are part of the evidence chain. Missing evidence
fails closed. A smoke run can never become a complete roundtrip success.
"""
import argparse
import hashlib
import json
import math
from collections import deque
from pathlib import Path
import numpy as np


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def hashed(value):return hashlib.sha256((json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()).hexdigest()
def read(path):return json.loads(path.read_text())


def errors_between(a,b):
    a=np.asarray(a,float);b=np.asarray(b,float)
    if a.shape!=(7,) or b.shape!=(7,) or not np.isfinite(a).all() or not np.isfinite(b).all():raise ValueError('invalid pose')
    qa=a[3:];qb=b[3:];na=np.linalg.norm(qa);nb=np.linalg.norm(qb)
    if min(na,nb)<1e-12:raise ValueError('zero quaternion')
    qa=qa/na;qb=qb/nb
    dot=float(np.clip(abs(qa@qb),0.,1.))
    return float(np.linalg.norm(a[:3]-b[:3])),math.degrees(2*math.acos(dot))


def vertices(pose,parts):
    p=np.asarray(pose,float);w,x,y,z=p[3:]/np.linalg.norm(p[3:])
    r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    return np.concatenate([np.asarray(m['vertices']) for m in parts])@r.T+p[:3]


def norm(v,length):
    x=np.asarray(v,float)
    if x.shape!=(length,) or not np.isfinite(x).all():raise ValueError('invalid vector')
    return float(np.linalg.norm(x))


def in_staging(o,parts):
    v=vertices(o['pose'],parts);lo=v.min(0);hi=v.max(0)
    in_xy=((lo[0]>=-.85 and hi[0]<=-.31) or (lo[0]>=.29 and hi[0]<=.67)) and lo[1]>=-.74 and hi[1]<=.34
    return bool(in_xy and .737<=lo[2]<=.753 and norm(o['linear_velocity'],3)<=.03 and norm(o['angular_velocity'],3)<=.15)


def audit(run,source_root=None,*,continuous_diagnostic=False):
    run=Path(run)
    if continuous_diagnostic:
        marker=read(run/'DIAGNOSTIC_ONLY.json')
        if marker!=dict(schema='continuous-roundtrip/22',accepted=False,initialized_episode=True,reconstructed=False,formal_credit=False):
            raise ValueError('invalid_continuous_diagnostic_marker')
    elif (run/'DIAGNOSTIC_ONLY.json').exists() or (run/'diagnostic-result.json').exists():
        return dict(mode='return_diagnostic',accepted=False,whole_episode_success=False,
                    full_ten_success=False,errors=['diagnostic_reconstruction_not_continuous_episode'])
    m=read(run/'scene-manifest.json');cfg=m['config']
    if cfg['mode']=='layout':return dict(mode='layout',accepted=False,whole_episode_success=False,errors=['layout_only_not_roundtrip'])
    final=read(run/'final-state.json')
    result=read(run/'result.json');meshes=read(run/'physics-meshes.json');events=[json.loads(x) for x in (run/'events.jsonl').read_text().splitlines()]
    errors=[];records=[]
    ids=set(m['objects']);selected=set(m['selected_ids']);expected=cfg['count'];mode=cfg['mode']
    if len(ids)!=expected or len(m['selected_ids'])!=len(selected) or not selected<=ids:errors.append('manifest_ids')
    if mode!='smoke' and selected!=ids:errors.append('incomplete_selected_ids')
    if mode=='smoke' and (expected!=10 or len(selected)!=1 or cfg['layout']!='original'):errors.append('invalid_smoke_scope')
    if cfg['layout']=='original' and ids!=set(map(str,(3,4,8,9,10,11,12,17,18,19))):errors.append('original_ids_changed')
    if read(run/'effective-config.json')!=cfg:errors.append('configuration_changed')
    if not m['layout_validation']['valid'] or read(run/'layout-validation.json')!=m['layout_validation']:errors.append('layout_gate_failed')
    if sha(run/'scene-manifest.json')!=final['scene_manifest_sha256'] or final['scene_manifest_sha256']!=result['scene_manifest_sha256']:errors.append('manifest_hash')
    if sha(run/'physics-meshes.json')!=m['physics_mesh_sha256'] or set(meshes)!=ids:errors.append('geometry_hash')
    if source_root is not None:
        source_root=Path(source_root).resolve()
        for name,h in m['source_sha256'].items():
            p=(source_root/name).resolve()
            if not p.is_relative_to(source_root) or not p.is_file() or sha(p)!=h:errors.append('source_changed:'+name)
    else:errors.append('source_snapshot_not_checked')
    if result.get('failure') or result.get('forbidden_legacy_calls'):errors.append('runner_failure_or_legacy_calls')
    if continuous_diagnostic:
        if (result.get('schema')!='continuous-roundtrip-result/22' or result.get('accepted') is not False
                or result.get('formal_credit') is not False or result.get('reconstructed') is not False
                or result.get('initialized_episode') is not True or not cfg.get('reserve_home_workspace')):
            errors.append('continuous_diagnostic_contract')
        if result.get('diagnostic_completed') is not True:errors.append('runner_not_completed')
    elif result.get('accepted') is not True:errors.append('runner_not_completed')
    if final.get('formal_mutations')!=[] or final.get('identity_preserved') is not True:errors.append('formal_mutation_or_substitution')
    if final.get('held_id') is not None or final.get('drive_active') is not False or final.get('active_magnetic_ids')!=[]:errors.append('active_assistance')
    terminal=final['state']
    if set(terminal['objects'])!=ids:errors.append('terminal_ids')
    for arm in ('left','right'):
        q=np.asarray(terminal['robots'][arm]['q'],float);q0=np.asarray(m['robots'][arm]['q'],float)
        qdot=terminal['robots'][arm]['qdot']
        if q.shape!=(9,) or not np.isfinite(q).all() or np.max(np.abs(q[:7]-q0[:7]))>cfg['arm_joint_tolerance_rad']:errors.append('arm_not_home:'+arm)
        if norm(qdot,9)>cfg['arm_speed_tolerance_rad_s']:errors.append('arm_not_stable:'+arm)
    initial_events=[e for e in events if e['kind']=='initial_state']
    if len(initial_events)!=1 or initial_events[0]['state']['objects']!=m['objects']:errors.append('initial_goals_changed')
    if any(e['kind'] in ('state_restore','teleport','rollback','object_substitution') for e in events):errors.append('forbidden_event')
    steps=[e['physics_step'] for e in events]
    if any(b<a for a,b in zip(steps,steps[1:])):errors.append('event_step_rollback')
    barriers=[(j,e) for j,e in enumerate(events) if e['kind']=='extraction_barrier']
    extracted=set();barrier_index=-1
    # Retain independently measured partial extraction rates on failed runs.
    for j,e in enumerate(events):
        if e['kind']!='extracted':continue
        oid=str(e['object_id'])
        if oid not in selected:errors.append('unexpected_extracted_id');continue
        history=events[:j]
        lifted=any(x['kind']=='lifted' and str(x.get('object_id'))==oid and x['bottom_z']>=.925 for x in history)
        released=any(x['kind']=='released' and str(x.get('object_id'))==oid and x.get('settled') for x in history)
        if lifted and released and in_staging(e['state']['objects'][oid],meshes[oid]):extracted.add(oid)
    if len(barriers)!=1:errors.append('missing_or_duplicate_extraction_barrier')
    else:
        barrier_index,barrier=barriers[0]
        if set(map(str,barrier['selected_ids']))!=selected:errors.append('barrier_ids')
        for oid in selected:
            before=events[:barrier_index]
            lifted=any(e['kind']=='lifted' and str(e.get('object_id'))==oid and e['bottom_z']>=.925 for e in before)
            released=any(e['kind']=='released' and str(e.get('object_id'))==oid and e.get('settled') for e in before)
            recorded=any(e['kind']=='extracted' and str(e.get('object_id'))==oid for e in before)
            if not (lifted and released and recorded and in_staging(barrier['state']['objects'][oid],meshes[oid])):errors.append('not_extracted_at_barrier:'+oid)
        if any(e['kind'] in ('return_started','return_release','returned') for e in events[:barrier_index]):errors.append('return_before_all_extracted')
    names={o['actor_name'] for o in m['objects'].values()}
    allowed_pairs={frozenset(c['bodies']) for c in m['initial_contacts']}
    for c in final['contacts']:
        pair=frozenset(c['bodies']);significant=c['impulse_max']>.002 or c['separation_min']<-.0003
        if not significant:continue
        if any(n.startswith('panda_') for n in pair):errors.append('terminal_robot_contact:'+str(sorted(pair)))
        if pair & names:
            if c['separation_min'] < -cfg['layout_penetration_m']:errors.append('terminal_deep_contact:'+str(sorted(pair)))
            if mode!='extraction' and pair not in allowed_pairs:errors.append('unexpected_final_contact:'+str(sorted(pair)))
    returned=set()
    for oid in sorted(ids):
        o=terminal['objects'][oid];initial=m['objects'][oid];pe,re=errors_between(o['pose'],initial['pose'])
        speed=norm(o['linear_velocity'],3);spin=norm(o['angular_velocity'],3)
        identity_ok=all(o[k]==initial[k] for k in ('asset_sha256','geometry_sha256','asset_id','modelname','model_id','scale','actor_name'))
        identity_ok=identity_ok and hashed(meshes[oid])==initial['geometry_sha256']
        stable=speed<=cfg['speed_tolerance_m_s'] and spin<=cfg['spin_tolerance_rad_s']
        exact=pe<=cfg['position_tolerance_m'] and re<=cfg['rotation_tolerance_deg']
        history=[(i,e) for i,e in enumerate(events) if str(e.get('object_id'))==oid]
        started=[i for i,e in history if e['kind']=='return_started'];released=[i for i,e in history if e['kind']=='return_release'];done=[i for i,e in history if e['kind']=='returned']
        physical=len(started)==len(released)==len(done)==1 and barrier_index<started[0]<released[0]<done[0]
        if physical:
            previews=[e for i,e in history if barrier_index<i<started[0] and e['kind']=='return_preview' and e.get('accepted') and e.get('source_unchanged') and e.get('independent_scene')]
            repeated=any({e['trial'] for e in previews if e['index']==index}>={1,2} for index in {e['index'] for e in previews})
            attached=any(started[0]<i<released[0] and e['kind']=='magnet_attached' for i,e in history)
            lifted=any(started[0]<i<released[0] and e['kind']=='lifted' and e['bottom_z']>=.925 for i,e in history)
            physical=physical and repeated and attached and lifted
        if oid in selected and mode!='extraction' and exact and stable and identity_ok and physical:returned.add(oid)
        passed=identity_ok and stable and (exact if mode!='extraction' else (oid not in selected or in_staging(o,meshes[oid])))
        if oid in selected and mode!='extraction':passed=passed and physical
        if not passed:errors.append('object_final_failed:'+oid)
        records.append(dict(object_id=oid,position_error_m=pe,rotation_error_deg=re,symmetry_adjusted_error_deg=None,
            speed_m_s=speed,spin_rad_s=spin,exact_pose=exact,identity_ok=identity_ok,stable=stable,extracted=oid in extracted,
            physically_returned=oid in returned,passed=bool(passed)))
    # Stream compact trajectory: do not load a long episode into RAM.
    previous=None;count=0;barrier_witness=False;tail=deque()
    with (run/'trajectory.jsonl').open() as f:
        for line in f:
            row=json.loads(line);count+=1
            tail.append(row)
            while tail and row['physics_step']-tail[0]['physics_step']>125:tail.popleft()
            if barriers and row['physics_step']==barriers[0][1]['physics_step'] and row['state']==barriers[0][1]['state'] and row['held_id'] is None and row['drive_active'] is False:barrier_witness=True
            if set(row['state']['objects'])!=ids:errors.append('trajectory_ids');break
            if previous:
                delta=row['physics_step']-previous['physics_step']
                if delta<0 or delta>25:errors.append('trajectory_gap_or_rollback');break
                for oid in ids:
                    pe,_=errors_between(row['state']['objects'][oid]['pose'],previous['state']['objects'][oid]['pose'])
                    if pe>.20:errors.append('trajectory_discontinuity:'+oid)
            elif row['physics_step']!=0:errors.append('trajectory_missing_initial')
            previous=row
    if previous is None or previous['physics_step']!=final['physics_step'] or previous['state']!=terminal:errors.append('trajectory_terminal_mismatch')
    if count!=final['trajectory_samples'] or sha(run/'trajectory.jsonl')!=final['trajectory_sha256']:errors.append('trajectory_hash_or_count')
    if not barrier_witness:errors.append('extraction_barrier_not_in_measured_trace')
    if not tail or tail[-1]['physics_step']-tail[0]['physics_step']<100:errors.append('missing_terminal_stability_window')
    for row in tail:
        if row['held_id'] is not None or row['drive_active']:errors.append('assistance_in_terminal_window')
        for oid,o in row['state']['objects'].items():
            if norm(o['linear_velocity'],3)>cfg['speed_tolerance_m_s'] or norm(o['angular_velocity'],3)>cfg['spin_tolerance_rad_s']:errors.append('unstable_terminal_window:'+oid)
    if set(map(str,final['completed_ids']))!=extracted or len(final['completed_ids'])!=len(extracted):errors.append('completed_ids_mismatch')
    if mode!='extraction' and (set(map(str,final['returned_ids']))!=returned or len(final['returned_ids'])!=len(returned)):errors.append('returned_ids_mismatch')
    verified=not errors;whole=verified and mode=='roundtrip' and extracted==ids and returned==ids
    return dict(schema='continuous-roundtrip-verification/22' if continuous_diagnostic else 'roundtrip-verification/1',
        **(dict(continuous_diagnostic_passed=whole,formal_credit=False) if continuous_diagnostic else {}),
        accepted=verified and not continuous_diagnostic,whole_episode_success=whole and not continuous_diagnostic,partial=mode=='smoke',mode=mode,
        errors=sorted(set(errors)),objects=records,extracted_count=len(extracted),exact_return_count=len(returned),scene_count=expected,
        extraction_rate=len(extracted)/expected,exact_pose_return_rate=len(returned)/expected,
        evidence_scope='recorded measured states and source hashes; no adversarial attestation or continuous swept-volume proof',
        elapsed_s=result.get('phase_elapsed_s'),physical_steps=result.get('physics_steps'),preview_steps=result.get('physics_preview_steps'),
        phase_physical_steps=result.get('phase_physical_steps'),phase_preview_steps=result.get('phase_preview_steps'),
        candidates=sum(e['kind'] in ('candidate_considered','return_candidate') for e in events),
        previews=sum(e['kind'] in ('grasp_preview','return_preview','separation_preview') for e in events),
        runtime_llm=m['runtime_llm'],development_llm=m['development_llm'])


def safe_audit(run,source_root=None):
    try:return audit(run,source_root)
    except Exception as exc:return dict(accepted=False,whole_episode_success=False,errors=['missing_or_malformed_evidence:'+type(exc).__name__+':'+str(exc)])


def main():
    ap=argparse.ArgumentParser();ap.add_argument('run',type=Path);ap.add_argument('--source-root',required=True,type=Path)
    args=ap.parse_args();report=safe_audit(args.run,args.source_root)
    with (args.run/'independent-verification.json').open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
    print(json.dumps(report,indent=2));return 0 if report['accepted'] else 2


if __name__=='__main__':raise SystemExit(main())
