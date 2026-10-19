"""Strict v22 functional receipt audit; actual v21 progress baseline may be reused.

Different source versions are explicitly allowlisted and all snapshot bytes
checked. Unchanged actual pre-HOME prefix/model/curve is required. Adaptive
HOME timing is intentional. Raw failures never become physical successes.
"""
import argparse
import copy
import json
from pathlib import Path
import sys
import math
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'harness')]
from provenance import digest,file_hash,source_files,write_once
from home_functional_v20 import prefix,replay,IGNORED_PREFIX_FIELDS
from home_trace_v18 import validate_trace
from n17_pair_v19 import observation_checks
from home_progress import BOUND_POLICY as POLICY


def outcome(result,preview,manifest):
    """Never read the nonexistent top-level preview_completed field."""
    if result.get('schema')!='return-diagnostic/7':raise ValueError('result_schema')
    identity={k:preview[k] for k in ('arm','index','trial')}
    if identity['arm'] not in ('left','right') or any(type(identity[k]) is not int for k in ('index','trial')):
        raise ValueError('raw_preview_identity_type')
    if identity['trial']!=1 or identity['index']!=manifest['candidate_index'] or preview['object_id']!=manifest['target']:
        raise ValueError('raw_preview_request_identity')
    rows=result['previews']
    if not isinstance(rows,list) or len(rows)!=1:raise ValueError('result_single_preview_required')
    row=rows[0]
    if any(type(row.get(k)) is not type(v) or row.get(k)!=v for k,v in identity.items()):raise ValueError('result_preview_identity')
    completed=row.get('preview_completed')
    if type(completed) is not bool or type(preview.get('accepted')) is not bool or completed!=preview['accepted']:
        raise ValueError('result_raw_outcome_disagreement')
    if (completed and preview.get('failure') is not None) or (not completed and not isinstance(preview.get('failure'),str)):
        raise ValueError('raw_failure_outcome_disagreement')
    if type(preview.get('trial_steps')) is not int or type(result.get('physical_preview_steps')) is not int or result['physical_preview_steps']!=preview['trial_steps']:
        raise ValueError('result_raw_tick_disagreement')
    if result.get('source_unchanged') is not True or result.get('reconstruction_validated') is not True:
        raise ValueError('result_integrity_flags')
    if any(result.get(k) is not False for k in ('accepted','whole_episode_success','full_ten_success','dataset_success')):
        raise ValueError('formal_flags_unlocked')
    return completed


def completed_guards(p):
    """Evaluate on every report, including failed raw outcomes."""
    trace=p['trace'];checks={};arm=p['arm'];target=p['object_id']
    exits=[e for e in trace if e['kind']=='motion_phase_exit']
    checks['two_successful_phase_exits']=len(exits)==2 and {e['motion_phase'] for e in exits}=={'retreat','home'} and all(
        e['arm']==arm and e['object_id']==target and e['failed'] is False and e['reason'] is None and
        e.get('actual_velocity_targets')==[0.]*7 and 0<e['completed_ticks']<=36000 and e['completed_hold_ticks']<=5000 for e in exits)
    motions=[e for e in trace if e['kind']=='return_motion_trace']
    checks['two_successful_motion_traces']=len(motions)==2 and all(e['failed'] is False and e['reason'] is None for e in motions)
    settled=[e for e in trace if e['kind']=='return_motion_settled']
    plans={e['motion_phase']:e for e in trace if e['kind']=='return_motion_plan'}
    checks['two_measured_terminal_poses']=len(settled)==2 and {e['motion_phase'] for e in settled}=={'retreat','home'} and all(
        e['arm']==arm and e['object_id']==target and 0<=e['position_error_m']<=.005 and 0<=e['rotation_error_rad']<=.04 and
        max(abs(x) for x in e['measured_qdot'][:7])<=.005 and
        e['motion_phase'] in plans and max(abs(np.asarray(e['measured_q'][:7])-np.asarray(plans[e['motion_phase']]['vertices'][-1])))<=.005 for e in settled)
    feedback=[e for e in trace if e['kind']=='velocity_feedback_receipt']
    checks['two_zero_target_feedback_exits']=len(feedback)==2 and all(e['setting']=='k6' and e['actual_final_targets']==[0.]*7 and
        e['drive_properties_changed'] is False and e['completed_ticks']<=36000 for e in feedback)
    vertices=[e for e in trace if e['kind']=='vertex_governor_vertex']
    checks['vertex_guards']=bool(vertices) and all(e['status'] in ('initial_pass','resumed') and e['reason'] is None and
        0<=e['completed_hold_ticks']<=500 and 0<=e['motion_added_ticks']<=5000 and
        (e['status']=='initial_pass' or e['streak']>=25) for e in vertices)
    checks['no_other_failed_guard']=not any(e.get('failed') is True or e['kind'] in ('home_contact_rejected','home_coarse_rejected','return_motion_rejected','motion_refinement_rejected') for e in trace)
    returned=[e for e in trace if e['kind']=='returned']
    checks['returned_pose_and_velocity']=False
    if len(returned)==1:
        r=returned[0];obj=r['state']['objects'][str(target)];pose=np.asarray(obj['pose']);goal=np.asarray(r['goal_pose'])
        position=float(np.linalg.norm(pose[:3]-goal[:3]));qa=pose[3:]/np.linalg.norm(pose[3:]);qb=goal[3:]/np.linalg.norm(goal[3:])
        angle=float(2*np.arccos(np.clip(abs(qa@qb),-1,1))*180/np.pi)
        checks['returned_pose_and_velocity']=r['object_id']==target and 0<=r['position_error_m']<=.01 and 0<=r['rotation_error_deg']<=10 and abs(position-r['position_error_m'])<1e-9 and abs(angle-r['rotation_error_deg'])<1e-5 and np.linalg.norm(obj['linear_velocity'])<=.03 and np.linalg.norm(obj['angular_velocity'])<=.15
    return {k:bool(v) for k,v in checks.items()}


def progress_checks(p):
    trace=p['trace'];rows=[e for e in trace if e['kind']=='home_progress_receipt'];admit=[e for e in trace if e['kind']=='home_progress_admitted']
    if len(rows)!=1 or len(admit)!=1:return dict(progress_receipt_count=False)
    r=rows[0];a=admit[0];plan=next(e for e in trace if e['kind']=='return_motion_plan' and e['motion_phase']=='home')
    phase=next(e for e in trace if e['kind']=='motion_phase_exit' and e['motion_phase']=='home')
    retained=r['samples']+r['tail'];vertices=np.asarray(plan['vertices']);okay=True
    for row in retained:
        seg=row['segment'];q=np.asarray(row['reference_q']);v=np.asarray(row['nominal_qdot'])
        if not 0<=seg<len(vertices) or not np.isfinite(q).all() or max(abs(v))>.06+1e-12:okay=False;continue
        if seg==len(vertices)-1:okay=okay and np.array_equal(q,vertices[-1])
        else:
            d=vertices[seg+1]-vertices[seg];l=max(abs(d));s=row['segment_position_rad']
            okay=okay and 0<=s<=l+1e-12 and np.max(np.abs(q-(vertices[seg] if l==0 else vertices[seg]+d*s/l)))<=1e-12
        b=row['reference_brake']
        if b is not None:okay=okay and b['tracking_plus_reference_brake_plus_step_m']<=.006 and all(b['link_arc_bound_m'][n]+x<=.006 for n,x in b['one_step_reserve_m'].items())
        okay=okay and row['actual_dt_s']==r['actual_dt_s'] and row['paused_ticks']+row['vertex_hold_ticks']<=5000
    motion=next(e for e in trace if e['kind']=='return_motion_trace' and e['motion_phase']=='home')
    packets=motion['samples']+motion['failure_window'];selected=[x for x in packets if 'home_progress_decision' in x]
    alignment=bool(selected)
    for packet in selected:
        d=packet['home_progress_decision']
        alignment=alignment and packet['command_q']==d['reference_q'] and packet.get('nominal_command_qdot',packet['command_qdot'])==d['nominal_qdot'] and packet['readback']['scene_step_count']==d['scene_step'] and packet['readback']['scene_timestep_s']==d['actual_dt_s'] and packet['command_index']==d['command_index']
    return dict(retained_command_decision_alignment=alignment,progress_receipt_count=True,progress_binding=r['path_sha256']==a['path_sha256']==plan['path_sha256'] and
        r['model_sha256']==a['model_sha256']==plan['certificate']['home_contact_model_sha256'],
        progress_policy=r['policy']==a['policy']==POLICY and r['physical_braking_proved'] is False,
        progress_bounds=0<=r['max_nominal_velocity_rad_s']<=.06+1e-12 and 0<=r['max_nominal_acceleration_rad_s2']<=.4+1e-8 and 0<=r['paused_ticks']<=5000 and r['paused_ticks']+phase['completed_hold_ticks']<=5000,
        retained_progress_decisions=bool(retained) and bool(okay),progress_outcome=r['failed']==phase['failed'] and (p['accepted'] is not True or r['reference_complete'] is True),
        dynamic_time_law_declared=plan['profile'].get('home_progress') is True and plan['profile'].get('dense_hash_semantics')=='nominal comparison profile; executed reference governed')


def compare(a,b,off,on):
    modified=copy.deepcopy(on);flag=modified.pop('home_progress_bound',False)
    checks=dict(one_tracking_intervention=flag is True and modified==off and off.get('home_contact_envelope') is True and off.get('home_progress') is True and not off.get('home_progress_bound',False) and not on.get('reserve_home_workspace',False),
        prefix_before_home_exact=bool(prefix(a['trace'])) and prefix(a['trace'])==prefix(b['trace']),
        independent_unchanged_shadows=all(p['source_unchanged'] is True and p['independent_scene'] is True for p in (a,b)),
        complete_motion_receipts=all(validate_trace(p['trace'],cfg)['receipt_complete'] for p,cfg in ((a,off),(b,on))))
    plans=[next(e for e in p['trace'] if e['kind']=='return_motion_plan' and e['motion_phase']=='home') for p in (a,b)]
    models=[next(e for e in p['trace'] if e['kind']=='home_contact_model') for p in (a,b)]
    checks['same_actual_home_curve']=plans[0]['vertices']==plans[1]['vertices'] and all(digest(p['vertices'])==p['path_sha256'] for p in plans)
    checks['same_live_contact_model']=models[0]['model']==models[1]['model'] and all(digest(m['model'])==m['model_sha256'] and m['model'].get('construction')=='live_public_shape_getters' and not m['model'].get('synthetic_cpu_fixture') for m in models)
    checks['entry_state_controls_rng_exact']=all(plans[0]['planning_start'][k]==plans[1]['planning_start'][k]==models[0][k]==models[1][k] for k in ('state_sha256','controls_sha256','rng_sha256')) and models[0]['state']==models[1]['state'] and all(digest(m['state'])==m['state_sha256'] for m in models)
    checks['entry_scene_steps_exact']=models[0]['scene_step']==models[1]['scene_step']==plans[0]['initial_readback']['scene_step_count']==plans[1]['initial_readback']['scene_step_count']
    certs=[p['certificate'] for p in plans]+[e[k] for p in (a,b) for e in p['trace'] if e['kind']=='home_coarse_prepared' for k in ('original_certificate','fresh_certificate')]
    checks['whole_curve_certificates']=len(certs)==6 and all(c==certs[0] and c['certified'] is True and c['home_contact_model_sha256']==models[0]['model_sha256'] for c in certs)
    checks.update(progress_checks(b))
    if b['object_id']==10:
        obs=[e for e in b['trace'] if e['kind']=='n17_observation']
        checks['n17_observer_complete']=len(obs)==1 and all(observation_checks(obs[0],b).values())
    return checks


def final_envelope(run,preview):
    from home_interaction import ContactEnvelope
    from direct_core import ForwardModel
    from physics_geometry import bounds_for_parts
    from return_world import ReturnWorld
    from return_clearance import ReturnClearance
    from return_reconstruction import load_bundle
    e=next(e for e in preview['trace'] if e['kind']=='home_contact_model')
    returned=next(e for e in preview['trace'] if e['kind']=='returned')
    plan=next(e for e in preview['trace'] if e['kind']=='return_motion_plan' and e['motion_phase']=='home')
    bundle=load_bundle(run/'snapshot/bundle',preview['object_id'])
    geometry=json.loads((run/'snapshot/src/fixtures/collision-boxes.json').read_text())
    vertices=json.loads((run/'snapshot/src/fixtures/robot-vertices.json').read_text())
    for oid,parts in bundle['meshes'].items():geometry['objects'].setdefault(oid,{})['parts']=bounds_for_parts(parts)
    model=ForwardModel(run/'snapshot/src/fixtures/panda.urdf')
    world=ReturnWorld(model,geometry,e['state'],bundle['meshes'],vertices,optimize=True)
    envelope=ContactEnvelope(e['model'],world)
    c=ReturnClearance(envelope.attach(world),e['arm'],e['object_id'])
    return c.monitor(returned['state'],plan['vertices'][-1])


def audit(baseline,proposed):
    runs=[Path(baseline),Path(proposed)];raw=[];checks={};errors=[];inputs={};completed=False;guards={};observed={};rechecked=None
    read=lambda p:json.loads(p.read_text())
    try:
        manifests=[read(r/'diagnostic-manifest.json') for r in runs];configs=[read(r/'config.json') for r in runs];previews=[]
        for r,m in zip(runs,manifests):
            events=[json.loads(x) for x in (r/'artifacts/events.jsonl').read_text().splitlines()];ps=[e for e in events if e['kind']=='return_preview']
            if len(ps)!=1:raise ValueError('single_preview_required')
            p=ps[0];previews.append(p);result=read(r/'artifacts/diagnostic-result.json')
            raw.append(dict(run=str(r),terminal=read(r/'terminal.json'),result=result,
                preview={k:p.get(k) for k in ('arm','object_id','index','trial','accepted','failure','trial_steps','source_unchanged','independent_scene')},
                guard_outcome=[e for e in p['trace'] if e['kind'] in ('motion_phase_exit','return_motion_trace','home_contact_rejected','home_progress_receipt')],returned=[e for e in p['trace'] if e['kind']=='returned']))
            outcome(result,p,m)
            for n in ('diagnostic-manifest.json','config.json','artifacts/events.jsonl','artifacts/diagnostic-result.json','terminal.json','started.json','assignment.json','artifacts/reconstruction.json'):inputs[str(r/n)]=file_hash(r/n)
        completed=outcome(raw[1]['result'],previews[1],manifests[1]);guards=completed_guards(previews[1])
        current=read(ROOT/'harness/configs/v22/BINDINGS.json');old=read(ROOT/'harness/configs/v21/BINDINGS.json');source=source_files(ROOT/'src')
        bound=[];versions=[]
        for i,(m,r) in enumerate(zip(manifests,runs)):
            mapping={n[4:]:h for n,h in m['snapshot_sha256'].items() if n.startswith('src/')}
            binding=current if mapping==current['source_sha256'] else old
            versions.append('v22' if binding is current else 'v21')
            bound.append(mapping==binding['source_sha256'] and m['config_sha256'] in binding['configs'].values() and raw[i]['result']['source_sha256']==mapping and (i==0 or binding is current))
            checks['snapshot_bytes_'+str(i)]=file_hash(r/'config.json')==m['config_sha256'] and all(file_hash(r/'snapshot'/n)==h for n,h in m['snapshot_sha256'].items())
            if m['experience']:
                policy=read(r/'snapshot/experience.json')
                checks['policy_binding_'+str(i)]=file_hash(r/'snapshot/experience.json')==binding['policy_file_sha256'] and policy['implementation_sha256']==digest(mapping) and policy['policy_sha256']==digest({k:v for k,v in policy.items() if k!='policy_sha256'})
        checks['source_config_bindings']=all(bound) and source==current['source_sha256'] and digest(source)==current['implementation_sha256']
        checks['baseline_version_allowlist']=versions in (['v21','v22'],['v22','v22'])
        for k in ('release_sha256','source_fingerprint','target','candidate_index','max_previews','python','gpu','experience','n17_observer','legacy_insertion','fixed_setpoint_diagnostic','continuation_diagnostic'):
            checks['same_'+k]=k in manifests[0] and manifests[0][k]==manifests[1].get(k)
        checks['request']=all((m['target'],m['candidate_index']) in ((10,5),(12,6)) and m['max_previews']==1 and not any(m.get(k) for k in ('legacy_insertion','fixed_setpoint_diagnostic','continuation_diagnostic')) for m in manifests)
        checks['manifest_intervention_flags']=all(m.get('home_contact_envelope') is True and type(m.get('home_progress',False)) is bool and m.get('home_progress',False)==c.get('home_progress',False) and m.get('home_progress_bound',False)==c.get('home_progress_bound',False) for m,c in zip(manifests,configs))
        recons=[read(r/'artifacts/reconstruction.json') for r in runs]
        checks['same_validated_reconstruction']=recons[0]==recons[1] and recons[0].get('validated') is True
        starts=[read(r/'started.json') for r in runs];assignments=[read(r/'assignment.json') for r in runs]
        checks['same_actual_gpu']=bool(assignments[0].get('gpu_uuid')) and assignments[0]['gpu_uuid']==assignments[1].get('gpu_uuid') and all(a['gpu_uuid']==s['gpu']['uuid'] for a,s in zip(assignments,starts))
        checks['terminal']=all(e['terminal'].get('state')=='terminal' and e['terminal'].get('exit_code')==0 for e in raw)
        checks['sequential']=starts[0]['time_unix']+raw[0]['terminal']['elapsed_s']<=starts[1]['time_unix']
        checks.update(compare(*previews,*configs))
        rechecked=replay(runs[1],previews[1]);checks['actual_model_path_replay']=rechecked['certified'] and rechecked['matches_runtime']
        if completed:
            rechecked['final_measured_envelope']=final_envelope(runs[1],previews[1])
            guards['final_home_world_robot_envelope']=rechecked['final_measured_envelope']['safe'] is True
        else:guards['final_home_world_robot_envelope']=False
        observers=[e for e in previews[1]['trace'] if e['kind']=='n17_observation']
        if observers:
            o=observers[0]['observation'];body=next(b for b in o['metadata']['bodies'] if b['label']==dict(name='216_inner0926_s1c0n17',arm=None))
            observed=dict(complete=o['complete'],samples=o['samples'],max_speed_m_s=o['max_speed_m_s'],max_spin_rad_s=o['max_spin_rad_s'],
                any_robot_pair=any(w['trigger']=='first_robot_contact_record' for w in o['windows']),
                stayed_asleep=o['complete'] is True and o['truncated_samples']==0 and body['is_sleeping'].get('value') is True and not any(w['trigger']=='sleep_transition' for w in o['windows']),
                transitions=[{k:w[k] for k in ('trigger','scene_step')} for w in o['windows']])
    except Exception as exc:errors.append(type(exc).__name__+':'+str(exc))
    valid=bool(checks) and all(checks.values()) and not errors
    guardpass=bool(guards) and all(guards.values())
    return dict(schema='home-functional/22',checks=checks,errors=errors,inputs_sha256=inputs,raw=raw,
        valid_functional_receipts=valid,proposed_fullpreview_completed=completed,completed_guard_checks=guards,completed_guard_exits=guardpass,
        functional_gate_passed=valid and completed and guardpass and observed.get('any_robot_pair',False) is False,
        n17_observation=observed,geometry_replay=rechecked,prefix_ignored_fields=sorted(IGNORED_PREFIX_FIELDS),
        intentional_home_timing_changed=True,home_curve_changed=False,formal_credit=False,whole_episode_success=False,
        interpretation='Existing v21 progress baseline or same-source v22 progress baseline; exact actual prefix and certified curve; intentional HOME progress intervention. Raw false stays false; no formal credit.')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('baseline',type=Path);p.add_argument('proposed',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=audit(a.baseline,a.proposed);write_once(a.output,r)
    print(json.dumps({k:r[k] for k in ('checks','errors','valid_functional_receipts','proposed_fullpreview_completed','completed_guard_exits','functional_gate_passed')}))
    raise SystemExit(0 if r['functional_gate_passed'] else 2)
