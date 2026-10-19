"""Independent continuous diagnostic contract, never the same-prefix audit.

Raw completion, stage completion, receipt integrity and formal credit remain
separate. Reuses the independent original-pose verifier in an explicit locked
diagnostic mode; no copied/edited evidence and no stats-only success route.
"""
import argparse
import json
from pathlib import Path
import sys
import copy
import math
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'harness')]
from provenance import file_hash,digest,source_files,write_once
from continuous_v36 import validate_manifest
from continuous_coverage_v24 import canonical_ids,coverage
from home_trace_v18 import validate_trace
from home_functional_v22 import completed_guards
from progress_audit_v23 import progress_checks
from verify_roundtrip import audit as verify_episode
from home_reservation import assess
from home_interaction import ContactEnvelope
from return_world import ReturnWorld
from return_clearance import ReturnClearance
from direct_core import ForwardModel
from physics_geometry import bounds_for_parts


def motion_groups(events):
    """A HOME model is followed by exactly its own plan/phase/guard records."""
    groups=[];start=None
    for i,e in enumerate(events):
        if e['kind']=='home_contact_model':
            if start is not None:groups.append(events[start:i])
            start=i
        if e['kind']=='motion_phase_exit' and e['motion_phase']=='home' and start is not None:
            groups.append(events[start:i+1]);start=None
    if start is not None:groups.append(events[start:])
    return groups


def return_groups(events):
    groups=[];start=None
    for i,e in enumerate(events):
        if e['kind']=='return_started':
            if start is not None:raise ValueError('overlapping_return_started')
            start=i
        if e['kind']=='returned':
            if start is None or e['object_id']!=events[start]['object_id']:raise ValueError('unpaired_actual_return')
            segment=events[start:i+1]
            arms={e['arm'] for e in segment if e['kind']=='motion_phase_exit'}
            if len(arms)!=1:raise ValueError('actual_return_arm_identity')
            groups.append(dict(object_id=e['object_id'],arm=arms.pop(),trace=segment,accepted=True,start_event_index=start))
            start=None
    return groups


def raw_consistency(events,result,final):
    previews=[e for e in events if e['kind'] in ('grasp_preview','return_preview','separation_preview')]
    for e in previews:
        if type(e.get('accepted')) is not bool or type(e.get('trial_steps')) is not int or e['trial_steps']<0:
            raise ValueError('raw_preview_outcome_or_steps')
        if (e['accepted'] and e.get('failure') is not None) or (not e['accepted'] and not isinstance(e.get('failure'),str)):
            raise ValueError('raw_preview_failure_consistency')
        if e.get('source_unchanged') is not True or e.get('independent_scene') is not True:raise ValueError('raw_preview_source_mutation')
        if e['kind']=='return_preview':
            if e.get('arm') not in ('left','right') or type(e.get('index')) is not int or e['index']<0 or type(e.get('trial')) is not int or e['trial'] not in (1,2):
                raise ValueError('raw_return_preview_identity')
            for row in e['trace']:
                if row['kind'] in ('home_contact_model','return_motion_plan','motion_phase_exit','return_motion_settled') and (row['arm']!=e['arm'] or row['object_id']!=e['object_id']):
                    raise ValueError('raw_return_trace_identity')
    from preview_accounting_audit_v35 import audit_lifecycle
    audit_lifecycle(events,result)
    if any(str(e['object_id']) not in final['state']['objects'] for e in events if e['kind'].startswith('preview_accounting_')):raise ValueError('preview_unknown_object')
    if type(result.get('physics_steps')) is not int or result['physics_steps']!=final['physics_step']:raise ValueError('raw_episode_steps')
    steps=[e['physics_step'] for e in events]
    if any(type(x) is not int or x<0 or x>final['physics_step'] for x in steps) or any(b<a for a,b in zip(steps,steps[1:])):
        raise ValueError('raw_step_alignment')
    for e in events:
        if e['kind']=='motion_phase_entry' and e['scene_step']!=e['physics_step']:raise ValueError('phase_entry_actual_step')
        if e['kind']=='motion_phase_exit' and (e['final_scene_step']!=e['physics_step'] or e['completed_ticks']!=e['final_scene_step']-e['initial_scene_step']):
            raise ValueError('phase_exit_actual_step')
    if type(result.get('diagnostic_completed')) is not bool:raise ValueError('raw_episode_completion_type')
    if result['diagnostic_completed'] and (result.get('failure') is not None or any(e['kind']=='stopped' for e in events)):
        raise ValueError('raw_episode_failure_consistency')
    if not result['diagnostic_completed'] and not isinstance(result.get('failure'),str):raise ValueError('raw_episode_missing_failure')
    return True


def classify_terminal(run, manifest, terminal):
    """Validate supervisor identity before reporting an outcome, never credit."""
    if (not isinstance(terminal,dict) or terminal.get('state')!='terminal'
            or terminal.get('run_id')!=manifest['run_id'] or terminal['run_id']!=run.name
            or terminal.get('status')!='DIAGNOSTIC ONLY'
            or type(terminal.get('exit_code')) is not int
            or type(terminal.get('elapsed_s')) not in (int,float)
            or not math.isfinite(terminal['elapsed_s']) or terminal['elapsed_s']<0):
        raise ValueError('continuous_terminal_identity')
    reason=terminal.get('reason');code=terminal['exit_code']
    if (reason not in ('exited','timeout','cancelled','launch_failed')
            or (reason=='timeout' and code!=124)
            or (reason=='cancelled' and code not in (130,143))
            or (reason=='launch_failed' and code!=125)):
        raise ValueError('continuous_terminal_reason_exit')
    classification=reason if reason!='exited' else ('exited_zero' if code==0 else 'process_failure')
    return dict(valid_identity=True,classification=classification,reason=reason,exit_code=code,
                elapsed_s=terminal['elapsed_s'],completion_credit=False)


def audit(run):
    run=Path(run);read=lambda p:json.loads(p.read_text());checks={};errors=[];raw={};guards={};stages={};verification=None
    entry_report=None;admission_report=None;trace_report=None;cfg=None;m=None
    inputs={};replays=[];terminal_report=dict(valid_identity=False,classification='unavailable_or_invalid',completion_credit=False)
    try:
        m=read(run/'diagnostic-manifest.json')
        validate_manifest(run,m);expected_source=source_files(ROOT/'src')
        cfg=read(run/'config.json')
        terminal=read(run/'terminal.json')
        terminal_report=classify_terminal(run,m,terminal);raw['terminal']=terminal
        for name in ('diagnostic-manifest.json','config.json','terminal.json'):
            inputs[str(run/name)]=file_hash(run/name)
        a=run/'artifacts'
        required=[run/n for n in ('started.json','assignment.json')]+[a/n for n in (
            'events.jsonl','result.json','final-state.json','scene-manifest.json',
            'effective-config.json','physics-meshes.json','trajectory.jsonl')]
        missing=[str(p.relative_to(run)) for p in required if not p.is_file()]
        if missing:raise FileNotFoundError('missing_required_artifacts:'+','.join(missing))
        events=[json.loads(x) for x in (a/'events.jsonl').read_text().splitlines()]
        result=read(a/'result.json');final=read(a/'final-state.json');scene=read(a/'scene-manifest.json')
        started=read(run/'started.json');assignment=read(run/'assignment.json')
        raw.update(result=result,terminal=terminal,stops=[e for e in events if e['kind']=='stopped'],
            actual_phase_exits=[e for e in events if e['kind']=='motion_phase_exit'],
            return_previews=[{k:e.get(k) for k in ('arm','object_id','index','trial','accepted','failure','trial_steps')} for e in events if e['kind']=='return_preview'])
        from placement_audit_v30 import audit_admissions
        admission_report=audit_admissions(run,m,events,scene)
        checks['placement_endpoint_admission']=admission_report['valid']
        from home_entry_audit_v33 import audit_entries
        checks['placement_HOME_entry_witness']=False
        entry_report=dict(valid=False,physical_completion_proved=False)
        entry_report=audit_entries(run,m,events,scene)
        checks['placement_HOME_entry_witness']=entry_report['valid']
        checks['source_snapshot']=m['source_sha256']==expected_source and all(file_hash(run/'snapshot'/n)==h for n,h in m['snapshot_sha256'].items())
        checks['scene_source_config']=scene['source_sha256']==m['source_sha256'] and cfg==scene['config']==read(a/'effective-config.json')
        checks['raw_consistency']=raw_consistency(events,result,final)
        checks['locked']=all(result.get(k) is False for k in ('accepted','whole_episode_success','full_ten_success','dataset_success','formal_credit','reconstructed')) and result.get('initialized_episode') is True
        checks['retained_physics_labels']=(result.get('magnetic_drive_limit_raw')==80 and result.get('finger_friction')==[3.,2.4]
            and result.get('release_velocity_reset') is True and result.get('forbidden_legacy_calls')==[])
        checks['terminal']=terminal.get('state')=='terminal' and type(terminal.get('exit_code')) is int and terminal['exit_code']==(0 if result['diagnostic_completed'] else 2)
        if m['execution']['mode']=='headless':
            from headless_audit_v36 import receipts
            checks['assigned_headless_cpu']=receipts(run,m,assignment,started,result)
        else:
            checks['assigned_gpu']=m['gpu']==assignment['gpu'] and assignment['gpu_uuid']==started['gpu']['uuid']==result['gpu_at_start']['uuid'] and assignment['run_id']==m['run_id']==run.name
        checks['no_reconstruction']=(not (a/'reconstruction.json').exists() and not (run/'snapshot/bundle').exists())
        canonical=canonical_ids(scene);count=len(canonical)
        identity_coverage=coverage(scene,events)
        stages.update(identity_coverage['checks'])
        models=[e for e in events if e['kind']=='home_workspace_model']
        if len(models)!=1:raise ValueError('workspace_model_count')
        wm=models[0];meshes=read(a/'physics-meshes.json');geometry=read(run/'snapshot/src/fixtures/collision-boxes.json')
        vertices=read(run/'snapshot/src/fixtures/robot-vertices.json');model=ForwardModel(run/'snapshot/src/fixtures/panda.urdf')
        for oid,p in meshes.items():geometry['objects'].setdefault(oid,{})['parts']=bounds_for_parts(p)
        def world(s):return ReturnWorld(model,copy.deepcopy(geometry),s,meshes,copy.deepcopy(vertices),optimize=True)
        from closure_audit import verify as verify_closure
        checks['closure_control']=verify_closure(events)['valid']
        from capture_audit import verify as verify_capture
        checks['capture_feedback']=verify_capture(events,world)['valid']
        from supported_handoff_audit import verify as verify_supported
        checks['supported_grasp_handoff']=verify_supported(events,world)['valid']
        from actuator_audit import verify as verify_actuators
        checks['actuator_representation']=verify_actuators(events,world,m['loaded_variant'])['valid']
        from placement_retreat_audit import verify as verify_placement_retreat
        checks['placement_unloaded_retreat']=verify_placement_retreat(events,world)['valid']
        from extraction_handoff_audit import verify as verify_handoff
        checks['extraction_handoff_binding']=verify_handoff(events,world)['valid']
        checks['canonical_home_reservation']=wm['homes']==final['homes']=={a:scene['robots'][a]['q'][:7] for a in ('left','right')}
        checks['live_model']=digest(wm['model'])==wm['model_sha256'] and wm['model']['construction']=='live_public_shape_getters'
        reserves=[e for e in events if e['kind']=='home_workspace_reservation']
        matches=[]
        for e in reserves:
            envelope=ContactEnvelope(wm['model'],world(e['state']))
            replay=assess(world,e['state'],wm['homes'],envelope,e['object_id'],e['predicted_transform'])
            matches.append(all(e.get(k)==v for k,v in replay.items()))
        checks['reservation_replay']=bool(matches) and all(matches)
        for group in motion_groups(events):
            e=group[0];plans=[x for x in group if x['kind']=='return_motion_plan' and x['motion_phase']=='home']
            if not plans:continue  # Preserve an actual preflight rejection.
            if len(plans)!=1:raise ValueError('HOME_plan_count')
            plan=plans[0];w=world(e['state']);ContactEnvelope(e['model'],w).attach(w)
            c=ReturnClearance(w,e['arm'],e['object_id']);cert=c.certify(plan['vertices'])
            replays.append(dict(arm=e['arm'],object_id=e['object_id'],scene_step=e['scene_step'],certificate=cert,
                exact=cert==plan['certificate'],path_sha256=digest(plan['vertices'])))
            if e['model_sha256']!=wm['model_sha256'] or digest(e['model'])!=e['model_sha256'] or digest(e['state'])!=e['state_sha256']:
                raise ValueError('HOME_live_model_state_binding')
            if digest(plan['vertices'])!=plan['path_sha256']:raise ValueError('HOME_path_binding')
            from home_entry import assessment
            admissions=[x for x in group if x['kind']=='home_entry_admission']
            readies=[x for x in group if x['kind']=='home_entry_ready']
            if not cfg.get('home_entry_hold',False) and (admissions or readies):raise ValueError('undeclared_HOME_entry_hold')
            if len(admissions)>1 or len(readies)>1:raise ValueError('HOME_entry_receipt_count')
            for x in admissions+readies:
                if (x['arm']!=plan['arm'] or x['object_id']!=plan['object_id'] or x['reference_q']!=plan['vertices'][0]
                        or x['assessment']!=assessment(c,x['reference_q'],x['state'])
                        or x['scene_step']!=x['physics_step']):raise ValueError('HOME_entry_replay')
            if readies:
                if not admissions or not admissions[0]['assessment']['required'] or readies[0]['assessment']['required']:
                    raise ValueError('HOME_entry_outcome')
                holds=[x for x in group if x['kind']=='vertex_governor_vertex' and x['initial_scene_step']==admissions[0]['scene_step']]
                if (len(holds)!=1 or holds[0]['status']!='resumed' or holds[0]['reference_q']!=plan['vertices'][0]
                        or readies[0]['scene_step']-admissions[0]['scene_step']!=holds[0]['completed_hold_ticks']):
                    raise ValueError('HOME_entry_hold_alignment')
            phase=next((x for x in group if x['kind']=='motion_phase_exit'),None)
            if phase is not None and phase['failed'] is False and cfg.get('home_entry_hold',False):
                if len(admissions)!=1 or (admissions[0]['assessment']['required'] and len(readies)!=1):
                    raise ValueError('HOME_completed_without_entry')
            if phase is not None:
                pc=progress_checks(dict(trace=group,accepted=not phase['failed']))
                if not all(pc.values()):raise ValueError('HOME_progress_receipts:'+str(pc))
        checks['executed_HOME_curve_replay']=all(x['exact'] and x['certificate']['certified'] for x in replays)
        checks['actual_motion_receipts']=validate_trace(events,cfg)['receipt_complete']
        for e in events:
            if e['kind']=='return_preview':
                if not validate_trace(e['trace'],cfg)['receipt_complete']:raise ValueError('preview_motion_receipts')
                if e['accepted'] and not all(completed_guards(e).values()):raise ValueError('preview_false_guard_completion')
        extracted=[e for e in events if e['kind']=='extracted'];ids=[e['object_id'] for e in extracted]
        admitted=True
        for ex in extracted:
            target=ex['object_id'];before=[e for e in events if e['physics_step']<=ex['physics_step'] and e.get('object_id')==target]
            admitted=admitted and all(any(e['kind']=='home_workspace_reservation' and e['stage']==stage and e['safe'] is True for e in before)
                for stage in ('predicted_slot','measured_after_retreat','measured_after_home'))
            exits=[e for e in before if e['kind']=='motion_phase_exit' and e['motion_phase']=='home']
            admitted=admitted and len(exits)==1 and exits[0]['failed'] is False and exits[0]['actual_velocity_targets']==[0.]*7
        guards['extracted_workspace_and_HOME']=bool(extracted) and bool(admitted)
        groups=return_groups(events)
        for group in groups:
            prior=[e for e in events[:group['start_event_index']] if e['kind']=='return_preview'][-2:]
            if (len(prior)!=2 or [e['trial'] for e in prior]!=[1,2]
                    or prior[0]['index']!=prior[1]['index'] or any(e['accepted'] is not True or
                    e['arm']!=group['arm'] or e['object_id']!=group['object_id'] for e in prior)):
                raise ValueError('committed_return_preview_arm_index_trial_binding')
        guardrows={str(p['object_id']):completed_guards(p) for p in groups}
        guards['all_actual_return_guards']=len(groups)==count and set(guardrows)==canonical and all(all(x.values()) for x in guardrows.values())
        guards['all_HOME_progress_complete']=len(replays)==2*count and stages['exact_2N_HOME_lifecycles'] and stages['HOME_identity_order_and_completion'] and all(e['reference_complete'] is True and e['failed'] is False for e in events if e['kind']=='home_progress_receipt')
        guards['final_both_HOME']=all(max(abs(np.array(final['state']['robots'][a]['q'][:7])-wm['homes'][a]))<=.005 and max(abs(np.array(final['state']['robots'][a]['qdot'][:7])))<=.005 for a in ('left','right'))
        verification=verify_episode(a,run/'snapshot/src',continuous_diagnostic=True)
        guards['independent_original_pose_verifier']=verification.get('continuous_diagnostic_passed') is True
        for p in [run/n for n in ('diagnostic-manifest.json','config.json','terminal.json','started.json','assignment.json')]+list(a.glob('*.json'))+[a/'events.jsonl',a/'trajectory.jsonl']:
            inputs[str(p)]=file_hash(p)
    except Exception as exc:errors.append(type(exc).__name__+':'+str(exc))
    if cfg is not None:
        from placement_trace_audit_v31 import inspect as inspect_placement_trace
        trace_report=inspect_placement_trace(run,m,cfg)
        checks['requested_placement_trace']=trace_report['valid']
        errors.extend(trace_report['errors'])
    from sleep_copy_audit_v35 import inspect as inspect_copy
    copy_report=inspect_copy(run,m,cfg)
    checks['requested_sleep_copy_trace']=copy_report['valid']
    errors.extend(copy_report['errors'])
    from live_sleep_audit import inspect as inspect_live_sleep
    live_report=inspect_live_sleep(run,m,cfg)
    checks['live_sleep_witness']=live_report['valid']
    guards['live_sleep_boundary_qualified']=live_report['qualified_boundaries']>0
    errors.extend(live_report['errors'])
    from loaded_audit import inspect as inspect_loaded
    loaded_report=inspect_loaded(run,m,cfg)
    checks['loaded_motion_witness']=loaded_report['valid']
    guards['all_extraction_loaded_motions_witnessed']=loaded_report['all_extraction_loaded_motions_witnessed']
    errors.extend(loaded_report['errors'])
    exits=[e for e in events if e['kind']=='extraction_release_transition' and e['stage']=='loaded_exit'] if 'events' in locals() else []
    checks['loaded_release_continuity']=all(len([r for r in loaded_report['motions'] if r['target']==e['object_id'] and r['phase']=='lowering' and r['completed']
        and r['terminal_step']==e['physics_step'] and r['state_sha256']==e['state_sha256'] and r['controls_sha256']==e['controls_sha256']])==1 for e in exits)
    valid=bool(checks) and all(checks.values()) and not errors
    raw_completed=raw.get('result',{}).get('diagnostic_completed') is True
    passed=valid and raw_completed and bool(guards) and all(guards.values()) and bool(stages) and all(stages.values())
    from home_cpu_audit_v36 import inspect as inspect_profile
    profiler_sidecar=inspect_profile(run)
    from runtime_profile_audit import inspect as inspect_runtime_profile
    runtime_measurement=inspect_runtime_profile(run)
    return dict(runtime_measurement=runtime_measurement,runtime_variant='v36-live-combined-024',loaded_motion_witness=loaded_report,live_sleep_witness=live_report,visual_deliverable_complete=False,offline_render_pending=True,sleep_copy_witness=copy_report,placement_HOME_entry_witness=entry_report,placement_endpoint_admissions=admission_report,placement_trace_witness=trace_report,terminal_outcome=terminal_report,profiler_sidecar=profiler_sidecar,schema='continuous-functional/36',checks=checks,errors=errors,inputs_sha256=inputs,raw=raw,
        valid_functional_receipts=valid,raw_continuous_completed=raw_completed,stage_checks=stages,completed_guard_checks=guards,
        completed_guard_exits=bool(guards) and all(guards.values()),functional_gate_passed=passed,
        independent_verification=verification,HOME_replays=replays,formal_credit=False,whole_episode_success=False,
        contract='Fresh continuous initial scene, changed temporary slots and extraction HOME prefix. Exact canonical IDs and 2N actual HOME lifecycles. No reconstruction credit or production promotion.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    r=audit(a.run);write_once(a.output,r)
    print(json.dumps({k:r[k] for k in ('terminal_outcome','placement_trace_witness','checks','errors','valid_functional_receipts','raw_continuous_completed','stage_checks','completed_guard_exits','functional_gate_passed')}))
    raise SystemExit(0 if r['functional_gate_passed'] else 2)
