"""Required unloaded-motion receipts and compact evidence; NOT acceptance audit.

A declared coarse-HOME config requires the lifecycle even on preflight failure.
Old v17 can be explicitly labelled legacy-incomplete, never silently repaired.
"""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from provenance import digest,file_hash,write_once
from motion_refinement import profile,CAPS

LEGACY_V17_HOME_SHA = '31c56aaf916bca05f7f40c59e890e4a4f7702918633375f92cb7d375b3ab2d90'
KEYS=('arm','object_id','motion_phase')
PHASE_KINDS={'home_coarse_prepared','home_coarse_rejected','motion_refinement_prepared',
    'motion_refinement_rejected','motion_refinement_execution','return_motion_plan',
    'return_motion_trace','return_motion_settled','velocity_feedback_receipt','vertex_governor_vertex'}


def validate_trace(trace, config):
    errors=[];groups=[];active=None;required=config.get('home_coarse',False)
    def fail(message):errors.append(message)
    for index,e in enumerate(trace):
        kind=e['kind']
        if kind=='motion_phase_entry':
            if active is not None:fail('entry_without_previous_exit:'+str(index))
            active=[e];groups.append(active)
        elif kind=='motion_phase_exit':
            if active is None:fail('exit_without_entry:'+str(index))
            else:active.append(e);active=None
        elif kind in PHASE_KINDS:
            if active is not None:active.append(e)
            elif required:fail('missing_phase_entry:'+kind+':'+str(index))
    if active is not None:fail('missing_phase_exit')
    speed=config.get('home_nominal_speed_rad_s',.1)
    summaries=[]
    for number,group in enumerate(groups):
        start=group[0];key=tuple(start.get(k) for k in KEYS)
        def check(ok,name):
            if not ok:fail(f'phase{number}:{name}')
        check(start.get('schema')=='motion-phase/18','entry_schema')
        for e in group:check(tuple(e.get(k) for k in KEYS)==key,'phase_identity:'+e['kind'])
        endings=[e for e in group if e['kind']=='motion_phase_exit'];plans=[e for e in group if e['kind']=='return_motion_plan']
        nodes=[e for e in group if e['kind']=='vertex_governor_vertex']
        motions=[e for e in group if e['kind']=='return_motion_trace'];feedback=[e for e in group if e['kind']=='velocity_feedback_receipt']
        prepared=[e for e in group if e['kind']=='home_coarse_prepared'];rejected=[e for e in group if e['kind']=='home_coarse_rejected']
        check(len(endings)==1,'exit_count');end=endings[0] if endings else {}
        check(end.get('schema')=='motion-phase/18','exit_schema')
        check(type(start.get('scene_step')) is int and type(end.get('final_scene_step')) is int,'clock_types')
        check(end.get('initial_scene_step')==start.get('scene_step'),'initial_clock')
        ticks=end.get('completed_ticks')
        check(type(ticks) is int and 0<=ticks<=CAPS['total_execution_ticks'],'execution_budget')
        if type(end.get('final_scene_step')) is int and type(start.get('scene_step')) is int:
            check(ticks==end['final_scene_step']-start['scene_step'],'clock_accounting')
        check(end.get('checkpoint_count')==len(nodes),'checkpoint_count')
        check(end.get('completed_hold_ticks')==sum(e['completed_hold_ticks'] for e in nodes),'hold_accounting')
        checkpoint_keys=('vertex','reference_index','resume_index','status','reason','completed_hold_ticks','motion_added_ticks','streak')
        check(end.get('checkpoints')==[{k:e[k] for k in checkpoint_keys} for e in nodes],'checkpoint_rows')
        check(len(plans)<=1,'plan_count')
        if plans:
            plan=plans[0];plan_keys=('path_sha256','certificate_sha256','dense_command_sha256','dense_commands')
            check(end.get('plan')=={k:plan[k] for k in plan_keys},'exit_plan_binding')
            check(plan['certificate'].get('certified') is True and digest(plan['certificate'])==plan['certificate_sha256'],'plan_certificate')
            check(len(motions)==1,'motion_trace_count')
            if motions:check(motions[0]['failed']==end.get('failed'),'motion_outcome')
            if config.get('velocity_feedback','off')!='off':
                check(len(feedback)==1,'feedback_receipt_count')
                if feedback:check(feedback[0]['completed_ticks']==ticks,'feedback_tick_accounting')
        else:
            check(end.get('failed') is True and bool(end.get('reason')) and ticks==0,'unplanned_failure')
            check(end.get('plan') is None,'unplanned_exit_plan')
        if start.get('motion_phase')=='home':
            check(len(prepared)+len(rejected)==1,'home_preflight_receipt_count')
            if prepared:
                e=prepared[0]
                try:
                    _,_,original=profile(e['original_vertices']);_,_,executed=profile(e['original_vertices'],speed)
                    check(e['schema']=='home-coarse/18','home_schema')
                    dynamic=config.get('home_progress',False)
                    check(e['original_profile']==original and (e.get('nominal_comparison_profile')==executed and e.get('dynamic_progress') is True and e['executed_profile'] is None and e.get('executed_clock')==('home-progress/22' if config.get('home_progress_bound',False) else 'home-progress/21') if dynamic else e['executed_profile']==executed),'original_and_executed_clocks')
                    check(e['nominal_speed_limit_rad_s']==speed and e['clocks_changed']==(speed!=.1),'declared_time_law')
                    check(e['original_path_sha256']==e['executed_path_sha256']==digest(e['original_vertices']),'curve_binding')
                    check(e['caps']==CAPS,'caps')
                    for name in ('original_certificate','fresh_certificate'):
                        check(e[name].get('certified') is True and digest(e[name])==e[name+'_sha256'],name)
                    if plans:
                        check(plans[0]['vertices']==e['original_vertices'],'plan_curve')
                        check(plans[0]['dense_command_sha256']==executed['command_sha256'] and plans[0]['dense_commands']==executed['dense_commands_including_initial'],'plan_clock')
                        check(plans[0]['certificate_sha256']==e['fresh_certificate_sha256'],'plan_fresh_certificate')
                        check(plans[0]['profile']['max_velocity']==speed,'plan_nominal_limit')
                except (KeyError,ValueError,TypeError,IndexError) as exc:fail(f'phase{number}:malformed_prepared:{exc}')
            if rejected:
                check(not plans and end.get('failed') is True and bool(rejected[0].get('reason')) and rejected[0].get('no_fallback') is True,'rejected_preflight')
        if end.get('failed') is False:
            check(len([e for e in group if e['kind']=='return_motion_settled'])==1,'settled_receipt')
            check(end.get('actual_velocity_targets')==[0.]*7,'success_zero_targets')
        failure=(motions[0].get('failure_window') or [None])[-1] if motions else None
        summaries.append(dict(identity=dict(zip(KEYS,key)),entry=start,exit=end,coarse_prepared=prepared,rejected=rejected,
            plans=[{k:e[k] for k in ('vertices','path_sha256','dense_command_sha256','dense_commands','certificate','profile')} for e in plans],
            checkpoints=[{k:e[k] for k in checkpoint_keys} for e in nodes],
            outcome=[{k:e[k] for k in ('failed','reason','measured_trace_sha256')} for e in motions],
            feedback=[{k:e[k] for k in ('completed_ticks','saturation_count','slew_count','actual_final_targets','all_numeric_sha256')} for e in feedback],
            settled=[e for e in group if e['kind']=='return_motion_settled'],last_failure_packet=failure))
    # With no unloaded motion attempted, the original loaded failure is still evidence.
    status='invalid' if errors else ('complete' if groups else 'not_reached')
    return dict(status=status,required=required,errors=errors,phase_count=len(groups),phases=summaries,
                receipt_complete=not errors,formal_credit=False)


def summarize(run, legacy_v17=False):
    run=Path(run);cfg=json.loads((run/'config.json').read_text());m=json.loads((run/'diagnostic-manifest.json').read_text())
    path=run/'artifacts/events.jsonl';previews=[]
    legacy=m.get('snapshot_sha256',{}).get('src/home_profile.py')==LEGACY_V17_HOME_SHA
    if legacy_v17 and not legacy:raise ValueError('legacy_v17_source_not_bound')
    with path.open() as stream:
        for line in stream:
            e=json.loads(line)
            if e.get('kind')!='return_preview':continue
            validation=validate_trace(e.get('trace',[]),cfg)
            if cfg.get('home_coarse') and e.get('accepted') is True and not {'retreat','home'} <= {p['identity']['motion_phase'] for p in validation['phases']}:
                validation['errors'].append('accepted_preview_missing_unloaded_phases');validation['status']='invalid';validation['receipt_complete']=False
            if legacy_v17 and validation['errors']:validation['status']='legacy_incomplete'
            previews.append(dict(raw={k:e.get(k) for k in ('trial','object_id','index','accepted','failure','source_unchanged','independent_scene','trial_steps','trial_elapsed_s')},
                returned=[{k:r.get(k) for k in ('object_id','position_error_m','rotation_error_deg','validation_pending','accepted')} for r in e.get('trace',[]) if r['kind']=='returned'],
                receipt_validation=validation))

    bound=file_hash(run/'config.json')==m.get('config_sha256')
    complete=bound and bool(previews) and all(p['receipt_validation']['receipt_complete'] for p in previews)
    return dict(schema='home-trace/18',run=str(run),events_sha256=file_hash(path),config_sha256=file_hash(run/'config.json'),
        declared_config_bound=bound,legacy_v17_source=legacy,
        previews=previews,receipt_complete=complete,formal_credit=False,acceptance_audit=False,whole_episode_success=False)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('runs',type=Path,nargs='+');ap.add_argument('--output',type=Path,required=True);ap.add_argument('--legacy-v17',action='store_true');a=ap.parse_args()
    try:r=dict(schema='home-traces/18',runs=[summarize(p,a.legacy_v17) for p in a.runs],formal_credit=False)
    except Exception as exc:r=dict(schema='home-traces/18',error=type(exc).__name__+':'+str(exc),runs=[],formal_credit=False)
    write_once(a.output,r);ok=bool(r['runs']) and all(x['receipt_complete'] and x['declared_config_bound'] for x in r['runs'])
    print(json.dumps(dict(output=str(a.output),required_receipts_complete=ok,formal_credit=False)))
    sys.exit(0 if ok else 2)
