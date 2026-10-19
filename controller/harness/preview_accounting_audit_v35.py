"""Independent v35 lifecycle evidence gate; no native imports or completion credit."""
import json

def validate_accounting(events,total_steps):
    """Proposed NEW-schema gate. Never retrofit/waive an old run's audit."""
    if type(total_steps) is not int or total_steps<0:raise ValueError('preview_total')
    stack=[];seen={};last_id=0;exclusive=0;failed=0;last_end=0
    for e in events:
        if e['kind']=='preview_accounting_started':
            if e.get('accounting_schema')!='preview-lifecycle/35':raise ValueError('preview_schema')
            if type(e.get('object_id')) is not int or e['object_id']<=0:raise ValueError('preview_object_id')
            meta=e.get('metadata')
            if not isinstance(meta,dict):raise ValueError('preview_metadata')
            if e.get('preview_kind') in ('grasp','return'):
                if set(meta)!=set(('index','trial')) or type(meta['index']) is not int or meta['index']<0 or type(meta['trial']) is not int or meta['trial'] not in (1,2):raise ValueError('preview_trial_metadata')
            elif (set(meta)!=set(('profile',)) and not (set(meta)=={'profile','initialization','forecast_contract'} and meta['initialization'] in ('sleep_zero','awake_raw') and meta['forecast_contract']=='separation-sleep-hypotheses/1')) or meta['profile'] not in [dict(preserve_bias=b,acceleration=a) for a in (.4,1.2,.1) for b in (False,True)] or type(meta['profile'].get('preserve_bias')) is not bool or type(meta['profile'].get('acceleration')) is not float:raise ValueError('preview_profile_metadata')
            i=e['preview_id'];begin=e['preview_steps_begin']
            if type(i) is not int or i!=last_id+1 or type(begin) is not int or begin<last_end:raise ValueError('preview_start_sequence')
            if e['parent_preview_id'] is not None and (type(e['parent_preview_id']) is not int or e['parent_preview_id']<=0):raise ValueError('preview_parent_type')
            if e['parent_preview_id']!=(stack[-1]['preview_id'] if stack else None):raise ValueError('preview_parent')
            if e['preview_kind'] not in ('grasp','separation','return') or e['arm'] not in ('left','right'):raise ValueError('preview_identity')
            if not stack and begin!=last_end:raise ValueError('preview_unaccounted_gap')
            if stack and (e['preview_kind']!='separation' or stack[-1]['preview_kind'] not in ('grasp','return') or e['arm']!=stack[-1]['arm'] or e['object_id']!=stack[-1]['object_id']):raise ValueError('preview_nested_identity')
            row=dict(e,children=0);stack.append(row);seen[i]=row;last_id=i;last_end=begin
        elif e['kind']=='preview_accounting_terminal':
            if not stack:raise ValueError('preview_terminal_without_start')
            start=stack.pop()
            for key in ('accounting_schema','preview_id','parent_preview_id','preview_kind','arm','object_id','metadata','preview_steps_begin'):
                if json.dumps(e[key],sort_keys=True,allow_nan=False)!=json.dumps(start[key],sort_keys=True,allow_nan=False):raise ValueError('preview_terminal_identity')
            end=e['preview_steps_end'];inc=e['inclusive_steps'];exc=e['exclusive_steps']
            if any(type(v) is not int for v in (end,inc,exc)) or end<last_end or inc!=end-start['preview_steps_begin'] or exc!=inc-start['children'] or exc<0:raise ValueError('preview_step_count')
            if e['started_record_emitted'] is not True or type(e['accepted']) is not bool:raise ValueError('preview_terminal_state')
            if e['status']=='failed':
                if e['accepted'] or not isinstance(e['error'],str) or not e['error']:raise ValueError('preview_failed_credit')
                failed+=1
            elif e['status']!='completed' or e['error'] is not None:raise ValueError('preview_terminal_state')
            if stack:stack[-1]['children']+=inc
            exclusive+=exc;last_end=end
    if stack or exclusive!=total_steps or last_end!=total_steps:raise ValueError('preview_incomplete_or_total')
    return dict(terminal_count=len(seen),exclusive_steps=exclusive,failed_count=failed,accounting_valid=True,physical_success=False)


def audit_lifecycle(events,result):
    if result.get('preview_accounting_schema')!='preview-lifecycle/35':raise ValueError('preview_result_schema')
    report=validate_accounting(events,result.get('physics_preview_steps'))
    if type(result.get('preview_lifecycle_attempts')) is not int or result['preview_lifecycle_attempts']!=report['terminal_count']:raise ValueError('preview_attempt_total')
    if result.get('diagnostic_completed') is True and report['failed_count']:raise ValueError('failed_preview_completion_credit')
    # Independently retain ordinary rejected previews and their original receipts.
    terminals=[e for e in events if e['kind']=='preview_accounting_terminal']
    legacy=[]
    for e in events:
        if e['kind'] in ('grasp_preview','return_preview','separation_preview'):
            legacy.extend(x for x in e.get('separation_trials',e.get('trace',[])) if x['kind']=='separation_preview')
            legacy.append(e)
    remaining=list(terminals)
    for e in legacy:
        kind=e['kind'].replace('_preview','')
        meta=dict(profile=e['profile']) if kind=='separation' else dict(index=e['index'],trial=e['trial'])
        if kind=='separation' and 'initialization' in e:
            meta.update(initialization=e['initialization'],forecast_contract=e['forecast_contract'])
        matches=[t for t in remaining if t['preview_kind']==kind and t['object_id']==e['object_id'] and t['metadata']==meta]
        if not matches:raise ValueError('legacy_preview_without_lifecycle')
        t=matches[0];remaining.remove(t)
        if t['accepted'] is not e['accepted'] or t['inclusive_steps']!=e['trial_steps']:raise ValueError('legacy_preview_outcome_steps')
    if any(t['status']=='completed' for t in remaining):raise ValueError('completed_preview_missing_legacy_receipt')
    return report
