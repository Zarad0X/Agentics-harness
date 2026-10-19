"""Finite paired-trial outcome grammar. A witnessed rejection grants no execution."""
import math
PROFILES=[dict(preserve_bias=b,acceleration=a) for a in (.4,1.2,.1) for b in (False,True)]
MODES=['sleep_zero','awake_raw']
def classify(events,path_hash,target):
    groups=[];trials=[];initial=None;ids=set();accepted=False
    for e in events:
        if accepted:raise ValueError('live_events_after_qualification')
        kind=e['kind']
        if kind=='forecast_initialization':
            if initial is not None:raise ValueError('live_initialization_without_trial')
            initial=e
        elif kind=='separation_preview':
            mode=MODES[len(trials)] if len(trials)<2 else None
            if (initial is None or initial['mode']!=mode or e['initialization']!=mode or
                e['path_sha256']!=path_hash or e['object_id']!=target or
                len(groups)>=len(PROFILES) or e['profile']!=PROFILES[len(groups)]):raise ValueError('live_trial_order_binding')
            i=e.get('preview_id')
            if type(i) is not int or i<=0 or i in ids:raise ValueError('live_trial_lifecycle_identity')
            ids.add(i)
            if type(e['accepted']) is not bool or type(e['trial_steps']) is not int or e['trial_steps']<0:raise ValueError('live_trial_outcome_type')
            if e['accepted']:
                m=e['measurement']
                if e['trial_steps']<1 or e['failure'] is not None or e.get('failure_category') is not None or m['clear'] is not True or not math.isfinite(m['position_error_m']) or not math.isfinite(m['rotation_error_rad']) or m['position_error_m']>.005 or m['rotation_error_rad']>.04:raise ValueError('live_trial_physical_failure')
            elif e.get('failure_category')!='candidate_infeasible' or not isinstance(e['failure'],str) or not e['failure'] or e['measurement'] is not None:
                raise ValueError('live_rejection_not_physical')
            if trials and not trials[-1]['accepted']:raise ValueError('live_trial_after_rejection')
            if e['source_unchanged'] is not True or e['independent_scene'] is not True:raise ValueError('live_trial_isolation')
            trials.append(e);initial=None
        elif kind=='separation_profile_bundle':
            if initial is not None or not trials or len(groups)>=len(PROFILES):raise ValueError('live_bundle_missing_trial')
            expected=len(trials)==2 and all(t['accepted'] for t in trials)
            if (e['object_id']!=target or e['path_sha256']!=path_hash or e['profile']!=PROFILES[len(groups)] or
                e['required_initializations']!=MODES or e['outcomes']!=[dict(initialization=t['initialization'],accepted=t['accepted']) for t in trials] or
                type(e['accepted']) is not bool or e['accepted'] is not expected or
                (not expected and trials[-1]['accepted'])):raise ValueError('live_bundle_outcome')
            groups.append(e);trials=[];accepted=expected
        else:raise ValueError('live_unknown_trial_event')
    if initial is not None or trials or not groups or (not accepted and len(groups)!=len(PROFILES)):
        raise ValueError('live_incomplete_profile_search')
    return dict(qualified=accepted,rejected=not accepted,profiles=len(groups),trial_ids=sorted(ids))
