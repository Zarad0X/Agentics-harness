"""Exact manifest identity and actual ALL-out / EACH-return HOME coverage.

Uses emitted event fields. Missing completion remains a failed gate, including
on a valid failed episode; this never grants physical or production credit.
"""
from collections import Counter
from scene_pipeline import BASE_IDS


def canonical_ids(scene):
    cfg=scene['config'];ids=list(scene['objects']);selected=scene['selected_ids']
    if (type(cfg['count']) is not int or cfg['count'] not in (10,15,20) or len(ids)!=cfg['count']
            or any(not isinstance(i,str) or str(int(i))!=i for i in ids)
            or len(selected)!=len(set(selected)) or set(selected)!=set(ids)):
        raise ValueError('canonical_manifest_ids')
    if cfg['layout']=='original' and set(ids)!=set(map(str,BASE_IDS)):
        raise ValueError('canonical_original_ids')
    names=[o['actor_name'] for o in scene['objects'].values()]
    if len(set(names))!=len(names):raise ValueError('canonical_actor_names')
    return set(ids)


def coverage(scene,events):
    ids=canonical_ids(scene);n=len(ids);checks={}
    def oid(e):
        value=e['object_id']
        if type(value) is not int or str(value) not in ids:raise ValueError('event_object_identity')
        return str(value)
    indexed=list(enumerate(events))
    extracted=[(i,e) for i,e in indexed if e['kind']=='extracted']
    barriers=[(i,e) for i,e in indexed if e['kind']=='extraction_barrier']
    starts=[(i,e) for i,e in indexed if e['kind']=='return_started']
    done=[(i,e) for i,e in indexed if e['kind']=='returned']
    exact=lambda rows:len(rows)==n and Counter(oid(e) for _,e in rows)==Counter(ids)
    checks['all_extracted_ids']=exact(extracted)
    checks['all_return_started_ids']=exact(starts)
    checks['all_returned_ids']=exact(done)
    barrier=barriers[0][0] if len(barriers)==1 else -1
    checks['all_out_barrier']=(len(barriers)==1 and checks['all_extracted_ids']
        and len(barriers[0][1]['selected_ids'])==n
        and all(type(i) is int for i in barriers[0][1]['selected_ids'])
        and set(map(str,barriers[0][1]['selected_ids']))==ids
        and set(barriers[0][1]['state']['objects'])==ids
        and all(i<barrier for i,_ in extracted)
        and not any(e['kind'] in ('return_started','return_release','returned') for e in events[:barrier]))
    checks['canonical_goals']=True
    for e in events:
        rows=[e]+(e.get('trace',[]) if e['kind']=='return_preview' else [])
        for row in rows:
            if row.get('object_id') is not None:oid(row)
            if 'objects' in row.get('state',{}) and set(row['state']['objects'])!=ids:
                raise ValueError('event_state_object_identity')
            if 'goal_pose' in row:
                target=oid(row)
                checks['canonical_goals'] &= row['goal_pose']==scene['objects'][target]['pose']
    # A HOME includes one model, plan, phase entry, measured settle, progress
    # receipt and phase exit for the same object AND arm in the same stage.
    kinds=('home_contact_model','return_motion_plan','motion_phase_entry',
           'return_motion_settled','home_progress_receipt','motion_phase_exit')
    homes=[(i,e) for i,e in indexed if e['kind'] in kinds and e.get('motion_phase')=='home']
    expected=Counter((stage,target,kind) for stage in ('extraction','return') for target in ids for kind in kinds)
    observed=Counter(('extraction' if i<barrier else 'return',oid(e),e['kind']) for i,e in homes)
    checks['exact_2N_HOME_lifecycles']=barrier>=0 and observed==expected
    checks['HOME_identity_order_and_completion']=checks['exact_2N_HOME_lifecycles']
    if checks['exact_2N_HOME_lifecycles']:
        for stage in ('extraction','return'):
            for target in ids:
                rows=[(i,e) for i,e in homes if oid(e)==target and (i<barrier)==(stage=='extraction')]
                by_kind={e['kind']:(i,e) for i,e in rows};arms={e['arm'] for _,e in rows}
                indices=[by_kind[k][0] for k in ('home_contact_model','motion_phase_entry','return_motion_plan',
                                                'return_motion_settled','home_progress_receipt','motion_phase_exit')]
                exit=by_kind['motion_phase_exit'][1];progress=by_kind['home_progress_receipt'][1]
                ok=(len(arms)==1 and arms<={'left','right'} and indices==sorted(indices)
                    and exit['failed'] is False and exit['reason'] is None and exit['completed_ticks']>0
                    and exit['actual_velocity_targets']==[0.]*7
                    and progress['failed'] is False and progress['reference_complete'] is True)
                if len(arms)==1 and arms<={'left','right'}:
                    arm=next(iter(arms));plan=by_kind['return_motion_plan'][1]
                    ok &= plan['vertices'][-1]==scene['robots'][arm]['q'][:7]
                if stage=='extraction':
                    matches=[i for i,e in extracted if oid(e)==target]
                    ok &= len(matches)==1 and indices[-1]<matches[0]<barrier
                else:
                    a=[i for i,e in starts if oid(e)==target];b=[i for i,e in done if oid(e)==target]
                    ok &= len(a)==len(b)==1 and barrier<a[0]<indices[0]<indices[-1]<b[0]
                checks['HOME_identity_order_and_completion'] &= ok
    checks['extraction_preview_pairs']=True
    previous=-1
    for end,ex in extracted:
        target=oid(ex);segment=events[previous+1:end];previous=end
        markers=[(i,e) for i,e in enumerate(segment) if e['kind']=='grasp_candidate_verified']
        ok=len(markers)==1
        if ok:
            at,marker=markers[0];pair=[e for e in segment[:at] if e['kind']=='grasp_preview'][-2:]
            ok=(oid(marker)==target and marker['independent_trials']==2 and len(pair)==2
                and [e['trial'] for e in pair]==[1,2]
                and all(oid(e)==target and e['index']==marker['index'] and e['accepted'] is True
                    and e['source_unchanged'] is True and e['independent_scene'] is True for e in pair))
            chosen=[e for e in segment[:at] if e['kind']=='grasp_preview' and oid(e)==target
                    and e['index']==marker['index'] and e['accepted'] is True]
            ok &= len(chosen)==2
        checks['extraction_preview_pairs'] &= ok
    checks['return_preview_pairs']=True
    previous=barrier
    for at,start in starts:
        segment=events[previous+1:at];previous=at;target=oid(start)
        previews=[e for e in segment if e['kind']=='return_preview'];pair=previews[-2:]
        ok=(len(pair)==2 and [e['trial'] for e in pair]==[1,2]
            and pair[0]['index']==pair[1]['index'] and pair[0]['arm']==pair[1]['arm']
            and all(oid(e)==target and e['accepted'] is True and e['source_unchanged'] is True
                    and e['independent_scene'] is True for e in pair))
        if ok:
            chosen=[e for e in previews if e['object_id']==pair[0]['object_id'] and e['index']==pair[0]['index']
                    and e['arm']==pair[0]['arm'] and e['accepted'] is True]
            ok=len(chosen)==2
        checks['return_preview_pairs'] &= ok
    return dict(checks={k:bool(v) for k,v in checks.items()},expected_ids=sorted(ids),expected_count=n,
                required_actual_HOME_count=2*n,completed=False if not checks else all(checks.values()))
