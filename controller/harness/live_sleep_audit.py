"""Independent live-forecast witness audit; no physical acceptance substitutions."""
import base64,gzip,hashlib,io,json
from pathlib import Path
from provenance import digest,file_hash
from placement_audit_v30 import expected_binding
from forecast_report_io_v36 import RAW_LIMIT,GZIP_LIMIT
from audit_forecast_pair_v36 import receipt
from live_sleep_readback import validate_identity,validate_controls
from live_sleep_outcome import classify


def unpack(path,binding):
    if path.stat().st_size!=binding['compressed_size'] or path.stat().st_size>GZIP_LIMIT:raise ValueError('live_gzip_size')
    data=path.read_bytes()
    if hashlib.sha256(data).hexdigest()!=binding['compressed_sha256']:raise ValueError('live_gzip_hash')
    size=binding['uncompressed_size']
    if type(size) is not int or not 0<=size<=RAW_LIMIT:raise ValueError('live_raw_budget')
    with gzip.GzipFile(fileobj=io.BytesIO(data)) as stream:raw=stream.read(size+1)
    if len(raw)!=size or hashlib.sha256(raw).hexdigest()!=binding['uncompressed_sha256']:raise ValueError('live_raw_binding')
    return raw


def validate_report(row):
    if row['schema']!='live-sleep-qualification/2':raise ValueError('live_schema')
    if row['contact_history_equivalent'] is not False or row['conservative_bound_proved'] is not False:raise ValueError('live_false_equivalence')
    validate_identity(row['native_system_identity'],row['source_system'],row['destination_system'])
    if not row['destination_reuse'] or row['destination_reset']!='all original copy setters before each hypothesis':raise ValueError('live_reset_disclosure')
    for key in ('source_before','source_after_baseline','source_after','formal_before','formal_after'):
        snapshot=row[key];validate_controls(snapshot['controls']);data=base64.b64decode(snapshot['pack_base64'],validate=True)
        if hashlib.sha256(data).hexdigest()!=snapshot['pack_sha256']:raise ValueError('live_pack_hash')
    if row['source_before']!=row['source_after_baseline'] or row['source_before']!=row['source_after']:raise ValueError('live_source_changed')
    if row['formal_before']!=row['formal_after'] or not row['source_unchanged'] or not row['formal_unchanged']:raise ValueError('live_formal_changed')
    validate_controls(row['baseline']['destination_after']['controls'])
    initial=row['source_before']['objects'];expected=json.loads(json.dumps(initial))
    ambiguous=[k for k,v in initial.items() if v['sleeping'] and any(v['linear']+v['angular'])]
    for k in ambiguous:expected[k].update(linear=[0.,0.,0.],angular=[0.,0.,0.])
    if not ambiguous or row['baseline']['failure']!='RuntimeError:forecast_sleep_copy_readback_mismatch' or row['baseline']['destination_after']['objects']!=expected:
        raise ValueError('live_strict_control_not_reproduced')
    if digest(row['path'])!=row['path_sha256']:raise ValueError('live_path_binding')
    for event in row['events']:
        if event['kind']=='forecast_initialization':
            receipt(event)
            raw={k:{f:v[f] for f in ('p','q','linear','angular','sleeping')} for k,v in initial.items()}
            if event['measured_source']!=raw:raise ValueError('live_hypothesis_different_source')
    outcome=classify(row['events'],row['path_sha256'],row['object_id'])
    status='QUALIFIED_FORECAST_PAIR' if outcome['qualified'] else 'REJECTED_FORECAST_PAIR'
    reason=None if outcome['qualified'] else 'all_separation_profiles_physically_rejected'
    if row['status']!=status or row.get('failure') is not None or row.get('rejection_reason')!=reason:raise ValueError('live_boundary_outcome')
    return int(outcome['qualified'])


def bind_trials(row,events):
    """Every hypothesis trial needs its real lifecycle AND ordinary receipt."""
    parent=row['context']['lifecycle'][-1]['preview_id'] if row['context']['lifecycle'] else None
    starts={e['preview_id']:e for e in events if e['kind']=='preview_accounting_started'}
    terminals={e['preview_id']:e for e in events if e['kind']=='preview_accounting_terminal'}
    legacy=[]
    for e in events:
        if e['kind']=='separation_preview':legacy.append(e)
        if e['kind'] in ('grasp_preview','return_preview'):
            legacy.extend(t for t in e.get('separation_trials',e.get('trace',[])) if t['kind']=='separation_preview')
    trials={}
    for e in row['events']:
        if e['kind']!='separation_preview':continue
        i=e['preview_id'];a=starts.get(i);b=terminals.get(i)
        meta=dict(profile=e['profile'],initialization=e['initialization'],forecast_contract=e['forecast_contract'])
        if (a is None or b is None or a['parent_preview_id']!=parent or a['preview_kind']!='separation' or
            a['arm']!=row['arm'] or a['object_id']!=row['object_id'] or a['metadata']!=meta or
            a['physics_step']!=row['context']['formal_step'] or b['status']!='completed' or b['error'] is not None or
            b['accepted'] is not e['accepted'] or b['inclusive_steps']!=e['trial_steps']):raise ValueError('live_trial_lifecycle_binding')
        matches=[v for v in legacy if v.get('preview_id')==i]
        if len(matches)!=1 or any(matches[0].get(k)!=v for k,v in e.items()):raise ValueError('live_trial_legacy_receipt')
        trials[i]=(row,e)
    return trials


def bind_approximate_copy(copied,validated_trials):
    ctx=copied['context'];life=ctx['lifecycle'];trial=validated_trials.get(life[-1]['preview_id']) if life else None
    if trial is None:raise ValueError('unwitnessed_approximate_copy')
    row,e=trial
    if (copied['status']!='completed' or copied['error'] is not None or copied['initialization']!=e['initialization'] or
        ctx['formal_step']!=row['context']['formal_step'] or life[:-1]!=row['context']['lifecycle']):raise ValueError('live_approximate_copy_binding')


def validate_action_clock(ticks,row,events):
    """Account for isolated nested steps absent from the source-only tape."""
    if row['source_role']=='formal':
        last=None
        for tick in ticks:
            if last is not None and tick['formal_step']!=last+1:raise ValueError('live_action_step_gap')
            last=tick['formal_step']
        if last is None or last+1!=row['context']['formal_step']:raise ValueError('live_action_final_step')
        return
    parent=row['context']['lifecycle'][-1]['preview_id'];end=row['context']['preview_step']
    starts=[e for e in events if e['kind']=='preview_accounting_started' and e['preview_id']==parent]
    if len(starts)!=1:raise ValueError('live_action_parent_clock')
    cursor=starts[0]['preview_steps_begin'];intervals=[]
    for e in events:
        if e['kind']!='preview_accounting_terminal' or e['parent_preview_id']!=parent or e['preview_steps_end']>end:continue
        if e['preview_kind']!='separation' or e['object_id']!=row['object_id'] or e['arm']!=row['arm'] or e['status']!='completed' or e['error'] is not None:raise ValueError('live_action_nested_clock')
        a,b=e['preview_steps_begin'],e['preview_steps_end']
        if a<cursor or b<a or e['inclusive_steps']!=b-a:raise ValueError('live_action_nested_interval')
        if b>a:intervals.append((a,b))
    intervals.sort();index=0;tick_seen=False
    for tick in ticks:
        tick_seen=True
        while index<len(intervals) and intervals[index][0]==cursor:cursor=intervals[index][1];index+=1
        if tick['preview_step']!=cursor:raise ValueError('live_action_step_gap')
        cursor+=1
    while index<len(intervals) and intervals[index][0]==cursor:cursor=intervals[index][1];index+=1
    if not tick_seen:raise ValueError('live_action_final_step')
    if cursor!=end or index!=len(intervals):raise ValueError('live_action_unaccounted_nested_steps')


def verify_tape_inventory(out,binding):
    """All closed prefixes, including physically rejected or unqualified ones."""
    from action_tape import records
    folder=Path(out)/'live-action-tapes';count=binding['count']
    if type(count) is not int or count<0:raise ValueError('live_tape_inventory_count')
    if folder.exists() and {p.name for p in folder.iterdir()}!={'%08d'%i for i in range(1,count+1)}:raise ValueError('live_tape_inventory_set')
    h=hashlib.sha256();total=0
    for i in range(1,count+1):
        directory=folder/('%08d'%i);raw=(directory/'terminal.json').read_bytes();h.update(raw);b=json.loads(raw)
        if b['tape_id']!=i or (directory/'index.jsonl').stat().st_size!=b['index_bytes']:raise ValueError('live_tape_terminal_binding')
        if {p.name for p in directory.iterdir()}!={'terminal.json','index.jsonl'}|{'%08d.jsonl.gz'%j for j in range(b['chunks'])}:raise ValueError('live_tape_chunk_set')
        for command in records(out,b):
            if command.get('kind')=='physics_command':validate_controls(command['controls'])
            elif command.get('kind')!='magnet_attached':raise ValueError('live_action_kind')
            total+=1
    if total!=binding['records'] or h.hexdigest()!=binding['terminal_sha256']:raise ValueError('live_tape_inventory_hash')
    return dict(valid=True,prefixes=count,records=total)


def inspect(run,manifest,cfg):
    run=Path(run);out=run/'artifacts';errors=[];qualified=0;rejected=0;inputs={};validated_trials={}
    try:
        expected=expected_binding(run,manifest);expected['scene_manifest_sha256']=file_hash(out/'scene-manifest.json')
        p=out/'live-sleep-receipt.json';terminal=json.loads(p.read_text());inputs[str(p)]=file_hash(p)
        if terminal.get('schema')!='live-sleep-qualification/2' or terminal['binding']!=expected or terminal['formal_credit'] is not False or terminal['goal_complete'] is not False or not terminal['qualification_only']:raise ValueError('live_receipt_binding')
        events=[json.loads(s) for s in (out/'events.jsonl').read_text().splitlines()]
        if manifest.get('runtime_variant') in ('v36-live-combined-011','v36-live-combined-012','v36-live-combined-013','v36-live-combined-014','v36-live-combined-015','v36-live-combined-016','v36-live-combined-017','v36-live-combined-018','v36-live-combined-021') and 'action_tapes' not in terminal:raise ValueError('live_tape_inventory_required')
        if 'action_tapes' in terminal:verify_tape_inventory(out,terminal['action_tapes'])
        logged=[e for e in events if e['kind']=='live_sleep_boundary']
        from summary_stream import rows as summary_rows
        if manifest.get('runtime_variant') in ('v36-live-combined-012','v36-live-combined-013','v36-live-combined-014','v36-live-combined-015','v36-live-combined-016','v36-live-combined-017','v36-live-combined-018','v36-live-combined-021') and 'summary_stream' not in terminal:raise ValueError('live_summary_stream_required')
        # Monotone captured formal trajectory counters permit a single pass.
        trajectory=(out/'trajectory.jsonl').open('rb');rolling=hashlib.sha256();trajectory_count=0;previous_count=0
        qualified_objects=set();summary_count=0
        for index,summary in enumerate(summary_rows(out,terminal),1):
            summary_count=index
            if index>len(logged):raise ValueError('live_boundary_event_coverage')
            requested=summary['context']['trajectory_samples']
            if type(requested) is not int or requested<=0 or requested<previous_count:raise ValueError('live_trajectory_prefix_order')
            while trajectory_count<requested:
                line=trajectory.readline()
                if not line:raise ValueError('live_missing_trajectory_prefix')
                rolling.update(line);trajectory_count+=1
            if rolling.hexdigest()!=summary['context']['trajectory_sha256']:raise ValueError('live_formal_trajectory_prefix_hash')
            previous_count=requested
            folder=out/'live-sleep'/('%04d'%index);sp=folder/'summary.json'
            if json.loads(sp.read_text())!=summary or summary['index']!=index:raise ValueError('live_summary_binding')
            e=logged[index-1]
            if e['summary_path']!=str(sp.relative_to(out)) or e['summary_sha256']!=digest(summary) or e['status']!=summary['status']:raise ValueError('live_log_binding')
            bound=summary['full_report']
            if bound is None or bound['path']!='report.json.gz':raise ValueError('live_report_missing')
            rp=folder/'report.json.gz';row=json.loads(unpack(rp,bound))
            if row['binding']!=expected or row['context']!=summary['context'] or any(row.get(k)!=summary.get(k) for k in ('schema','status','failure','rejection_reason','object_id','arm','path_sha256','source_role','source_unchanged','formal_unchanged')):raise ValueError('live_full_binding')
            if row['context']['formal_step']!=e['physics_step']:raise ValueError('live_formal_step')
            life=row['context']['lifecycle']
            if row['source_role']=='grasp_preview':
                if len(life)!=1 or life[0]['preview_kind'] not in ('grasp','return') or life[0]['object_id']!=row['object_id']:raise ValueError('live_lifecycle')
                starts=[v for v in events if v['kind']=='preview_accounting_started' and v['preview_id']==life[0]['preview_id']]
                if len(starts)!=1 or starts[0]['physics_step']!=row['context']['formal_step']:raise ValueError('live_lifecycle_binding')
            elif row['source_role']=='formal':
                if life or not row['object_id'] in qualified_objects:raise ValueError('formal_hypothesis_without_live_preview_qualification')
            else:raise ValueError('live_source_role')
            prefix=row['grasp_prefix'];validate_controls(prefix['initial']['controls'])
            if prefix.get('encoding')=='chunked-action-tape/1':
                from action_tape import records
                commands=records(out,prefix);tape=out/'live-action-tapes'/('%08d'%prefix['tape_id'])/'index.jsonl'
            else:
                tape=folder/'grasp-actions.jsonl.gz'
                if prefix['path']!=tape.name:raise ValueError('live_action_path')
                tb=dict(compressed_size=prefix['compressed_size'],compressed_sha256=prefix['compressed_sha256'],uncompressed_size=prefix['uncompressed_size'],uncompressed_sha256=prefix['sha256'])
                raw=unpack(tape,tb);commands=(json.loads(line) for line in raw.splitlines())
            attached=[];count=[0]
            def validated_ticks():
                for command in commands:
                    count[0]+=1;kind=command.get('kind')
                    if kind=='physics_command':
                        if command['preview'] is not (row['source_role']=='grasp_preview') or command['formal_step']>e['physics_step']:raise ValueError('live_action_context')
                        validate_controls(command['controls']);yield command
                    elif kind=='magnet_attached':attached.append(command)
                    else:raise ValueError('live_action_kind')
            validate_action_clock(validated_ticks(),row,events)
            if count[0]!=prefix['count'] or not count[0]:raise ValueError('live_action_count')
            if row['source_role']=='grasp_preview' and prefix['initial']!=row['formal_before']:raise ValueError('live_prefix_not_current_formal_snapshot')
            if len(attached)!=1 or attached[0]['raw_drive_limit']!=80 or attached[0]['object_id']!=row['object_id']:raise ValueError('live_attachment_binding')
            if row['context']['held_id']!=row['object_id'] or row['context']['drive_active'] is not True:raise ValueError('live_held_drive_context')
            is_qualified=validate_report(row);qualified+=is_qualified;rejected+=not is_qualified
            if is_qualified and row['source_role']=='grasp_preview':qualified_objects.add(row['object_id'])
            trials=bind_trials(row,events)
            if set(trials)&set(validated_trials):raise ValueError('live_duplicate_trial_coverage')
            validated_trials.update(trials)
            for path in (sp,rp):inputs[str(path)]=file_hash(path)
            from action_tape import hash_file
            inputs[str(tape)]=hash_file(tape)
        trajectory.close()
        if summary_count!=len(logged):raise ValueError('live_boundary_event_coverage')
        # Approximate trial copies may be witnessed rejections, but never
        # unwitnessed motion. Rejecting a boundary grants zero qualification.
        receipt_path=out/'sleep-copy-native-receipt.json'
        if receipt_path.exists():
            from sleep_copy_audit_v35 import trace_records
            copy_receipt=json.loads(receipt_path.read_bytes())
            for copied in trace_records(out,copy_receipt):
                if copied['kind']=='copy_terminal' and copied.get('initialization','exact')!='exact':
                    bind_approximate_copy(copied,validated_trials)
    except Exception as exc:errors.append(type(exc).__name__+':'+str(exc))
    finally:
        if 'trajectory' in locals():trajectory.close()
    return dict(valid=not errors,qualified_boundaries=qualified,rejected_boundaries=rejected,errors=errors,inputs_sha256=inputs,
        boundary_forecast_only=True,formal_credit=False,goal_complete=False)
