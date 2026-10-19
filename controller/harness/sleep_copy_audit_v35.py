"""Independent copy-observer witness integrity; never proof of native completion."""
import json,os,tempfile,shutil
from pathlib import Path
import numpy as np
from provenance import file_hash,digest
from placement_audit_v30 import expected_binding
from live_sleep_readback import validate_identity

def raw(rows,canonical):
    if set(rows)!=set(canonical):raise ValueError('copy_trace_object_coverage')
    for r in rows.values():
        for k,n in (('p',3),('q',4),('linear',3),('angular',3)):
            if r.get(k+'_available') is not True:raise ValueError('copy_trace_unavailable')
            a=np.asarray(r[k],dtype=float)
            if a.shape!=(n,) or not np.isfinite(a).all():raise ValueError('copy_trace_raw_value')
        if type(r.get('sleeping')) is not bool or r.get('sleeping_available') is not True:raise ValueError('copy_trace_sleep_flag')

def diffs(a,b):
    out=[]
    for oid in sorted(a):
        for field in ('p','q','linear','angular','sleeping'):
            x,y=a[oid][field],b[oid][field]
            if x!=y:out.append(dict(object_id=oid,field=field,before=x,after=y,
                delta=None if field=='sleeping' else (np.asarray(y)-np.asarray(x)).tolist()))
    return out

def _verify(rows,receipt,binding,target,events,canonical,db):
    if (receipt.get('schema') not in ('sleep-copy-observer/35','sleep-copy-observer/36-chunked') or receipt.get('binding')!=binding
            or receipt.get('target')!=target or receipt.get('complete') is not True or receipt.get('errors')!=[]
            or receipt.get('active_boundaries')!=0
            or receipt.get('formal_credit') is not False or receipt.get('historical_cause_proved') is not False):raise ValueError('copy_trace_receipt')
    if receipt['schema']=='sleep-copy-observer/35':
        if receipt.get('record_cap')!=2000 or receipt.get('byte_cap')!=64*1024*1024 or len(rows)>2000:raise ValueError('copy_trace_legacy_cap')
    elif (receipt.get('record_cap'),receipt.get('record_raw_cap'),receipt.get('chunk_raw_cap'))!=(450_000_001,512*1024,1024*1024):raise ValueError('copy_trace_profile')
    from copy_trace_index import Map
    starts=Map(db,'starts');terminals=Map(db,'terminals')
    for e in events:
        if e['kind'] in ('preview_accounting_started','preview_accounting_terminal'):
            table=starts if e['kind']=='preview_accounting_started' else terminals
            if e['preview_id'] in table:raise ValueError('copy_trace_duplicate_lifecycle')
            table[e['preview_id']]=e
    copies=Map(db,'copies');rechecks=Map(db,'rechecks');setter_seen=Map(db,'setters');actual=Map(db,'actual')
    active=None;api=None;last_id=0;systems={}
    for i,r in enumerate(rows):
        if type(r.get('sequence')) is not int or r['sequence']!=i:raise ValueError('copy_trace_sequence')
        kind=r['kind']
        if kind=='observer_terminal':
            if i!=len(rows)-1 or active is not None or r['binding']!=binding or r['copies']!=len(copies) or r['completed_boundaries']!=len(copies) or r['active_boundaries']!=0 or r['error']!=receipt['terminal_error']:raise ValueError('copy_trace_terminal')
            continue
        ctx=r['context'];cid=ctx['copy_id'];life=ctx['lifecycle']
        validate_identity(ctx['native_system_identity'],ctx['source_system'],ctx['destination_system'])
        if ctx['binding']!=binding or not life or ctx['source_system']==ctx['destination_system']:raise ValueError('copy_trace_context')
        for j,l in enumerate(life):
            start=starts.get(l['preview_id'])
            if start is None or any(l.get(k)!=start.get(k) for k in ('accounting_schema','preview_id','parent_preview_id','preview_kind','arm','object_id','metadata')) or l['parent_preview_id']!=(None if j==0 else life[j-1]['preview_id']):raise ValueError('copy_trace_lifecycle')
        l=life[-1];t=terminals.get(l['preview_id'])
        if l['object_id']!=target or t is None:raise ValueError('copy_trace_target_terminal')
        roles=('formal','grasp_preview') if l['preview_kind'] in ('grasp','return') else (('grasp_preview','separation_preview') if len(life)>1 else ('formal','separation_preview'))
        if (ctx['source_role'],ctx['destination_role'])!=roles:raise ValueError('copy_trace_roles')
        if (type(ctx['formal_step']) is not int or ctx['formal_step']!=starts[l['preview_id']]['physics_step']
                or type(ctx['preview_step']) is not int or ctx['preview_step']!=starts[l['preview_id']]['preview_steps_begin']
                or ctx['phase']!=starts[l['preview_id']]['phase'] or type(ctx['drive_active']) is not bool):raise ValueError('copy_trace_phase_step')
        for role,key in ((ctx['source_role'],'source_system'),(ctx['destination_role'],'destination_system')):
            if not isinstance(ctx[key],str) or not ctx[key]:raise ValueError('copy_trace_system')
            if role in systems and systems[role]!=ctx[key]:raise ValueError('copy_trace_system_binding')
            systems[role]=ctx[key]
        if len(set(systems.values()))!=len(systems):raise ValueError('copy_trace_system_alias')
        if kind=='copy_before':
            if active is not None or type(cid) is not int or cid!=last_id+1:raise ValueError('copy_trace_boundary_order')
            copies[cid]=r;active=cid;last_id=cid
            pid=life[-1]['preview_id']
            if pid in actual:raise ValueError('copy_trace_duplicate_lifecycle_boundary')
            actual[pid]=cid
        elif cid not in copies or ctx!=copies[cid]['context']:raise ValueError('copy_trace_immutable_context')
        for k in ('source','destination'):
            if k in r:raw(r[k],canonical)
        initial=copies[cid]['source']
        # Only declared separation hypotheses can change destination expectations.
        mode=rows.terminals.get(cid,'exact')
        expected=__import__('copy').deepcopy(initial)
        ambiguous=[k for k,v in initial.items() if v['sleeping'] and any(v['linear']+v['angular'])]
        if mode not in ('exact','sleep_zero','awake_raw'):raise ValueError('copy_trace_unknown_initialization')
        if mode!='exact':
            if not ambiguous or roles not in (('grasp_preview','separation_preview'),('formal','separation_preview')):raise ValueError('copy_trace_hypothesis_scope')
            for k in ambiguous:
                if mode=='sleep_zero':expected[k].update(linear=[0.,0.,0.],angular=[0.,0.,0.])
                else:expected[k]['sleeping']=False
        if 'source_differences' in r and r['source_differences']!=diffs(initial,r['source']):raise ValueError('copy_trace_source_diff')
        if kind=='after_old_setters':
            if active!=cid or api is not None or cid in setter_seen:raise ValueError('copy_trace_setter_phase')
            setter_seen.add(cid)
        elif kind=='sleep_api_before':
            if active!=cid or api is not None or r['method'] not in ('put_to_sleep','wake_up') or r['object_id'] not in canonical:raise ValueError('copy_trace_api')
            if r['destination_differences']!=[] or r['native_error'] is not None:raise ValueError('copy_trace_api_before')
            if cid not in setter_seen:raise ValueError('copy_trace_api_before_setters')
            oid=r['object_id'];desired=expected[oid]['sleeping']
            if (r['destination'][oid]['sleeping']==desired and not (mode=='sleep_zero' and oid in ambiguous)) or r['method']!=('put_to_sleep' if desired else 'wake_up'):raise ValueError('copy_trace_api_policy')
            api=r
        elif kind=='sleep_api_after':
            if api is None or (cid,r['object_id'],r['method'])!=(api['context']['copy_id'],api['object_id'],api['method']):raise ValueError('copy_trace_api_pair')
            if r['destination_differences']!=diffs(api['destination'],r['destination']):raise ValueError('copy_trace_api_diff')
            api=None
        elif kind=='copy_terminal':
            if active!=cid or api is not None or r['status'] not in ('completed','failed') or (r['error'] is None)!=(r['status']=='completed'):raise ValueError('copy_trace_boundary_terminal')
            if r['raw_differences']!=diffs(initial,r['destination']):raise ValueError('copy_trace_raw_diff')
            if r['status']=='completed' and (cid not in setter_seen or diffs(expected,r['destination']) or r['source_differences']):raise ValueError('copy_trace_false_copy')
            if r['status']=='failed' and t['status']!='failed':raise ValueError('copy_trace_failed_lifecycle')
            active=None
        elif kind=='source_recheck':
            if active is not None:raise ValueError('copy_trace_recheck_order')
            if r['error'] is None and r['source_differences']:raise ValueError('copy_trace_false_source_check')
            if cid in rechecks:raise ValueError('copy_trace_duplicate_source_recheck')
            rechecks.add(cid)
        elif kind!='copy_before':raise ValueError('copy_trace_unknown_record')
    if not rows or rows[-1]['kind']!='observer_terminal' or receipt['copies']!=len(copies) or receipt['completed_boundaries']!=len(copies):raise ValueError('copy_trace_missing')
    # Every selected lifecycle (including nested separation) must have a boundary.
    selected=0
    for pid,e in starts.items():
        if e['object_id']==target:
            selected+=1
            if pid not in actual:raise ValueError('copy_trace_lifecycle_coverage')
    if len(actual)!=selected:raise ValueError('copy_trace_lifecycle_coverage')
    for cid,r in copies.items():
        if terminals[r['context']['lifecycle'][-1]['preview_id']]['status']=='completed' and cid not in rechecks:raise ValueError('copy_trace_missing_source_recheck')
    return dict(valid=True,copies=len(copies),boundary_entered=bool(selected),
        coverage='witnessed' if selected else 'not_entered',diagnostic_conclusion=bool(selected),
        formal_credit=False,historical_cause_proved=False)

def verify(rows,receipt,binding,target,events,canonical,*,scratch=None):
    from copy_trace_index import Rows,connect
    # Never default to system /tmp. Runtime inspect passes the data run parent.
    scratch=Path(scratch or os.environ.get('TMPDIR') or Path.cwd())
    if shutil.disk_usage(scratch).free<64*1024*1024:raise ValueError('copy_audit_scratch_reserve')
    with tempfile.TemporaryDirectory(prefix='copy-audit-',dir=scratch) as temp:
        db=connect(Path(temp)/'index.sqlite')
        try:
            indexed=Rows(db,rows)
            return _verify(indexed,receipt,binding,target,events,canonical,db)
        finally:db.close()

def trace_records(out,receipt):
    out=Path(out)
    if receipt.get('schema')=='sleep-copy-observer/36-chunked':
        from action_tape import complete_records
        yield from complete_records(out/'copy-observations',receipt['trace'],profile='copy')
        return
    if receipt.get('schema')!='sleep-copy-observer/35':raise ValueError('copy_trace_schema')
    p=out/'sleep-copy-native.jsonl'
    if p.stat().st_size>64*1024*1024:raise ValueError('copy_trace_byte_cap')
    import hashlib
    h=hashlib.sha256();count=0;size=0
    with p.open('rb') as stream:
        for line in stream:
            count+=1;size+=len(line);h.update(line)
            if count>2000:raise ValueError('copy_trace_hash_cap')
            yield json.loads(line)
    if receipt['trace']!=dict(rows=count,bytes=size,sha256=h.hexdigest()):raise ValueError('copy_trace_hash_cap')

def inspect(run,manifest,cfg):
    run=Path(run);errors=[];target=(cfg or {}).get('sleep_copy_trace_object_id')
    if target is None:return dict(requested=False,valid=True,errors=[],diagnostic_conclusion=False)
    try:
        a=run/'artifacts';receipt=json.loads((a/'sleep-copy-native-receipt.json').read_text())
        if manifest.get('runtime_variant') in ('v36-live-combined-012','v36-live-combined-013','v36-live-combined-014','v36-live-combined-015','v36-live-combined-016','v36-live-combined-017','v36-live-combined-018','v36-live-combined-024') and receipt.get('schema')!='sleep-copy-observer/36-chunked':raise ValueError('copy_chunked_required')
        scene=json.loads((a/'scene-manifest.json').read_text())
        binding=expected_binding(run,manifest);binding['scene_manifest_sha256']=file_hash(a/'scene-manifest.json')
        with (a/'events.jsonl').open() as stream:
            report=verify(trace_records(a,receipt),receipt,binding,target,(json.loads(s) for s in stream),set(scene['objects']),scratch=run.parent)
        return dict(requested=True,errors=[],**report,
            inputs_sha256={str(a/'sleep-copy-native-receipt.json'):file_hash(a/'sleep-copy-native-receipt.json')})
    except Exception as exc:errors.append(type(exc).__name__+':'+str(exc))
    return dict(requested=True,valid=False,errors=errors,diagnostic_conclusion=False,formal_credit=False)
