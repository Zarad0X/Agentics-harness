"""Read a completed profile sidecar early. Never a terminal/physical audit."""
import argparse,json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'harness')]
from provenance import file_hash
from continuous_v36 import validate_manifest


def inspect(run):
    run=Path(run);errors=[];capture=None;state='missing';valid=False
    try:
        manifest=json.loads((run/'diagnostic-manifest.json').read_text());validate_manifest(run,manifest)
        cfg=json.loads((run/'config.json').read_text());art=run/'artifacts'
        success=art/'home-cpu-001.json';failure=art/'home-cpu-001.failure.json'
        if not cfg.get('home_cpu_profile',False):
            if success.exists() or failure.exists():raise ValueError('undeclared_profile')
            state='disabled'
        elif failure.exists():
            state='reporting_failed';capture=json.loads(failure.read_text())
            if capture.get('capture_success') is not False:raise ValueError('failure_claims_success')
        elif not success.exists():state='pending_or_not_reached'
        else:
            capture=json.loads(success.read_text());state='complete'
            b=json.loads((ROOT/'harness/configs/v36/BINDINGS.json').read_text())
            expected=dict(source_sha256=b['source_sha256'],implementation_sha256=b['implementation_sha256'],
                config_sha256=manifest['config_sha256'],bindings_sha256=file_hash(ROOT/'harness/configs/v36/BINDINGS.json'))
            if capture['binding']!=expected:raise ValueError('profile_binding')
            if (capture['schema']!='home-cpu/36' or capture['capture_success'] is not True or capture['actual'] is not True
                    or capture['preview'] is not False or type(capture['invocation']) is not int or capture['invocation']!=1
                    or type(capture['max_calls']) is not int or capture['max_calls']!=1
                    or capture['motion_phase']!='home' or capture['arm'] not in ('left','right')
                    or type(capture['object_id']) is not int
                    or any(capture[k] is not False for k in ('formal_credit','terminal','physical_success_inferred'))
                    or capture['profiler_overhead_can_affect_deadlines'] is not True):raise ValueError('profile_contract')
            for ctx in (capture['start'],capture['end']):
                if ctx['preview'] is not False or type(ctx['step']) is not int or ctx['step']<0 or not isinstance(ctx['phase'],str):raise ValueError('profile_context')
            if capture['end']['step']<capture['start']['step']:raise ValueError('profile_clock')
            def number(x):return type(x) in (int,float) and math.isfinite(x) and x>=0
            if not all(number(capture[k]) for k in ('wall_s','process_cpu_s','profile_total_exclusive_s')):raise ValueError('profile_timing')
            if capture['physical_exception'] is not None and (not isinstance(capture['physical_exception'],dict) or not isinstance(capture['physical_exception'].get('type'),str)):
                raise ValueError('profile_physical_exception')
            rows=capture['functions'];keys=[]
            if not rows:raise ValueError('empty_profile')
            for row in rows:
                keys.append((row['file'],row['line'],row['function']))
                if (type(row['line']) is not int or not isinstance(row['file'],str) or not isinstance(row['function'],str)
                        or not all(type(row[k]) is int and row[k]>=0 for k in ('calls','primitive_calls'))
                        or not all(number(row[k]) for k in ('exclusive_s','inclusive_s'))):raise ValueError('profile_row')
                for edge in row['callers']:
                    if len(edge['values'])!=4 or not all(number(v) for v in edge['values']):raise ValueError('profile_callers')
            if len(keys)!=len(set(keys)):raise ValueError('duplicate_profile_function')
            if not math.isclose(sum(row['exclusive_s'] for row in rows),capture['profile_total_exclusive_s'],rel_tol=1e-9,abs_tol=1e-9):raise ValueError('profile_exclusive_partition')
            events=art/'events.jsonl'
            if events.exists():
                # Only complete already-flushed lines; never require terminal.
                lines=events.read_text().splitlines(keepends=True)
                homes=[json.loads(line) for line in lines if line.endswith('\n')]
                homes=[e for e in homes if e.get('kind')=='home_contact_model' and e.get('motion_phase')=='home']
                if homes:
                    first=homes[0]
                    if (first['arm'],first['object_id'],first['scene_step'])!=(capture['arm'],capture['object_id'],capture['start']['step']):raise ValueError('not_first_actual_HOME')
            valid=True
    except Exception as exc:errors.append(type(exc).__name__+':'+str(exc))
    return dict(schema='home-cpu-audit/36',state=state,capture_valid=valid,errors=errors,capture=capture,
                terminal_status_not_inferred=True,physical_success_inferred=False,formal_credit=False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();r=inspect(a.run)
    print(json.dumps(r));raise SystemExit(0 if r['capture_valid'] else 2)
