"""Separate, assignment-gated supervisor for DIAGNOSTIC ONLY return previews.

Reuses resource/lease checks, never the formal stage/acceptance route. Names
and schemas deliberately differ from run-manifest/summary/production results.
"""
import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
import experiment as supervisor
from provenance import write_once,file_hash,source_files
from run_config import load_config,validate_diagnostic_config
from return_reconstruction import FILES,load_bundle,diagnostic_result

ROOT=Path(__file__).resolve().parents[1]


def prepare(args):
    if not supervisor.RUN_ID.fullmatch(args.run_id):raise ValueError('invalid_run_id')
    if not 60<=args.timeout<=3600 or args.gpu<0 or not 1<=args.max_previews<=40:raise ValueError('diagnostic_resource_limits')
    if args.candidate_index is not None and args.candidate_index<0:raise ValueError('diagnostic_candidate_index')
    bundle=load_bundle(args.bundle,args.target)
    cfg=load_config(args.config)
    continuation=getattr(args,'continuation_diagnostic',None)
    if continuation and (args.target!=12 or args.candidate_index!=6 or args.max_previews!=2 or args.legacy_insertion
            or getattr(args,'fixed_setpoint_diagnostic',None) or not cfg.high_transfer_branch_seed or cfg.retreat_time_scale!=1):
        raise ValueError('continuation_diagnostic_requires_n12_c6_seedon_scale1_two_trials')
    validate_diagnostic_config(cfg,bundle['config'])
    from fixed_setpoint_diagnostic import validate_request
    probe=getattr(args,'fixed_setpoint_diagnostic',None)
    validate_request(probe,dict(bundle=bundle,candidate_index=args.candidate_index,
        max_previews=args.max_previews,legacy_insertion=args.legacy_insertion),cfg)
    from n17_observer import validate_request as validate_n17_observer
    observer=getattr(args,'n17_observer',None)
    validate_n17_observer(observer,dict(bundle=bundle,candidate_index=args.candidate_index,
        max_previews=args.max_previews,legacy_insertion=args.legacy_insertion,
        fixed_setpoint_diagnostic=probe,continuation_diagnostic=continuation),cfg)
    from experience_v4 import load_experience
    load_experience(args.experience,cfg)
    release=args.release.resolve(strict=True);python=args.python.resolve(strict=True)
    release_hashes=supervisor.release_files(release)
    if release_hashes!=bundle['run']['release_sha256']:raise ValueError('diagnostic_release_differs_from_original')
    disk=supervisor.disk_gate(args.root);root=Path(disk['root']);run=root/args.run_id;run.mkdir()
    snap=run/'snapshot';(snap/'src/fixtures').mkdir(parents=True);(snap/'harness').mkdir();(snap/'bundle/artifacts').mkdir(parents=True)
    for p in (ROOT/'src').glob('*.py'):shutil.copyfile(p,snap/'src'/p.name)
    for p in (ROOT/'src/fixtures').iterdir():
        if p.is_file():shutil.copyfile(p,snap/'src/fixtures'/p.name)
    for p in (ROOT/'harness').glob('*.py'):shutil.copyfile(p,snap/'harness'/p.name)
    for n in bundle['hashes']:shutil.copyfile(bundle['path']/n,snap/'bundle'/n)
    write_once(run/'config.json',cfg.to_dict())
    if args.experience:shutil.copyfile(args.experience,snap/'experience.json')
    hashes={str(p.relative_to(snap)):file_hash(p) for p in snap.rglob('*') if p.is_file()}
    for p in snap.rglob('*'):
        if p.is_file():p.chmod(0o444)
    write_once(run/'diagnostic-manifest.json',dict(schema='return-diagnostic-supervisor/9',status='DIAGNOSTIC ONLY',
        retreat_time_scale=cfg.retreat_time_scale,home_time_scale=1,fixed_setpoint_diagnostic=probe,
        home_nominal_speed_rad_s=cfg.home_nominal_speed_rad_s,home_clocks_changed=cfg.home_nominal_speed_rad_s!=.1,
        continuation_diagnostic=continuation,defer_intermediate_home=cfg.defer_intermediate_home,
        n17_observer=observer,home_contact_envelope=cfg.home_contact_envelope,home_progress=cfg.home_progress,home_progress_bound=cfg.home_progress_bound,
        dataset_success=False,full_ten_success=False,
        run_id=args.run_id,release=str(release),release_sha256=release_hashes,python=str(python),gpu=args.gpu,
        timeout_s=args.timeout,snapshot_sha256=hashes,config_sha256=file_hash(run/'config.json'),
        candidate_index=args.candidate_index,max_previews=args.max_previews,legacy_insertion=args.legacy_insertion,target=args.target,
        source_fingerprint=bundle['source_fingerprint'],experience=bool(args.experience),
        accepted=False,whole_episode_success=False,runtime_llm_calls=0,development_usage=None))
    return run


def launch(run,assignment_path):
    run=run.resolve(strict=True);m=json.loads((run/'diagnostic-manifest.json').read_text())
    if m.get('continuous_diagnostic') is True or m.get('schema')=='continuous-diagnostic-supervisor/36':
        from continuous_v36 import validate_manifest
        validate_manifest(run,m)
    with ExitStack() as locks:
        locks.enter_context(supervisor.lock(run/'run.lock'))
        if (run/'started.json').exists() or (run/'terminal.json').exists():raise RuntimeError('diagnostic_ID_already_started')
        supervisor.disk_gate(run)
        for n,h in m['snapshot_sha256'].items():
            if file_hash(run/'snapshot'/n)!=h:raise RuntimeError('diagnostic_snapshot_changed:'+n)
        if file_hash(run/'config.json')!=m['config_sha256']:raise RuntimeError('diagnostic_config_changed')
        if supervisor.release_files(Path(m['release']))!=m['release_sha256']:raise RuntimeError('diagnostic_release_changed')
        headless=m.get('execution',{}).get('mode')=='headless'
        if headless:
            from headless_resource_v36 import assignment_gate,environment
            assignment=assignment_gate(assignment_path,run,m)
            locks.enter_context(supervisor.lock(run.parent/('cpu-'+str(assignment['cpu_core'])+'.lock')))
            assignment_gate(assignment_path,run,m)
            if environment(m['release'],m['headless_environment']['asset_root'])!=m['headless_environment']:raise RuntimeError('headless_environment_changed')
            gpu=dict(headless=True,gpu_queried=False,uuid='',index=None,pci_bus_id=None,cpu_core=assignment['cpu_core'])
        else:
            assignment=supervisor.assignment_gate(assignment_path,run,m);gpu=supervisor.resources(m['gpu'])
            if assignment['gpu_uuid']!=gpu['uuid']:raise RuntimeError('GPU_UUID_mismatch')
            locks.enter_context(supervisor.lock(run.parent/('gpu-'+gpu['uuid']+'.lock')))
            gpu=supervisor.resources(m['gpu']);supervisor.assignment_gate(assignment_path,run,m)
        if m.get('continuous_diagnostic') is True:
            if m.get('schema')=='continuous-diagnostic-supervisor/36':
                from continuous_v36 import validate_manifest
            elif m.get('schema')=='continuous-diagnostic-supervisor/25':
                from continuous_v25 import validate_manifest
            elif m.get('schema')=='continuous-diagnostic-supervisor/24':
                from continuous_v24 import validate_manifest
            elif m.get('schema')=='continuous-diagnostic-supervisor/23':
                from continuous_v23 import validate_manifest
            else:
                from continuous_v22 import validate_manifest
            validate_manifest(run,m)
            command=[m['python'],str(run/('snapshot/src/continuous_diagnostic_v36.py' if m.get('schema')=='continuous-diagnostic-supervisor/36' else 'snapshot/src/continuous_diagnostic_v25.py' if m.get('schema')=='continuous-diagnostic-supervisor/25' else 'snapshot/src/continuous_diagnostic_v24.py' if m.get('schema')=='continuous-diagnostic-supervisor/24' else 'snapshot/src/continuous_diagnostic.py')),
                '--release',m['release'],'--output',str(run/'artifacts'),'--output-root',str(run),'--gpu',str(0 if headless else m['gpu']),
                '--config',str(run/'config.json')]
        else:
            command=[m['python'],str(run/'snapshot/src/return_diagnostic.py'),'--bundle',str(run/'snapshot/bundle'),
                '--release',m['release'],'--output',str(run/'artifacts'),'--output-root',str(run),'--gpu',str(0 if headless else m['gpu']),
                '--config',str(run/'config.json'),'--max-previews',str(m['max_previews'])]
            command+=['--target',str(m.get('target',10))]
        if headless:command+=['--execution-manifest',str(run/'diagnostic-manifest.json')]
        if m['experience']:command+=['--experience-snapshot',str(run/'snapshot/experience.json')]
        if m['candidate_index'] is not None:command+=['--candidate-index',str(m['candidate_index'])]
        if m['legacy_insertion']:command+=['--legacy-insertion']
        if m.get('fixed_setpoint_diagnostic') is not None:
            command+=['--fixed-setpoint-diagnostic',m['fixed_setpoint_diagnostic']]
        if m.get('continuation_diagnostic') is not None:
            command+=['--continuation-diagnostic',m['continuation_diagnostic']]
        if m.get('n17_observer') is not None:
            command+=['--n17-observer',m['n17_observer']]
        write_once(run/'assignment.json',assignment)
        write_once(run/'started.json',dict(command=command,time_unix=time.time(),supervisor_pid=os.getpid(),host=socket.gethostname(),gpu=gpu))
        start=time.monotonic();proc=None;interrupted=[];code=125;reason='launch_failed'
        def cancel(signum,frame):
            interrupted.append(signum)
            if proc is not None and proc.poll() is None:
                try:os.killpg(proc.pid,signal.SIGTERM)
                except ProcessLookupError:pass
        handlers={s:signal.signal(s,cancel) for s in (signal.SIGINT,signal.SIGTERM)}
        try:
            with (run/'runner.log').open('x') as log:
                proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
                                      env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
                write_once(run/'child.json',dict(pid=proc.pid,start_ticks=Path('/proc',str(proc.pid),'stat').read_text().split()[21]))
                try:code=proc.wait(timeout=m['timeout_s']);reason='exited'
                except subprocess.TimeoutExpired:code=124;reason='timeout'
                if interrupted:code=128+interrupted[0];reason='cancelled'
        finally:
            if proc is not None and proc.poll() is None:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=20)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            for s,h in handlers.items():signal.signal(s,h)
            write_once(run/'terminal.json',dict(state='terminal',run_id=run.name,exit_code=code,reason=reason,
                                               elapsed_s=time.monotonic()-start,status='DIAGNOSTIC ONLY'))
            # No summary.json, formal run manifest, verifier invocation, or
            # accepted smoke/full10 receipt can be created by this supervisor.
            write_once(run/'diagnostic-summary.json',diagnostic_result(exit_code=code,reason=reason,
                artifact_result_sha256=file_hash(run/'artifacts/diagnostic-result.json') if (run/'artifacts/diagnostic-result.json').exists() else None))
        return code


def main():
    ap=argparse.ArgumentParser();sub=ap.add_subparsers(dest='action',required=True)
    p=sub.add_parser('prepare')
    for name in ('root','bundle','release','python','config'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--experience',type=Path);p.add_argument('--run-id',required=True);p.add_argument('--gpu',type=int,required=True)
    p.add_argument('--timeout',type=int,default=900);p.add_argument('--candidate-index',type=int)
    p.add_argument('--max-previews',type=int,default=10);p.add_argument('--legacy-insertion',action='store_true')
    p.add_argument('--target',type=int,choices=(10,12),default=10)
    p.add_argument('--fixed-setpoint-diagnostic',choices=('baseline','hold'))
    p.add_argument('--continuation-diagnostic',choices=('natural','same-first','switch-first'))
    p.add_argument('--n17-observer',choices=('baseline','observe'))
    p=sub.add_parser('launch');p.add_argument('run',type=Path);p.add_argument('--assignment',type=Path,required=True)
    a=ap.parse_args()
    if a.action=='prepare':print(prepare(a));return 0
    return launch(a.run,a.assignment)


if __name__=='__main__':raise SystemExit(main())
