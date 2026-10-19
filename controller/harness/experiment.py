#!/usr/bin/env python3
"""Run preparation, foreground supervision, status and independent reporting.

No SSH, model API or credentials. Parent supplies a short-lived GPU assignment.
All launch checks fail closed. A run ID is never reused or resumed physically.
"""
import argparse
from contextlib import ExitStack
import csv
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from provenance import write_once,file_hash
from run_config import RunConfig,load_config

MOUNTS={'/data4','/data_ft_6','/data_ft_7','/data_ft_9_10'}
RUN_ID=re.compile(r'[a-zA-Z0-9][a-zA-Z0-9_.-]{0,95}\Z')


def disk_gate(root,min_free_gib=20):
    root=Path(root).resolve(strict=True)
    report=json.loads(subprocess.check_output(['findmnt','-J','-T',str(root),'-o','TARGET,SOURCE,FSTYPE'],text=True))['filesystems'][0]
    if report['target'] not in MOUNTS or report['fstype'] not in ('ext4','xfs'):raise RuntimeError('unsupported_data_disk:'+str(report))
    free=shutil.disk_usage(root).free
    if free<min_free_gib*1024**3:raise RuntimeError('insufficient_data_disk_space')
    return dict(report,free_bytes=free,root=str(root))


def resources(gpu):
    value=subprocess.check_output(['nvidia-smi',f'--id={gpu}','--query-gpu=uuid,memory.used,memory.total,utilization.gpu','--format=csv,noheader,nounits'],text=True).strip()
    uuid,used,total,util=[x.strip() for x in value.split(',')];used,total,util=map(int,(used,total,util))
    jobs=subprocess.check_output(['nvidia-smi',f'--id={gpu}','--query-compute-apps=pid','--format=csv,noheader,nounits'],text=True).strip()
    if jobs or used>1024 or total-used<12000 or util>10:raise RuntimeError('GPU_not_idle:'+value)
    memory={}
    for line in Path('/proc/meminfo').read_text().splitlines():
        name,value=line.split(':',1);memory[name]=int(value.strip().split()[0])*1024
    if memory['MemAvailable']<8*1024**3:raise RuntimeError('insufficient_host_memory')
    load=os.getloadavg()[0]
    if load>max(1,(os.cpu_count() or 1)*.9):raise RuntimeError('host_CPU_busy')
    return dict(gpu=gpu,uuid=uuid,used_mib=used,total_mib=total,utilization_percent=util,host_available_bytes=memory['MemAvailable'],load1=load)


def release_files(release):
    # Explicit runtime source/robot allowlist. Never recursively archive assets,
    # dot-directories, credentials or old planner/candidate pool outputs.
    base=release/'workspace/base';candidate=release/'workspace/candidate'
    paths=[candidate/n for n in ('empty_container_inner0926_v11.py','run_real10_v11_targeted.py','franka-panda/config.yml','franka-panda/panda.urdf')]
    paths.append(base/'task_config/demo_franka.yml')
    paths.extend(sorted((base/'envs').rglob('*.py')))
    paths.extend([base/'assets/objects/inner0926_layouts'/n for n in ('1_0.json','1_0_settled.json')])
    return {str(p.relative_to(release)):file_hash(p) for p in paths}


def prepare(root,run_id,config,release,python,gpu,timeout=14400,experience=None,evaluation_plan=None,layout_selection=None):
    if not RUN_ID.fullmatch(run_id):raise ValueError('invalid run ID')
    disk=disk_gate(root);root=Path(disk['root']);release=Path(release).resolve(strict=True)
    config=load_config(Path(config));python=Path(python).resolve(strict=True)
    if config.home_progress:raise ValueError('home_progress_diagnostic_only_until_physical_gate')
    if config.home_contact_envelope:raise ValueError('home_contact_envelope_diagnostic_only_until_physical_gate')
    if config.home_coarse:raise ValueError('home_coarse_diagnostic_only_until_physical_gate')
    if config.velocity_feedback=='k6' or config.feedback_scope=='calibration':raise ValueError('feedback_setting_diagnostic_only')
    if not 60<=timeout<=28800 or gpu<0:raise ValueError('invalid resource limits')
    if config.experience and experience is None:raise ValueError('missing experience snapshot')
    if experience:
        from experience_v4 import load_experience
        load_experience(Path(experience),config)  # Validate before reserving a run ID.
    if config.split=='heldout' and evaluation_plan is None:raise ValueError('heldout_requires_prefrozen_evaluation_plan')
    if evaluation_plan is not None:
        from evaluation import check_plan
        check_plan(config,evaluation_plan,experience)
    expected_layout=None
    if config.split=='heldout' and config.mode!='layout':
        if layout_selection is None:raise ValueError('heldout_roundtrip_requires_frozen_layout_selection')
        from evaluation import selected_scene
        expected_layout=selected_scene(config,evaluation_plan,layout_selection)
    # Compute required release evidence before reserving an immutable ID.
    hashes=release_files(release)
    run=root/run_id;run.mkdir()  # Atomic refusal if anyone already owns this ID.
    snapshot=run/'snapshot';(snapshot/'src/fixtures').mkdir(parents=True);(snapshot/'harness').mkdir()
    for p in sorted((ROOT/'src').glob('*.py')):shutil.copy2(p,snapshot/'src'/p.name)
    for p in sorted((ROOT/'src/fixtures').iterdir()):
        if p.is_file():shutil.copy2(p,snapshot/'src/fixtures'/p.name)
    for p in sorted((ROOT/'harness').glob('*.py')):shutil.copy2(p,snapshot/'harness'/p.name)
    if experience:
        # Freeze the exact policy before evaluation, not a live mutable path.
        data=json.loads(Path(experience).read_text())
        if data.get('schema') not in ('roundtrip-experience/1','roundtrip-experience/2','roundtrip-experience/4'):raise ValueError('experience schema')
        write_once(snapshot/'experience.json',data)
    if evaluation_plan is not None:
        write_once(snapshot/'evaluation-plan.json',json.loads(Path(evaluation_plan).read_text()))
        write_once(snapshot/'evaluation-policy.json',json.loads((Path(evaluation_plan).parent/'experience.json').read_text()))
    if expected_layout is not None:write_once(snapshot/'expected-layout.json',expected_layout)
    snapshot_hashes={str(p.relative_to(snapshot)):file_hash(p) for p in sorted(snapshot.rglob('*')) if p.is_file()}
    for p in snapshot.rglob('*'):
        if p.is_file():p.chmod(0o444)
    config_sha=write_once(run/'config.json',config.to_dict())
    write_once(run/'run-manifest.json',dict(schema='roundtrip-run/1',run_id=run_id,prepared_unix=time.time(),
        config_sha256=config_sha,snapshot_sha256=snapshot_hashes,release=str(release),release_sha256=hashes,
        python=str(python),gpu=gpu,timeout_s=timeout,disk=disk,experience=bool(experience),
        evaluation_plan_sha256=file_hash(evaluation_plan) if evaluation_plan else None,expected_layout=expected_layout,
        development_usage=None,runtime_llm_calls=0))
    return run


def assignment_gate(path,run,manifest):
    assignment=json.loads(Path(path).read_text());now=time.time()
    if assignment['run_id']!=run.name or assignment['gpu']!=manifest['gpu'] or assignment['host']!=socket.gethostname():raise RuntimeError('assignment_scope_mismatch')
    if not assignment['issued_unix']<=now<assignment['expires_unix'] or assignment['expires_unix']-assignment['issued_unix']>3600:raise RuntimeError('assignment_expired_or_too_long')
    if Path(assignment['output_root']).resolve()!=run.parent:raise RuntimeError('assignment_disk_mismatch')
    return assignment


def lock(path):
    f=path.open('a+')
    try:fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:f.close();raise RuntimeError('already_running:'+str(path))
    return f


def status(run):
    run=Path(run)
    if (run/'terminal.json').exists():return json.loads((run/'terminal.json').read_text())
    if not (run/'started.json').exists():return dict(state='prepared',run_id=run.name)
    try:f=lock(run/'run.lock')
    except RuntimeError:return dict(state='running',run_id=run.name)
    f.close()
    return dict(state='interrupted_unreconciled',run_id=run.name,restart_allowed=False,
                action='Use reconcile; preserve evidence and allocate a fresh run ID for another physical episode.')


def verify_and_report(run):
    # Separate process, source snapshot; runner success is never sufficient.
    manifest=json.loads((run/'run-manifest.json').read_text());artifacts=run/'artifacts'
    verification=artifacts/'independent-verification.json'
    if artifacts.exists() and not verification.exists():
        attempt=0
        while (run/f'verification-{attempt}.log').exists():attempt+=1
        with (run/f'verification-{attempt}.log').open('x') as log:
            try:
                subprocess.run([manifest['python'],str(run/('snapshot/src/layout_preflight.py' if json.loads((run/'config.json').read_text())['mode']=='layout' else 'snapshot/src/verify_roundtrip.py')),str(artifacts),
                    '--source-root',str(run/'snapshot/src')],stdout=log,stderr=subprocess.STDOUT,timeout=1200,check=False)
            except subprocess.TimeoutExpired:
                log.write('Independent verifier timed out. Acceptance withheld.\n')
    report=json.loads(verification.read_text()) if verification.exists() else dict(accepted=False,whole_episode_success=False,errors=['no_physics_evidence'])
    terminal=json.loads((run/'terminal.json').read_text())
    if json.loads((run/'config.json').read_text())['mode']=='layout':
        report.update(mode='layout',accepted=False,whole_episode_success=False,extraction_rate=None,exact_pose_return_rate=None)
        report['layout_valid']=bool(report.get('layout_valid') and terminal.get('exit_code')==0)
    report['whole_episode_success']=bool(report.get('whole_episode_success') and terminal.get('exit_code')==0)
    write_once(run/'summary.json',dict(run_id=run.name,**report))
    return report


def stage_gate(root,config):
    if config['mode'] in ('smoke','layout'):return
    evidence=[]
    for p in Path(root).glob('*/summary.json'):
        report=json.loads(p.read_text());terminal=p.parent/'terminal.json'
        if terminal.exists() and json.loads(terminal.read_text()).get('exit_code')==0:evidence.append(report)
    if not any(r.get('mode')=='smoke' and r.get('accepted') for r in evidence):raise RuntimeError('first_GPU_evidence_must_be_original_clutter_smoke')
    if config['layout']=='generated' or config['count']>10:
        if not any(r.get('mode')=='roundtrip' and r.get('accepted') and r.get('whole_episode_success') and r.get('scene_count')==10 for r in evidence):raise RuntimeError('full10_roundtrip_required_before_dense_sweeps')


def launch(run,assignment_path):
    with ExitStack() as locks:return _launch(run,assignment_path,locks)


def _launch(run,assignment_path,locks):
    run=Path(run).resolve(strict=True);run_lock=locks.enter_context(lock(run/'run.lock'))
    if (run/'started.json').exists() or (run/'terminal.json').exists():raise RuntimeError('run_ID_already_started; use status/reconcile, not relaunch')
    manifest=json.loads((run/'run-manifest.json').read_text());disk_gate(run)
    stage_gate(run.parent,json.loads((run/'config.json').read_text()))
    assignment=assignment_gate(assignment_path,run,manifest)
    for n,h in manifest['snapshot_sha256'].items():
        if file_hash(run/'snapshot'/n)!=h:raise RuntimeError('snapshot_changed:'+n)
    if file_hash(run/'config.json')!=manifest['config_sha256']:raise RuntimeError('config_changed')
    if release_files(Path(manifest['release']))!=manifest['release_sha256']:raise RuntimeError('release_changed')
    gpu=resources(manifest['gpu'])
    if assignment['gpu_uuid']!=gpu['uuid']:raise RuntimeError('GPU_UUID_mismatch')
    gpu_lock=locks.enter_context(lock(run.parent/('gpu-'+gpu['uuid']+'.lock')))
    # Recheck AFTER acquiring the lease; keep both locks until child terminates.
    gpu=resources(manifest['gpu']);assignment_gate(assignment_path,run,manifest)
    command=[manifest['python'],str(run/'snapshot/src/physics_runner.py'),'--release',manifest['release'],
        '--output',str(run/'artifacts'),'--output-root',str(run),'--gpu',str(manifest['gpu']),'--config',str(run/'config.json')]
    if manifest.get('expected_layout'):command+=['--expected-layout',str(run/'snapshot/expected-layout.json')]
    if manifest['experience']:command+=['--experience-snapshot',str(run/'snapshot/experience.json')]
    write_once(run/'assignment.json',assignment)
    write_once(run/'started.json',dict(time_unix=time.time(),supervisor_pid=os.getpid(),host=socket.gethostname(),gpu=gpu,command=command))
    started=time.monotonic();proc=None;code=None;reason=None;interrupted=[None]
    def on_signal(signum,frame):
        interrupted[0]=signum
        if proc is not None and proc.poll() is None:
            try:os.killpg(proc.pid,signal.SIGTERM)
            except ProcessLookupError:pass
    previous={s:signal.signal(s,on_signal) for s in (signal.SIGINT,signal.SIGTERM)}
    try:
        env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1')
        with (run/'runner.log').open('x') as log:
            proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=env)
            write_once(run/'child.json',dict(pid=proc.pid,start_ticks=Path('/proc/'+str(proc.pid)+'/stat').read_text().split()[21]))
            try:code=proc.wait(timeout=manifest['timeout_s'])
            except subprocess.TimeoutExpired:
                reason='timeout';os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=20)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                code=124
        if interrupted[0] is not None:reason='cancelled';code=128+interrupted[0]
        elif reason is None:reason='exited'
    except Exception as exc:
        reason=type(exc).__name__+':'+str(exc);code=125
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid,signal.SIGTERM)
            try:proc.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    finally:
        for s,handler in previous.items():signal.signal(s,handler)
        write_once(run/'terminal.json',dict(state='terminal',run_id=run.name,exit_code=code,reason=reason,elapsed_s=time.monotonic()-started,time_unix=time.time()))
        gpu_lock.close()
    try:report=verify_and_report(run)
    finally:run_lock.close()
    return 0 if code==0 and (report.get('layout_valid') if report.get('mode')=='layout' else report['accepted']) else 2


def reconcile(run):
    with ExitStack() as locks:return _reconcile(run,locks)


def _reconcile(run,locks):
    run=Path(run).resolve(strict=True);f=locks.enter_context(lock(run/'run.lock'))
    if (run/'terminal.json').exists():
        if not (run/'summary.json').exists():verify_and_report(run)
        f.close();return status(run)
    if not (run/'started.json').exists():raise RuntimeError('not_started')
    if (run/'child.json').exists():
        child=json.loads((run/'child.json').read_text());p=Path('/proc')/str(child['pid'])/'stat'
        if p.exists() and p.read_text().split()[21]==child['start_ticks']:raise RuntimeError('owned_child_still_alive; no_duplicate_launch')
    write_once(run/'terminal.json',dict(state='terminal',run_id=run.name,exit_code=None,reason='supervisor_lost_exit_unknown',time_unix=time.time()))
    verify_and_report(run);f.close();return status(run)


def aggregate(root,output):
    rows=[]
    for p in sorted(Path(root).glob('*/run-manifest.json')):
        run=p.parent;cfg=json.loads((run/'config.json').read_text());summary_path=run/'summary.json'
        summary=json.loads(summary_path.read_text()) if summary_path.exists() else {}
        terminal=status(run)
        rows.append(dict(run_id=run.name,mode=cfg['mode'],count=cfg['count'],seed=cfg['seed'],layout=cfg['layout'],split=cfg['split'],
            experience=cfg['experience'],optimize_geometry=cfg['optimize_geometry'],state=terminal['state'],exit_code=terminal.get('exit_code'),
            layout_valid=summary.get('layout_valid'),layout_status=summary.get('layout_status'),
            whole_episode_success=bool(summary.get('whole_episode_success') and terminal.get('exit_code')==0),
            extraction_rate=summary.get('extraction_rate'),exact_pose_return_rate=summary.get('exact_pose_return_rate'),
            elapsed_s=terminal.get('elapsed_s'),physical_steps=summary.get('physical_steps'),preview_steps=summary.get('preview_steps'),
            candidates=summary.get('candidates'),previews=summary.get('previews'),errors=';'.join(summary.get('errors',[]))))
    write_once(Path(str(output)+'.json'),rows)
    with Path(str(output)+'.csv').open('x',newline='') as f:
        if rows:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    return rows


def main():
    ap=argparse.ArgumentParser();sub=ap.add_subparsers(dest='command',required=True)
    p=sub.add_parser('prepare');p.add_argument('--root',type=Path,required=True);p.add_argument('--run-id',required=True);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--release',type=Path,required=True);p.add_argument('--python',required=True);p.add_argument('--gpu',type=int,required=True);p.add_argument('--timeout',type=int,default=14400);p.add_argument('--experience',type=Path);p.add_argument('--evaluation-plan',type=Path);p.add_argument('--layout-selection',type=Path)
    p=sub.add_parser('launch');p.add_argument('run',type=Path);p.add_argument('--assignment',type=Path,required=True)
    for command in ('status','reconcile'):
        p=sub.add_parser(command);p.add_argument('run',type=Path)
    p=sub.add_parser('aggregate');p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True)
    a=ap.parse_args()
    if a.command=='prepare':print(prepare(a.root,a.run_id,a.config,a.release,a.python,a.gpu,a.timeout,a.experience,a.evaluation_plan,a.layout_selection));return 0
    if a.command=='launch':return launch(a.run,a.assignment)
    if a.command=='status':print(json.dumps(status(a.run),indent=2))
    if a.command=='reconcile':print(json.dumps(reconcile(a.run),indent=2))
    if a.command=='aggregate':print(json.dumps(aggregate(a.root,a.output),indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
