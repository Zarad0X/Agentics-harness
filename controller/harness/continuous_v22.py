"""Fresh continuous diagnostic preparation; existing assignment-gated launcher.

No reconstruction input, production manifest, dataset acceptance, or automatic
retry. The first stage and full10 share one initialized physical trajectory.
"""
import argparse
import json
from pathlib import Path
import shutil
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'harness')]
import experiment as supervisor
from provenance import source_files,digest,file_hash,write_once
from run_config import load_config


def validate_manifest(run,m):
    cfg=load_config(run/'config.json')
    if (m.get('schema')!='continuous-diagnostic-supervisor/22' or m.get('continuous_diagnostic') is not True
            or m.get('accepted') is not False or m.get('reconstructed') is not False
            or not cfg.reserve_home_workspace or cfg.mode!='roundtrip' or cfg.layout!='original' or cfg.count!=10
            or cfg.split!='development' or cfg.defer_intermediate_home):raise ValueError('continuous_manifest_contract')
    if m['source_sha256']!={n[4:]:h for n,h in m['snapshot_sha256'].items() if n.startswith('src/')}:
        raise ValueError('continuous_source_binding')
    b=json.loads((ROOT/'harness/configs/v22/BINDINGS.json').read_text())
    if (m['source_sha256']!=b['source_sha256'] or m['config_sha256'] not in b['configs'].values()
            or file_hash(run/'config.json')!=m['config_sha256']):raise ValueError('continuous_sealed_binding')
    if m['experience']:
        if file_hash(run/'snapshot/experience.json')!=b['policy_file_sha256']:raise ValueError('continuous_policy_binding')


def prepare(a):
    cfg=load_config(a.config);b=json.loads((ROOT/'harness/configs/v22/BINDINGS.json').read_text())
    source=source_files(ROOT/'src')
    if source!=b['source_sha256'] or digest(source)!=b['implementation_sha256'] or file_hash(a.config) not in b['configs'].values():
        raise ValueError('continuous_sealed_source_config')
    if not cfg.reserve_home_workspace or cfg.mode!='roundtrip' or cfg.layout!='original' or cfg.count!=10:
        raise ValueError('continuous_fresh_original10_only')
    if not supervisor.RUN_ID.fullmatch(a.run_id) or not 60<=a.timeout<=28800 or a.gpu<0:raise ValueError('continuous_resource_contract')
    from experience_v4 import load_experience
    load_experience(a.experience,cfg)
    if cfg.experience and (a.experience is None or file_hash(a.experience)!=b['policy_file_sha256']):raise ValueError('continuous_sealed_policy')
    disk=supervisor.disk_gate(a.root);release=a.release.resolve(strict=True);python=a.python.resolve(strict=True)
    hashes=supervisor.release_files(release)
    run=Path(disk['root'])/a.run_id;run.mkdir();snap=run/'snapshot'
    for folder in ('src','harness'):
        (snap/folder).mkdir(parents=True)
        for p in (ROOT/folder).glob('*.py'):shutil.copyfile(p,snap/folder/p.name)
    shutil.copytree(ROOT/'src/fixtures',snap/'src/fixtures',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    # The launcher uses explicit sealed bindings; no historical evidence copied.
    (snap/'harness/configs/v22').mkdir(parents=True)
    shutil.copyfile(ROOT/'harness/configs/v22/BINDINGS.json',snap/'harness/configs/v22/BINDINGS.json')
    if a.experience:shutil.copyfile(a.experience,snap/'experience.json')
    write_once(run/'config.json',cfg.to_dict())
    mapping={str(p.relative_to(snap)):file_hash(p) for p in sorted(snap.rglob('*')) if p.is_file()}
    m=dict(schema='continuous-diagnostic-supervisor/22',continuous_diagnostic=True,reconstructed=False,
        accepted=False,whole_episode_success=False,full_ten_success=False,dataset_success=False,formal_credit=False,
        run_id=a.run_id,release=str(release),release_sha256=hashes,python=str(python),gpu=a.gpu,timeout_s=a.timeout,
        snapshot_sha256=mapping,source_sha256=source,config_sha256=file_hash(run/'config.json'),disk=disk,
        experience=cfg.experience,candidate_index=None,max_previews=None,legacy_insertion=False,
        source_fingerprint=None,runtime_llm_calls=0,development_usage=None)
    validate_manifest(run,m)
    write_once(run/'diagnostic-manifest.json',m)
    for p in snap.rglob('*'):
        if p.is_file():p.chmod(0o444)
    return run


if __name__=='__main__':
    p=argparse.ArgumentParser();s=p.add_subparsers(dest='action',required=True)
    q=s.add_parser('prepare')
    for name in ('root','release','python','config'):q.add_argument('--'+name,type=Path,required=True)
    q.add_argument('--run-id',required=True);q.add_argument('--gpu',type=int,required=True)
    q.add_argument('--timeout',type=int,default=28800);q.add_argument('--experience',type=Path)
    q=s.add_parser('launch');q.add_argument('run',type=Path);q.add_argument('--assignment',type=Path,required=True)
    a=p.parse_args()
    if a.action=='prepare':print(prepare(a))
    else:
        from return_diagnostic import launch
        raise SystemExit(launch(a.run,a.assignment))
