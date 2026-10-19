"""Read-only v36 config/source/policy/dependency validator; no device query."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'harness')]
from continuous_scope_v36 import sealed_config
from run_config import load_config
from provenance import file_hash,digest

def validate(path):
    cfg=load_config(path);b=sealed_config(cfg,ROOT)
    if file_hash(path) not in b['continuous_configs'].values():raise ValueError('config_bytes_not_frozen')
    policy=ROOT/b['policy']
    if file_hash(policy)!=b['policy_file_sha256']:raise ValueError('policy_file_binding')
    p=json.loads(policy.read_text())
    if p['implementation_sha256']!=b['implementation_sha256'] or p['policy_sha256']!=digest({k:v for k,v in p.items() if k!='policy_sha256'}):raise ValueError('policy_source_binding')
    return dict(config=Path(path).name,config_sha256=file_hash(path),implementation_sha256=b['implementation_sha256'],
                bindings_sha256=file_hash(ROOT/'harness/configs/v36/BINDINGS.json'),layout=cfg.layout,count=cfg.count,seed=cfg.seed,
                profiler=cfg.home_cpu_profile,experience=False,formal_credit=False,validated=True,physical_runs=0)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('config',type=Path);a=p.parse_args();print(json.dumps(validate(a.config)))
