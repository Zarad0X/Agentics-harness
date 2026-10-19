"""Source-bound diagnostic arm identity. Does not authorize native execution."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
VARIANT='v36-live-combined-024'
ENABLED=True
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def contract():
    return dict(schema='earlyhold-native-integration/1',runtime_variant=VARIANT,enabled=ENABLED,source_binding_sha256=sha(ROOT/'harness/configs/v36/BINDINGS.json'),physical_validation=False)
def validate_runtime(manifest=None,cfg=None):
    b=json.loads((ROOT/'harness/configs/v36/BINDINGS.json').read_text())
    if b['runtime_variant']!=VARIANT or b['earlyhold_enabled'] is not ENABLED:raise ValueError('earlyhold_runtime_identity')
    for n,h in b['runtime_dependencies'].items():
        p=ROOT/n
        if p.is_symlink() or not p.is_file() or sha(p)!=h:raise ValueError('earlyhold_runtime_source:'+n)
    if manifest is not None:
        if manifest.get('runtime_variant')!=VARIANT or manifest.get('earlyhold_integration')!=contract():raise ValueError('earlyhold_manifest_contract')
        seals=json.loads((ROOT/'harness/configs/v36/BINDINGS.json').read_text())
        if manifest.get('source_sha256')!=seals['source_sha256']:raise ValueError('earlyhold_manifest_source')
        if manifest.get('config_sha256') not in b['continuous_configs'].values():raise ValueError('earlyhold_manifest_config')
        if cfg is not None:
            from provenance import digest
            if digest(cfg)!=manifest['config_sha256']:raise ValueError('earlyhold_effective_config')
    return b
def make_live(api,arm,target,variant,log):
    from live_clearance import Live
    validate_runtime()
    if variant!='B_clearance':raise ValueError('earlyhold_requires_B_clearance')
    return Live(api,arm,target,variant,log,earlyhold=ENABLED)
