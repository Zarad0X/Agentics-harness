"""Sealed v26 development diagnostics only; no production promotion."""
import json
from pathlib import Path
from provenance import digest,file_hash,source_files


def validate_scope(cfg):
    if (type(cfg.count) is not int or type(cfg.seed) is not int or cfg.mode!='roundtrip'
            or cfg.split!='development' or not cfg.reserve_home_workspace or not cfg.home_entry_hold
            or cfg.defer_intermediate_home or cfg.experience or cfg.optimize_geometry
            or not cfg.high_transfer_branch_seed
            or not ((cfg.layout=='original' and cfg.count==10 and cfg.seed==2)
                    or (cfg.layout=='generated' and cfg.count in (10,15,20) and cfg.seed in (101,103)))):
        raise ValueError('v26_continuous_development_scope')



def sealed_config(cfg,root=None):
    root=Path(root) if root is not None else Path(__file__).resolve().parents[1]
    validate_scope(cfg)
    b=json.loads((root/'harness/configs/v26/BINDINGS.json').read_text())
    source=source_files(root/'src')
    if (b['source_sha256']!=source or b['implementation_sha256']!=digest(source)
            or digest(cfg.to_dict()) not in b['continuous_configs'].values()):
        raise ValueError('v26_continuous_sealed_source_config')
    # Exact dependency list travels with the runtime, including historical
    # binding JSON needed by imported harness helpers. No latest-version search.
    for name,h in b['runtime_dependencies'].items():
        path=(root/name).resolve()
        if not path.is_relative_to(root.resolve()) or file_hash(path)!=h:
            raise ValueError('v26_runtime_dependency:'+name)
    return b
