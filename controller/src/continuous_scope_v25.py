"""Sealed v25 development diagnostics only; no production promotion."""
import json
from pathlib import Path
from provenance import digest,file_hash,source_files


def validate_scope(cfg):
    if (cfg.mode!='roundtrip' or cfg.layout!='original' or cfg.count!=10 or cfg.seed!=2
            or cfg.split!='development' or not cfg.reserve_home_workspace or not cfg.home_entry_hold
            or cfg.defer_intermediate_home or cfg.experience or cfg.optimize_geometry):
        raise ValueError('v25_continuous_original10_scope')



def sealed_config(cfg,root=None):
    root=Path(root) if root is not None else Path(__file__).resolve().parents[1]
    validate_scope(cfg)
    b=json.loads((root/'harness/configs/v25/BINDINGS.json').read_text())
    source=source_files(root/'src')
    if (b['source_sha256']!=source or b['implementation_sha256']!=digest(source)
            or digest(cfg.to_dict()) not in b['continuous_configs'].values()):
        raise ValueError('v25_continuous_sealed_source_config')
    # Exact dependency list travels with the runtime, including historical
    # binding JSON needed by imported harness helpers. No latest-version search.
    for name,h in b['runtime_dependencies'].items():
        path=(root/name).resolve()
        if not path.is_relative_to(root.resolve()) or file_hash(path)!=h:
            raise ValueError('v25_runtime_dependency:'+name)
    return b
