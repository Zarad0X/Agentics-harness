"""Unambiguous supervisor loading; truthful pre-child exception receipts."""
import importlib.util
import json
from pathlib import Path
import time
import sys
from provenance import file_hash,write_once


def launch_continuous(run,assignment):
    run=Path(run).resolve(strict=True)
    if (run/'startup-failure.json').exists():
        print('diagnostic_ID_has_startup_failure; use a fresh run ID',file=sys.stderr)
        return 2
    try:
        from continuous_v36 import validate_manifest
        validate_manifest(run,json.loads((run/'diagnostic-manifest.json').read_text()))
        path=Path(__file__).resolve().with_name('return_diagnostic_v36.py')
        spec=importlib.util.spec_from_file_location('_sealed_diagnostic_supervisor',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module.launch(run,assignment)
    except Exception as exc:
        # Never overwrite a launch/child/terminal outcome or label it physics.
        if not any((run/n).exists() for n in ('started.json','child.json','terminal.json','startup-failure.json')):
            write_once(run/'startup-failure.json',dict(schema='diagnostic-startup-failure/23',
                state='startup_failed',run_id=run.name,time_unix=time.time(),
                error_type=type(exc).__name__,reason=str(exc),exit_code=2,child_started=False,
                physics_started=False,physical_outcome=None,accepted=False,formal_credit=False,
                manifest_sha256=file_hash(run/'diagnostic-manifest.json') if (run/'diagnostic-manifest.json').exists() else None))
            print(type(exc).__name__+':'+str(exc),file=sys.stderr)
            return 2
        raise
