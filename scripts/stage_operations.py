#!/usr/bin/env python3
"""Copy the exact active integration sources into a fresh historical layout.

Does not supply a live ledger, configure credentials, rewrite paths or start jobs.
"""

import argparse
import json
from pathlib import Path
import shutil
import sys

import archive_utils as au


def stage(root):
    root = au.checked_path(root, directory=True)
    active = au.check_active(root)
    mapping = {"harness/" + n: "work/astra-roundtrip/" + n for n in au.CORE_NAMES}
    mapping.update({"adapters/" + n: "work/astra-roundtrip/" + n for n in au.ADAPTER_NAMES})
    mapping.update(
        {
            "transport/codex22.py": "outputs/codex22.py",
            "transport/remote15.py": "work/astra-direct/remote15.py",
        }
    )
    parent = au.owned_directory(root, ".local")
    target = parent / "operations"
    target.mkdir()  # Intentionally no exist_ok: never clobber a prior staging.
    for source, destination in mapping.items():
        path = target / destination
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(au.safe_path(root, source), path)
        au.require(
            au.digest_file(path) == active[source]["sha256"], "source changed during staging"
        )
    receipt = {
        "schema": "harness-operations-staging/1",
        "ready_for_live_operations": False,
        "mapping": mapping,
        "sha256": {n: active[n]["sha256"] for n in mapping},
        "live_ledger": "external; not supplied",
        "executed": False,
    }
    (target / "STAGING.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return {
        "destination": str(target),
        "files": len(mapping),
        "executed": False,
        "ready_for_live_operations": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    try:
        print(json.dumps(stage(args.root), sort_keys=True))
        return 0
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print("operations staging FAILED: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
