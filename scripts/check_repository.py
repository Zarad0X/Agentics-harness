#!/usr/bin/env python3
"""Offline lean-repository verification; never fetches or repairs evidence."""

import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import sys

import archive_utils as au

sys.dont_write_bytecode = True
ARCHIVED_CORE = "work/astra-roundtrip/"


def snapshot_paths(scan):
    raw = scan["metadata"].get(scan["prefix"] + au.SNAPSHOT_INDEX)
    au.require(raw is not None, "missing archived snapshot index")
    index = au.json_bytes(raw)
    au.require(
        isinstance(index, dict) and isinstance(index.get("snapshots"), list) and index["snapshots"],
        "invalid archived snapshot index",
    )
    paths = []
    for entry in index["snapshots"]:
        path = (
            entry
            if isinstance(entry, str)
            else entry.get("path")
            if isinstance(entry, dict)
            else None
        )
        parts = au.relative(path).parts
        au.require(
            len(parts) == 3 and parts[:2] == ("evidence", "ledger-snapshots"),
            "unsafe snapshot path",
        )
        au.require(path not in paths, "duplicate snapshot identity")
        paths.append(path)
    for name in scan["files"]:
        relative = name[len(scan["prefix"]) :]
        if relative.startswith("evidence/") and relative != au.SNAPSHOT_INDEX:
            au.require(relative in scan["sources"], "unmanifested archived evidence")
        if relative.startswith("evidence/ledger-snapshots/") and relative != au.SNAPSHOT_INDEX:
            au.require(any(relative.startswith(p + "/") for p in paths), "unindexed snapshot")
    return paths


def verify_ledgers(root, archive, scan):
    """Use the archived implementation and immutable closure in writable copies."""
    paths = snapshot_paths(scan)
    mapping = {}
    for name in au.CORE_NAMES:
        source = ARCHIVED_CORE + name
        au.require(source in scan["sources"], "missing pinned archived core")
        mapping[source] = "src/" + name
    object_counts = {}
    for snapshot in paths:
        for leaf in ("ledger.jsonl", "HEAD.json"):
            au.require(snapshot + "/" + leaf in scan["sources"], "missing snapshot " + leaf)
        count = 0
        for name in scan["files"]:
            source = name[len(scan["prefix"]) :]
            if not source.startswith(snapshot + "/"):
                continue
            tail = source[len(snapshot) + 1 :]
            parts = au.relative(tail).parts
            au.require(
                tail in ("ledger.jsonl", "HEAD.json", "LOCK")
                or (len(parts) == 2 and parts[0] == "objects" and au.valid_hash(parts[1])),
                "unexpected snapshot file",
            )
            if parts[0] == "objects":
                au.require(scan["files"][name]["sha256"] == parts[1], "object address mismatch")
                count += 1
            mapping[source] = source
        object_counts[snapshot] = count
    temporary = au.new_workspace(root, "ledger-")
    success = False
    try:
        au.extract_selected(archive, scan, temporary, mapping)
        spec = importlib.util.spec_from_file_location(
            "_archived_evolution", str(temporary / "src/evolution.py")
        )
        module = importlib.util.module_from_spec(spec)
        sys.path.insert(0, str(temporary / "src"))
        try:
            spec.loader.exec_module(module)
            summaries = []
            for snapshot in paths:
                rows = module.Store(temporary / snapshot).rows()
                summaries.append(
                    {"path": snapshot, "records": len(rows), "objects": object_counts[snapshot]}
                )
        finally:
            sys.path.pop(0)
        success = True
        return summaries
    finally:
        if success:
            shutil.rmtree(temporary)
        else:
            print("Failed ledger check retained: " + str(temporary), file=sys.stderr)


def check_repository(root, override=None):
    root = au.checked_path(root, directory=True)
    active = au.check_active(root)
    index = au.read_index(root)
    archive = au.resolve_archive(root, index, override)
    scan = au.scan_archive(archive, index)
    for name in au.TEST_NAMES:
        path = "tests/core/" + name
        archived = scan["sources"].get(path)
        au.require(archived is not None, "missing original core test pin")
        au.require(
            all(archived[k] == active[path][k] for k in ("bytes", "sha256")),
            "core tests changed from archived original",
        )
    summaries = verify_ledgers(root, archive, scan)
    au.require(au.check_active(root) == active, "active sources changed during verification")
    au.validate_blob(archive, index)
    return {
        "ok": True,
        "active_files": len(active),
        "archived_files": scan["archived_files"],
        "archive_members": scan["members"],
        "unpacked_bytes": scan["unpacked_bytes"],
        "archived_source_files": len(scan["sources"]),
        "source_verification": "passed",
        "ledger_verification": summaries,
        "physical_acceptance": "not assessed",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--archive", type=Path, help="explicit local archive; never downloaded")
    args = parser.parse_args()
    try:
        print(json.dumps(check_repository(args.root, args.archive), sort_keys=True))
        return 0
    except (OSError, ValueError, KeyError, TypeError, EOFError) as exc:
        print("repository verification FAILED: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
