#!/usr/bin/env python3
"""Inspect or selectively restore pinned history under .local; never execute it."""

import argparse
import json
from pathlib import Path
import sys

import archive_utils as au


def restore(root, override, prefix, destination=None):
    root = au.checked_path(root, directory=True)
    au.relative(prefix)
    index = au.read_index(root)
    archive = au.resolve_archive(root, index, override)
    scan = au.scan_archive(archive, index)
    selected = {}
    for name in scan["files"]:
        relative = name[len(scan["prefix"]) :]
        if relative.startswith(prefix + "/"):
            selected[relative] = relative[len(prefix) + 1 :]
    au.require(bool(selected), "no files under selected archive prefix")
    if destination is None:
        return {"files": sorted(selected), "extracted": False}
    parts = au.relative(destination).parts
    au.require(len(parts) >= 2 and parts[0] == ".local", "destination must be a new .local child")
    parent = au.owned_directory(root, str(au.relative(destination).parent))
    dest = parent / parts[-1]
    dest.mkdir()  # Fail even for an empty pre-existing directory or symlink.
    try:
        au.extract_selected(archive, scan, dest, selected)
    except BaseException:
        print("Partial restore retained: " + str(dest), file=sys.stderr)
        raise
    return {"destination": str(dest), "files": len(selected), "extracted": True, "executed": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--archive", type=Path)
    parser.add_argument(
        "--prefix", required=True, help="archive repo-relative directory to inspect/restore"
    )
    parser.add_argument("--destination", help="new .local path; omit to list only")
    args = parser.parse_args()
    try:
        print(
            json.dumps(
                restore(args.root, args.archive, args.prefix, args.destination), sort_keys=True
            )
        )
        return 0
    except (OSError, ValueError, KeyError, TypeError, EOFError) as exc:
        print("archive restore FAILED: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
