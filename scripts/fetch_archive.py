#!/usr/bin/env python3
"""Explicit private-release download via existing gh authentication, never implicit."""

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile

import archive_utils as au


def download(root):
    root = au.checked_path(root, directory=True)
    index = au.read_index(root)
    # Operator must create these parents explicitly. No home/auth discovery.
    cache = au.safe_path(root, ".local/archive-cache", directory=True)
    target = cache / index["asset_name"]
    if target.exists() or target.is_symlink():
        au.validate_blob(target, index)
        return {"cache": str(target), "downloaded": False, "sha256": index["sha256"]}
    temporary = Path(tempfile.mkdtemp(prefix="download-", dir=str(cache)))
    command = [
        "gh",
        "release",
        "download",
        index["release_tag"],
        "--repo",
        index["repo"],
        "--pattern",
        index["asset_name"],
        "--dir",
        str(temporary),
    ]
    success = False
    try:
        # Inherit existing gh auth; never inspect environment values or auth files.
        # Suppress gh output to avoid accidentally recording authentication detail.
        process = subprocess.Popen(
            command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True
        )
        try:
            code = process.wait(timeout=55)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
            raise au.Invalid("gh download timed out; partial cache retained")
        au.require(
            code == 0, "gh download failed; check private-release access outside these tools"
        )
        downloaded = au.validate_blob(temporary / index["asset_name"], index)
        au.checked_path(cache, directory=True)
        # Atomic no-clobber publication; racing existing files are never replaced.
        os.link(str(downloaded), str(target))
        success = True
        return {"cache": str(target), "downloaded": True, "sha256": index["sha256"]}
    finally:
        if success:
            shutil.rmtree(temporary)
        else:
            print("Download attempt retained: " + str(temporary), file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    try:
        print(json.dumps(download(args.root), sort_keys=True))
        return 0
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print("archive fetch FAILED: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
