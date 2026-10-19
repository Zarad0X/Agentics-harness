#!/usr/bin/env python3
"""Verify the single maintained controller's source; never run simulation."""

import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
import archive_utils as au


def check(root):
    root = au.checked_path(root, directory=True)
    source = au.read_json(au.safe_path(root, "provenance/CONTROLLER_SOURCE.json"))
    binding = au.safe_path(root, "controller/RUNTIME_BINDING.json")
    au.require(
        au.digest_file(binding) == source["runtime_binding_sha256"], "controller binding changed"
    )
    declared = au.read_json(binding)
    files = dict(declared["files_sha256"])
    files["RUNTIME_BINDING.json"] = source["runtime_binding_sha256"]
    au.require(files == source["files"], "controller inventory changed")
    au.require(declared["runtime_variant"] == source["variant"], "controller variant mismatch")
    for name, digest in files.items():
        au.require(
            au.digest_file(au.safe_path(root, "controller/" + name)) == digest,
            "controller source changed without reviewed binding: " + name,
        )
    actual = set()
    for path in (root / "controller").rglob("*"):
        au.checked_path(path, directory=path.is_dir())
        if path.is_file():
            actual.add(path.relative_to(root / "controller").as_posix())
    au.require(actual == set(files), "unbound extra controller files")
    return {
        "ok": True,
        "variant": source["variant"],
        "bound_files": len(files),
        "physical_acceptance": "not assessed",
        "simulation_launched": False,
    }


if __name__ == "__main__":
    print(json.dumps(check(Path(__file__).resolve().parents[1]), sort_keys=True))
