#!/usr/bin/env python3
"""Run exactly 44 original tests against CURRENT harness core, offline."""

import argparse
import importlib
import json
from pathlib import Path
import shutil
import sys
import unittest

import archive_utils as au

sys.dont_write_bytecode = True


def offline_audit(event, args):
    if event in {
        "subprocess.Popen",
        "os.system",
        "os.exec",
        "os.posix_spawn",
        "pty.spawn",
        "socket.connect",
        "socket.bind",
        "socket.getaddrinfo",
    }:
        raise RuntimeError("offline core test prohibited event: " + event)


def stage_fixtures(archive, scan, temporary):
    prefix = "tests/fixtures/"
    pins_name = prefix + "PINS.json"
    au.require(pins_name in scan["sources"], "missing frozen fixture pins")
    au.extract_selected(archive, scan, temporary, {pins_name: "fixtures/PINS.json"})
    pins = au.read_json(temporary / "fixtures/PINS.json")
    au.require(
        isinstance(pins.get("incoming"), dict) and bool(pins["incoming"]),
        "empty incoming fixture pins",
    )
    mapping = {prefix + "previous-evolution.py": "fixtures/previous-evolution.py"}
    for name, expected in pins["incoming"].items():
        au.relative(name)
        source = prefix + "incoming/" + name
        au.require(
            source in scan["sources"] and scan["sources"][source]["sha256"] == expected,
            "frozen incoming pin mismatch",
        )
        mapping[source] = "fixtures/incoming/" + name
    previous = scan["sources"].get(prefix + "previous-evolution.py", {})
    au.require(
        previous.get("sha256") == pins.get("previous_evolution_sha256"),
        "previous core fixture pin mismatch",
    )
    for name in au.WRAPPERS:
        mapping[prefix + "adapters/" + name] = "adapters/" + name
    au.require(all(name in scan["sources"] for name in mapping), "unmanifested fixture source")
    au.extract_selected(archive, scan, temporary, mapping)


def run(root, override=None):
    root = au.checked_path(root, directory=True)
    active = au.check_active(root)
    index = au.read_index(root)
    archive = au.resolve_archive(root, index, override)
    scan = au.scan_archive(archive, index)
    for name in au.TEST_NAMES:
        path = "tests/core/" + name
        original = scan["sources"].get(path)
        au.require(
            original is not None
            and all(original[k] == active[path][k] for k in ("bytes", "sha256")),
            "test modules must be byte-exact archived originals",
        )
    temporary = au.new_workspace(root, "core-")
    success = False
    try:
        stage_fixtures(archive, scan, temporary)
        for name in au.CORE_NAMES:
            dest = temporary / "src" / name
            dest.parent.mkdir(exist_ok=True)
            shutil.copyfile(au.safe_path(root, "harness/" + name), dest)
        for name in au.TEST_NAMES:
            dest = temporary / "tests" / name
            dest.parent.mkdir(exist_ok=True)
            shutil.copyfile(au.safe_path(root, "tests/core/" + name), dest)
        # Never fall back to a previously imported/archived core in this process.
        au.require(
            not any(
                n in sys.modules
                for n in (
                    "evolution",
                    "parent_adapter",
                    "usage_accounting",
                    "test_evolution",
                    "test_revision",
                )
            ),
            "core runner requires a fresh Python process",
        )
        sys.path[:0] = [str(temporary / "src"), str(temporary / "tests")]
        sys.addaudithook(offline_audit)
        modules = [importlib.import_module(Path(name).stem) for name in au.TEST_NAMES]
        for module in modules:
            module.IN = temporary / "fixtures/incoming"
        suite = unittest.TestSuite(
            unittest.defaultTestLoader.loadTestsFromModule(m) for m in modules
        )
        au.require(suite.countTestCases() == 44, "expected exactly 44 unchanged upstream tests")
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        success = result.wasSuccessful() and result.testsRun == 44 and not result.skipped
        summary = {
            "mode": "current-active-core-with-frozen-fixtures",
            "tests": result.testsRun,
            "failures": len(result.failures),
            "errors": len(result.errors),
            "skipped": len(result.skipped),
            "archive_sha256": index["sha256"],
            "source_manifest_sha256": index["source_manifest_sha256"],
            "current_core_sha256": {n: active["harness/" + n]["sha256"] for n in au.CORE_NAMES},
            "physical_acceptance": "not assessed",
            "native_execution": False,
        }
        (temporary / "result.json").write_text(json.dumps(summary, indent=2) + "\n")
        au.require(au.check_active(root) == active, "active sources changed during test")
        print(json.dumps(summary, sort_keys=True))
        return 0 if success else 1
    except BaseException:
        success = False
        raise
    finally:
        if success:
            shutil.rmtree(temporary)
        else:
            print("Failed core run retained: " + str(temporary), file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--archive", type=Path, help="explicit local archive; no fetch")
    args = parser.parse_args()
    try:
        return run(args.root, args.archive)
    except (OSError, ValueError, KeyError, TypeError, EOFError) as exc:
        print("core test FAILED: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
