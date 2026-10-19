"""Synthetic archive fixtures; no native evidence or deployment behavior."""

import gzip
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tarfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import archive_utils as au


def encode(value):
    return json.dumps(value, sort_keys=True).encode()


def pin(raw, role="synthetic fixture"):
    return {
        "bytes": len(raw),
        "sha256": hashlib.sha256(raw).hexdigest(),
        "source_path": "synthetic-test-only",
        "role": role,
    }


class Case:
    def __init__(self, root, core):
        self.root = Path(root)
        self.files = {}
        self.snapshot = "evidence/ledger-snapshots/synthetic-001"
        self.archive = au.owned_directory(root, ".local/archive-cache") / "fixture.tar.gz"
        for name in au.CORE_NAMES:
            raw = au.checked_path(Path(core) / name).read_bytes()
            path = self.root / "harness" / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(raw)
            self.files["work/astra-roundtrip/" + name] = raw
        for name in au.ADAPTER_NAMES:
            self.write_active(
                "adapters/" + name, b"# Synthetic integration source; never executed.\n"
            )
        for name in ("codex22.py", "remote15.py"):
            self.write_active("transport/" + name, b"# Synthetic transport; never executed.\n")
        for name in au.TEST_NAMES:
            raw = b"# Synthetic identity placeholder; never counted as an upstream test.\n"
            self.write_active("tests/core/" + name, raw)
            self.files["tests/core/" + name] = raw
        for name in ("issues.json", "regressions.json"):
            self.write_active("experience/" + name, encode({"synthetic": True, "records": []}))
        spec = importlib.util.spec_from_file_location(
            "_fixture_store", str(self.root / "harness/evolution.py")
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        store = module.Store(au.owned_directory(root, ".local/fixture-ledger"))
        key = store.put(b"synthetic evidence; no physical claim")
        store.append({"id": "fixture", "kind": "task", "objects": [key]})
        for path in store.root.rglob("*"):
            if path.is_file() and path.name != "LOCK":
                self.files[self.snapshot + "/" + path.relative_to(store.root).as_posix()] = (
                    path.read_bytes()
                )
        self.files[au.SNAPSHOT_INDEX] = encode({"snapshots": [{"path": self.snapshot}]})
        self.refresh_active()
        self.repack()

    def write_active(self, name, raw):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(raw)

    def refresh_active(self):
        files = {}
        for folder in ("harness", "adapters", "transport", "tests/core", "experience"):
            for path in (self.root / folder).rglob("*"):
                if path.is_file():
                    files[path.relative_to(self.root).as_posix()] = pin(path.read_bytes())
        self.write_active(
            au.ACTIVE_MANIFEST, encode({"schema": "harness-active-source/1", "files": files})
        )

    def repack(self, refresh_manifest=True, extras=()):
        if refresh_manifest:
            sources = {
                n: pin(raw)
                for n, raw in self.files.items()
                if n not in (au.SOURCE_MANIFEST, au.SNAPSHOT_INDEX)
            }
            self.files[au.SOURCE_MANIFEST] = encode(
                {"schema": "harness-source-snapshot/1", "files": sources}
            )
        entries = []
        for name, raw in sorted(self.files.items()):
            info = tarfile.TarInfo(name)
            info.size = len(raw)
            entries.append((info, raw))
        entries.extend(extras)
        stream = io.BytesIO()
        with tarfile.open(fileobj=stream, mode="w") as tar:
            for info, raw in entries:
                tar.addfile(info, io.BytesIO(raw) if raw is not None else None)
        compressed = gzip.compress(stream.getvalue(), mtime=0)
        self.archive.write_bytes(compressed)
        self.index = {
            "repo": "Zarad0X/robotwin-astra-harness",
            "release_tag": "synthetic-test",
            "asset_name": "fixture.tar.gz",
            "bytes": len(compressed),
            "sha256": hashlib.sha256(compressed).hexdigest(),
            "source_manifest_sha256": hashlib.sha256(self.files[au.SOURCE_MANIFEST]).hexdigest(),
            "unpacked_bytes": sum(info.size for info, _ in entries),
            "members": len(entries),
        }
        self.write_active("archive/index.json", encode(self.index))
        return self.index
