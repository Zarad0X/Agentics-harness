"""Adversarial lean-verifier tests using real Store with synthetic evidence."""

import json
import os
from pathlib import Path
import subprocess
import tarfile
import tempfile
import unittest
from unittest import mock

from support import Case, ROOT, encode
import archive_utils as au
import check_repository
import fetch_archive
import restore_archive
import stage_operations


class RepositoryTests(unittest.TestCase):
    def setUp(self):
        # macOS /var is a symlink. Keep test-created roots canonical without
        # weakening the production rejection of user-supplied symlink paths.
        parent = au.owned_directory(ROOT, ".local/test-cases")
        self.temp = tempfile.TemporaryDirectory(dir=str(parent))
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        source = Path(os.environ.get("HARNESS_TEST_CORE", str(ROOT / "harness")))
        self.case = Case(self.root, source)

    def verify(self):
        return check_repository.check_repository(self.root)

    def test_valid_archive_and_real_store_leave_originals_unchanged(self):
        archive = self.case.archive.read_bytes()
        result = self.verify()
        self.assertEqual(result["ledger_verification"][0]["records"], 1)
        self.assertEqual(result["physical_acceptance"], "not assessed")
        self.assertEqual(self.case.archive.read_bytes(), archive)
        self.assertFalse((self.root / "evidence").exists())

    def test_future_indexed_snapshot(self):
        second = "evidence/ledger-snapshots/synthetic-002"
        for name, raw in list(self.case.files.items()):
            if name.startswith(self.case.snapshot + "/"):
                self.case.files[second + name[len(self.case.snapshot) :]] = raw
        self.case.files[au.SNAPSHOT_INDEX] = encode({"snapshots": [self.case.snapshot, second]})
        self.case.repack()
        self.assertEqual(len(self.verify()["ledger_verification"]), 2)

    def test_active_tamper_and_partial_manifest(self):
        path = self.root / "harness/evolution.py"
        path.write_bytes(path.read_bytes() + b"\n")
        with self.assertRaisesRegex(au.Invalid, "active source mismatch"):
            self.verify()
        self.case.refresh_active()
        manifest = au.read_json(self.root / au.ACTIVE_MANIFEST)
        del manifest["files"]["adapters/account22.py"]
        self.case.write_active(au.ACTIVE_MANIFEST, encode(manifest))
        with self.assertRaisesRegex(au.Invalid, "incomplete active"):
            self.verify()

    def test_unmanifested_active_file(self):
        self.case.write_active("harness/unreviewed.py", b"# unreviewed\n")
        with self.assertRaisesRegex(au.Invalid, "exactly the active"):
            self.verify()

    def test_current_core_identity_can_differ_from_archived_baseline(self):
        path = self.root / "harness/evolution.py"
        path.write_bytes(path.read_bytes() + b"\n# Synthetic current-source identity variation.\n")
        self.case.refresh_active()
        self.assertTrue(self.verify()["ok"])

    def test_original_test_identity_cannot_silently_change(self):
        self.case.write_active("tests/core/test_evolution.py", b"# different\n")
        self.case.refresh_active()
        with self.assertRaisesRegex(au.Invalid, "tests changed"):
            self.verify()

    def test_compressed_tamper_and_source_manifest_pin(self):
        self.case.archive.write_bytes(self.case.archive.read_bytes() + b"x")
        with self.assertRaisesRegex(au.Invalid, "compressed size"):
            self.verify()
        self.case.repack()
        index = dict(self.case.index, source_manifest_sha256="0" * 64)
        self.case.write_active("archive/index.json", encode(index))
        with self.assertRaisesRegex(au.Invalid, "manifest pin"):
            self.verify()

    def test_inner_source_tamper_despite_fresh_outer_hash(self):
        self.case.files["work/astra-roundtrip/evolution.py"] += b"\n"
        self.case.repack(refresh_manifest=False)
        with self.assertRaisesRegex(au.Invalid, "archived source mismatch"):
            self.verify()

    def test_chain_corruption_with_refreshed_source_pins(self):
        path = self.case.snapshot + "/ledger.jsonl"
        self.case.files[path] = self.case.files[path].replace(b'"fixture"', b'"changed"')
        self.case.repack()
        with self.assertRaisesRegex(ValueError, "ledger chain mismatch"):
            self.verify()

    def test_missing_head_and_object_never_repaired(self):
        head = self.case.files.pop(self.case.snapshot + "/HEAD.json")
        self.case.repack()
        with self.assertRaisesRegex(au.Invalid, "missing snapshot HEAD"):
            self.verify()
        self.case.files[self.case.snapshot + "/HEAD.json"] = head
        key = next(n for n in self.case.files if "/objects/" in n)
        del self.case.files[key]
        self.case.repack()
        with self.assertRaisesRegex(ValueError, "missing immutable object"):
            self.verify()

    def test_object_address_and_unindexed_snapshot(self):
        key = next(n for n in self.case.files if "/objects/" in n)
        old = self.case.files[key]
        self.case.files[key] = b"changed"
        self.case.repack()
        with self.assertRaisesRegex(au.Invalid, "object address"):
            self.verify()
        self.case.files[key] = old
        self.case.files["evidence/ledger-snapshots/hidden/HEAD.json"] = b"{}"
        self.case.repack()
        with self.assertRaisesRegex(au.Invalid, "unindexed"):
            self.verify()

    def test_duplicate_paths(self):
        info = tarfile.TarInfo("tests/core/test_evolution.py")
        self.case.repack(extras=[(info, b"")])
        with self.assertRaisesRegex(au.Invalid, "duplicate archive path"):
            self.verify()

    def test_traversal_absolute_windows_and_special_members(self):
        for name in ("../escape", "/absolute", "a/../b", "a//b", "C:/bad", "a\\b"):
            with self.subTest(name=name):
                info = tarfile.TarInfo(name)
                self.case.repack(extras=[(info, b"")])
                with self.assertRaisesRegex(au.Invalid, "unsafe relative"):
                    self.verify()
        for kind in (tarfile.SYMTYPE, tarfile.LNKTYPE, tarfile.FIFOTYPE, tarfile.CHRTYPE):
            with self.subTest(kind=kind):
                info = tarfile.TarInfo("unsafe-member")
                info.type = kind
                info.linkname = "tests/core/test_evolution.py"
                self.case.repack(extras=[(info, None)])
                with self.assertRaisesRegex(au.Invalid, "ordinary files"):
                    self.verify()

    def test_census_and_independent_limits(self):
        self.case.write_active(
            "archive/index.json",
            encode(dict(self.case.index, members=self.case.index["members"] + 1)),
        )
        with self.assertRaisesRegex(au.Invalid, "census"):
            self.verify()
        self.case.repack()
        with mock.patch.object(au, "MAX_BYTES", 128):
            with self.assertRaisesRegex(au.Invalid, "total byte ceiling"):
                au.scan_archive(self.case.archive, self.case.index)
        with mock.patch.object(au, "MAX_MEMBERS", 2):
            with self.assertRaisesRegex(au.Invalid, "member ceiling"):
                au.scan_archive(self.case.archive, self.case.index)
        info = tarfile.TarInfo("oversize")
        info.size = au.MAX_FILE + 1
        self.case.repack(extras=[(info, None)])
        with self.assertRaisesRegex(au.Invalid, "per-file ceiling"):
            self.verify()

    def test_file_parent_conflicts(self):
        parent = tarfile.TarInfo("conflict")
        child = tarfile.TarInfo("conflict/child")
        for entries in ([(parent, b""), (child, b"")], [(child, b""), (parent, b"")]):
            self.case.repack(extras=entries)
            with self.assertRaisesRegex(au.Invalid, "file/directory conflict"):
                self.verify()

    def test_local_symlink_and_duplicate_json_keys(self):
        path = self.root / "harness/evolution.py"
        path.unlink()
        path.symlink_to(self.root / "harness/parent_adapter.py")
        with self.assertRaisesRegex(au.Invalid, "symlink"):
            self.verify()
        with self.assertRaisesRegex(au.Invalid, "duplicate JSON"):
            au.json_bytes(b'{"x":1,"x":2}')

    def test_fetch_existing_cache_is_offline_and_tamper_not_refetched(self):
        with mock.patch.object(
            fetch_archive.subprocess, "Popen", side_effect=AssertionError("network forbidden")
        ):
            self.assertFalse(fetch_archive.download(self.root)["downloaded"])
            self.case.archive.write_bytes(b"bad")
            with self.assertRaisesRegex(au.Invalid, "size mismatch"):
                fetch_archive.download(self.root)

    def test_fetch_mocked_command_no_clobber_and_no_auth_inspection(self):
        raw = self.case.archive.read_bytes()
        self.case.archive.unlink()

        def fake(command, **kwargs):
            self.assertNotIn("env", kwargs)
            self.assertEqual(command[:3], ["gh", "release", "download"])
            self.assertNotIn("--clobber", command)
            (Path(command[-1]) / "fixture.tar.gz").write_bytes(raw)
            return mock.Mock(wait=mock.Mock(return_value=0))

        with mock.patch.object(fetch_archive.subprocess, "Popen", side_effect=fake):
            self.assertTrue(fetch_archive.download(self.root)["downloaded"])
        self.assertEqual(self.case.archive.read_bytes(), raw)

    def test_fetch_missing_or_symlink_cache_parent_refused(self):
        cache = self.case.archive.parent
        moved = cache.with_name("preserved-cache")
        cache.rename(moved)
        with self.assertRaises(FileNotFoundError):
            fetch_archive.download(self.root)
        cache.symlink_to(moved, target_is_directory=True)
        with self.assertRaisesRegex(au.Invalid, "symlink"):
            fetch_archive.download(self.root)

    def test_fetch_timeout_preserves_partial_attempt(self):
        self.case.archive.unlink()
        process = mock.Mock(
            pid=999999, wait=mock.Mock(side_effect=[subprocess.TimeoutExpired("gh", 55), 1])
        )
        with mock.patch.object(
            fetch_archive.subprocess, "Popen", return_value=process
        ), mock.patch.object(fetch_archive.os, "killpg") as kill:
            with self.assertRaisesRegex(au.Invalid, "timed out"):
                fetch_archive.download(self.root)
            kill.assert_called_once()
        self.assertFalse(self.case.archive.exists())

    def test_restore_is_selective_and_refuses_overwrite(self):
        result = restore_archive.restore(
            self.root, None, self.case.snapshot, ".local/disposable-ledger"
        )
        self.assertTrue(result["extracted"])
        self.assertFalse((self.root / ".local/disposable-ledger/work").exists())
        with self.assertRaises(FileExistsError):
            restore_archive.restore(self.root, None, self.case.snapshot, ".local/disposable-ledger")

    def test_stage_only_sources_no_ledger_or_execution(self):
        result = stage_operations.stage(self.root)
        self.assertEqual(result["files"], 16)
        self.assertTrue(
            (
                self.root / ".local/operations/work/astra-roundtrip/upload_evidence22_data3.py"
            ).is_file()
        )
        target = Path(result["destination"])
        self.assertFalse((target / "work/astra-roundtrip/evolution-ledger").exists())
        self.assertFalse(result["ready_for_live_operations"])
        with self.assertRaises(FileExistsError):
            stage_operations.stage(self.root)

    def test_missing_export_never_false_success(self):
        (self.root / au.ACTIVE_MANIFEST).unlink()
        with self.assertRaises(FileNotFoundError):
            self.verify()


if __name__ == "__main__":
    unittest.main()
