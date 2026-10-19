"""Shared offline integrity and selective archive I/O. Python 3.8+, stdlib.

Use a quiescent, reviewed checkout. Digests are identity, not execution evidence.
No extractall, network, credential reads, or source workspace access.
"""

import gzip
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import tarfile
import tempfile

MAX_BYTES = 128 * 1024 * 1024
MAX_FILE = 16 * 1024 * 1024
MAX_MEMBERS = 8000
MAX_COMPRESSED = 64 * 1024 * 1024
MAX_STREAM = MAX_BYTES + MAX_MEMBERS * 2048 + 10240
HEX = re.compile(r"[0-9a-f]{64}\Z")
SOURCE_MANIFEST = "provenance/SOURCE_MANIFEST.json"
SNAPSHOT_INDEX = "evidence/ledger-snapshots/index.json"
ACTIVE_MANIFEST = "provenance/ACTIVE_SOURCE_MANIFEST.json"
CORE_NAMES = ("evolution.py", "parent_adapter.py", "usage_accounting.py")
TEST_NAMES = ("test_evolution.py", "test_revision.py")
WRAPPERS = (
    "append_completed_usage.py.patched",
    "completed_usage_source.py.patched",
    "dispatch22_data3.py.patched",
    "collect_live_combined012_15.py.patched",
    "launch_existing_combined15.py.patched",
)
ADAPTER_NAMES = (
    "dispatch22_data3.py",
    "collect_live_combined012_15.py",
    "launch_existing_combined15.py",
    "account22.py",
    "append_completed_usage.py",
    "completed_usage_source.py",
    "sync_completed_usage22.py",
    "watch019020_terminals.py",
    "watch_n15_feedbackpair_001.py",
    "upload_reference22.py",
    "upload_evidence22_data3.py",
)


class Invalid(ValueError):
    """Invalid input; no archive or live store is repaired."""


def require(condition, message):
    if not condition:
        raise Invalid(message)


def relative(name):
    require(isinstance(name, str) and bool(name), "empty/non-string path")
    require(
        not name.startswith("/")
        and "\\" not in name
        and ":" not in name
        and all(x not in ("", ".", "..") for x in name.split("/"))
        and not any(ord(x) < 32 or ord(x) == 127 for x in name),
        "unsafe relative path",
    )
    parts = PurePosixPath(name).parts
    require(
        not set(parts) & {".ssh", ".codex", ".aws", ".credentials", "__pycache__"},
        "forbidden artifact directory",
    )
    require(
        not any(
            x in {"auth.json", ".env", "id_rsa", "id_ed25519"} or x.startswith("account-preflight")
            for x in parts
        ),
        "forbidden artifact path",
    )
    return PurePosixPath(name)


def checked_path(path, directory=False):
    """Check each ancestor before accessing the leaf; no symlinks or special files."""
    path = Path(path).absolute()
    require(".." not in path.parts, "unsafe local path")
    for item in reversed([path] + list(path.parents)):
        mode = item.lstat().st_mode
        require(not stat.S_ISLNK(mode), "symlink refused")
        is_dir = item != path or directory
        require(stat.S_ISDIR(mode) if is_dir else stat.S_ISREG(mode), "wrong file type")
    return path


def safe_path(root, name, directory=False):
    return checked_path(Path(root) / relative(name), directory=directory)


def owned_directory(root, name):
    """Create only ignored .local directories, checking existing parents first."""
    parts = relative(name).parts
    require(parts[0] == ".local", "writable directory must be ignored .local")
    current = checked_path(root, directory=True)
    for part in parts:
        current = current / part
        try:
            current.mkdir()
        except FileExistsError:
            pass
        checked_path(current, directory=True)
    return current


def new_workspace(root, prefix):
    parent = owned_directory(root, ".local/validation")
    return Path(tempfile.mkdtemp(prefix=prefix, dir=str(parent)))


def unique_keys(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, "duplicate JSON key")
        result[key] = value
    return result


def json_bytes(raw):
    return json.loads(raw, object_pairs_hook=unique_keys)


def read_json(path):
    path = checked_path(path)
    require(path.stat().st_size <= MAX_FILE, "JSON too large")
    return json_bytes(path.read_bytes())


def digest_file(path):
    h = hashlib.sha256()
    with checked_path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def valid_hash(value):
    return isinstance(value, str) and HEX.fullmatch(value) is not None


def read_index(root):
    index = read_json(safe_path(root, "archive/index.json"))
    require(isinstance(index, dict), "invalid archive index")
    for field, ceiling in (
        ("bytes", MAX_COMPRESSED),
        ("unpacked_bytes", MAX_BYTES),
        ("members", MAX_MEMBERS),
    ):
        require(
            type(index.get(field)) is int and 0 < index[field] <= ceiling,
            "archive index exceeds limit: " + field,
        )
    for field in ("sha256", "source_manifest_sha256"):
        require(valid_hash(index.get(field)), "invalid archive hash: " + field)
    require(index.get("repo") == "Zarad0X/robotwin-astra-harness", "unexpected release repository")
    require(
        isinstance(index.get("release_tag"), str)
        and re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,100}", index["release_tag"]),
        "invalid release tag",
    )
    asset = index.get("asset_name")
    require(
        isinstance(asset, str) and re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*\.tar\.gz", asset),
        "invalid asset name",
    )
    return index


def validate_blob(path, index):
    path = checked_path(path)
    require(path.stat().st_size == index["bytes"], "archive compressed size mismatch")
    require(digest_file(path) == index["sha256"], "archive digest mismatch")
    return path


def resolve_archive(root, index, override=None):
    path = (
        override
        if override is not None
        else Path(root) / ".local/archive-cache" / index["asset_name"]
    )
    return validate_blob(path, index)


class BoundedReader:
    """Bound even tar metadata/padding expansion, before tarfile allocates it."""

    def __init__(self, stream):
        self.stream = stream
        self.remaining = MAX_STREAM

    def read(self, size):
        require(0 <= size <= self.remaining, "archive expanded stream exceeds limit")
        raw = self.stream.read(size)
        self.remaining -= len(raw)
        return raw


def archive_members(path):
    """Yield validated headers and streaming contents, with independent hard caps."""
    seen = {}
    parent_paths = set()
    total = 0
    with checked_path(path).open("rb") as raw:
        with gzip.GzipFile(fileobj=raw) as expanded:
            with tarfile.open(fileobj=BoundedReader(expanded), mode="r|") as archive:
                for member in archive:
                    name = member.name
                    if member.isdir() and name.endswith("/"):
                        name = name[:-1]
                    relative(name)
                    require(name not in seen, "duplicate archive path")
                    require(len(seen) < MAX_MEMBERS, "archive member ceiling exceeded")
                    require(
                        (member.isfile() or member.isdir()) and not member.issparse(),
                        "archive permits only ordinary files/directories",
                    )
                    require(
                        not any(k.startswith("GNU.sparse") for k in member.pax_headers),
                        "sparse archive refused",
                    )
                    require(
                        type(member.size) is int and 0 <= member.size <= MAX_FILE,
                        "archive per-file ceiling exceeded",
                    )
                    require(not member.isdir() or member.size == 0, "directory has content")
                    for parent in PurePosixPath(name).parents:
                        require(seen.get(str(parent)) != "file", "archive file/directory conflict")
                    if member.isfile():
                        require(name not in parent_paths, "archive file/directory conflict")
                    parent_paths.update(str(p) for p in PurePosixPath(name).parents)
                    seen[name] = "dir" if member.isdir() else "file"
                    total += member.size
                    require(total <= MAX_BYTES, "archive total byte ceiling exceeded")
                    yield name, member, archive.extractfile(member) if member.isfile() else None


def manifest_entries(value, schema):
    require(
        isinstance(value, dict) and value.get("schema") == schema, "source manifest schema mismatch"
    )
    files = value.get("files")
    require(isinstance(files, dict) and bool(files), "empty source manifest")
    for name, entry in files.items():
        relative(name)
        require(
            isinstance(entry, dict)
            and type(entry.get("bytes")) is int
            and 0 <= entry["bytes"] <= MAX_FILE
            and valid_hash(entry.get("sha256")),
            "invalid source manifest entry",
        )
        for field in ("role", "source_path"):
            require(
                isinstance(entry.get(field), str) and bool(entry[field]),
                "missing source provenance",
            )
    return files


def scan_archive(path, index):
    """Hash every file, bind the old source manifest; retain only small metadata."""
    validate_blob(path, index)
    files, metadata = {}, {}
    count = total = 0
    for name, member, stream in archive_members(path):
        count += 1
        total += member.size
        if stream is None:
            continue
        keep = name == SOURCE_MANIFEST or name.endswith("/" + SOURCE_MANIFEST)
        keep = keep or name == SNAPSHOT_INDEX or name.endswith("/" + SNAPSHOT_INDEX)
        chunks = [] if keep else None
        h = hashlib.sha256()
        size = 0
        for block in iter(lambda: stream.read(65536), b""):
            size += len(block)
            require(size <= member.size, "archive member size overflow")
            h.update(block)
            if chunks is not None:
                chunks.append(block)
        require(size == member.size, "truncated archive member")
        files[name] = {"bytes": size, "sha256": h.hexdigest()}
        if keep:
            metadata[name] = b"".join(chunks)
    require(
        count == index["members"] and total == index["unpacked_bytes"], "archive census mismatch"
    )
    manifests = [n for n in metadata if n == SOURCE_MANIFEST or n.endswith("/" + SOURCE_MANIFEST)]
    require(len(manifests) == 1, "expected exactly one original source manifest")
    manifest_name = manifests[0]
    prefix = manifest_name[: -len(SOURCE_MANIFEST)]
    require(
        files[manifest_name]["sha256"] == index["source_manifest_sha256"],
        "source manifest pin mismatch",
    )
    require(all(n.startswith(prefix) for n in files), "files outside archive repository root")
    sources = manifest_entries(json_bytes(metadata[manifest_name]), "harness-source-snapshot/1")
    for name, entry in sources.items():
        require(name != SOURCE_MANIFEST, "source manifest cannot pin itself")
        measured = files.get(prefix + name)
        require(measured is not None, "missing archived source: " + name)
        require(
            all(measured[k] == entry[k] for k in ("bytes", "sha256")),
            "archived source mismatch: " + name,
        )
    return {
        "files": files,
        "metadata": metadata,
        "sources": sources,
        "prefix": prefix,
        "members": count,
        "unpacked_bytes": total,
        "archived_files": len(files),
    }


def extract_selected(path, scan, destination, mapping):
    """Explicit archived repo path -> fresh destination-relative path, no overwrites."""
    checked_path(destination, directory=True)
    targets = set()
    for origin, target in mapping.items():
        relative(origin)
        relative(target)
        require(target not in targets, "duplicate extraction destination")
        targets.add(target)
        require(
            scan["prefix"] + origin in scan["files"], "missing selected archive file: " + origin
        )
    found = set()
    for name, member, stream in archive_members(path):
        origin = name[len(scan["prefix"]) :] if name.startswith(scan["prefix"]) else None
        if origin not in mapping:
            continue
        require(stream is not None, "selected archive member is not regular")
        target = Path(destination) / relative(mapping[origin])
        current = Path(destination)
        for part in relative(mapping[origin]).parts[:-1]:
            current = current / part
            try:
                current.mkdir()
            except FileExistsError:
                pass
            checked_path(current, directory=True)
        h = hashlib.sha256()
        size = 0
        # Exclusive creation rejects existing regular files and dangling symlinks.
        with target.open("xb") as output:
            for block in iter(lambda: stream.read(65536), b""):
                output.write(block)
                h.update(block)
                size += len(block)
        require(
            {"bytes": size, "sha256": h.hexdigest()} == scan["files"][name],
            "archive changed during extraction",
        )
        found.add(origin)
    require(found == set(mapping), "incomplete selective extraction")


def check_active(root):
    files = manifest_entries(read_json(safe_path(root, ACTIVE_MANIFEST)), "harness-active-source/1")
    required = {"harness/" + n for n in CORE_NAMES}
    required |= {"adapters/" + n for n in ADAPTER_NAMES}
    required |= {
        "transport/codex22.py",
        "transport/remote15.py",
        "experience/issues.json",
        "experience/regressions.json",
    }
    required |= {"tests/core/" + n for n in TEST_NAMES}
    require(required <= set(files), "incomplete active source manifest")
    actual = set()
    for folder in ("harness", "adapters", "transport", "tests/core", "experience"):
        base = safe_path(root, folder, directory=True)
        for here, dirs, names in os.walk(str(base), followlinks=False):
            for leaf in dirs + names:
                name = (Path(here) / leaf).relative_to(root).as_posix()
                safe_path(root, name, directory=leaf in dirs)
                if leaf in names:
                    actual.add(name)
    require(actual == set(files), "active manifest must cover exactly the active source trees")
    for name, entry in files.items():
        path = safe_path(root, name)
        require(
            path.stat().st_size == entry["bytes"] and digest_file(path) == entry["sha256"],
            "active source mismatch: " + name,
        )
    return files
