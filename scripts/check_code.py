#!/usr/bin/env python3
"""Compile and check whitespace only in new maintained Python; no execution."""

from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    paths = sorted((root / "scripts").glob("*.py"))
    paths += sorted((root / "tests/scaffold").glob("*.py"))
    for path in paths:
        raw = path.read_bytes()
        source = raw.decode("utf-8")
        compile(source, str(path), "exec")
        for number, line in enumerate(source.splitlines(), 1):
            if "\t" in line or line.rstrip() != line:
                raise SystemExit("whitespace violation: %s:%d" % (path.name, number))
        if not source.endswith("\n"):
            raise SystemExit("missing final newline: " + path.name)
    print("Maintained source syntax/whitespace passed: %d files; no legacy formatting" % len(paths))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
