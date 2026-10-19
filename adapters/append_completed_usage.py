"""Serialize the existing completed-usage authority without changing its schema."""

from pathlib import Path
import json, sys
from completed_usage_source import completed_source
from usage_accounting import append_completed

ROOT = Path(__file__).resolve().parent


def append_names(names):
    return append_completed(ROOT / "development-usage-ledger.json", names, completed_source)


if __name__ == "__main__":
    print(json.dumps(append_names(sys.argv[1:])["total_incremental"]))
