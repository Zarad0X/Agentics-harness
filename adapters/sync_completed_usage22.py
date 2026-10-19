"""Import verified completed cumulative usage; preserve append-only receipts."""

from pathlib import Path
import json, subprocess, sys
from completed_usage_source import completed_source

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parents[1] / "outputs"))
import codex22


def completion_evidence_for_sync(row, status, current_evidence):
    """Sync authority receipts without rewriting older completion provenance."""
    assert status["session_id"] == row["session_id"]
    assert status["usage"] == row["raw_session_cumulative"]
    stored = row.get("completion_evidence")
    if current_evidence is not None:
        if current_evidence.get("kind") == "completed_dispatch_status":
            assert status.get("completed") is True
            assert status.get("ssh_exit") == 0 or status.get("completed_event")
            if stored is not None:
                for key in (
                    "kind",
                    "status_sha256",
                    "session_id",
                    "completed_event",
                    "transport_completion",
                ):
                    assert stored.get(key) == current_evidence.get(key), key
        else:
            assert stored == current_evidence
    return stored


rows = json.loads((ROOT / "development-usage-ledger.json").read_text())["rows"]
payload = []
for name in sys.argv[1:]:
    row = next(r for r in rows if r["delegation"] == name)
    status, recovered_evidence = completed_source(name)
    assert status["usage"] == row["raw_session_cumulative"]
    prior = [r for r in rows[: rows.index(row)] if r["session_id"] == row["session_id"]]
    previous = prior[-1]["delegation"] + "-final" if prior else None
    payload.append(
        dict(
            source="turn.completed",
            counter_kind="session_cumulative",
            session_id=row["session_id"],
            event_id=name + "-final",
            previous_event_id=previous,
            new_session=not prior,
            **row["raw_session_cumulative"],
        )
    )
    # Preserve the real source when normalizing a recovered completed turn to
    # the ledger's completed-turn vocabulary; never claim SSH finished normally.
    evidence = completion_evidence_for_sync(row, status, recovered_evidence)
    if evidence is not None:
        payload[-1]["completion_evidence"] = evidence
script = r"""
from pathlib import Path
import sys,json
root=Path('/data4/yuantao/astra-roundtrip-20261009')
sys.path[:0]=[str(root/'src'),str(root/'harness')]
from usage_ledger import ingest,totals
ledger=root/'harness/development-usage';inputs=root/'harness/development-usage-inputs';inputs.mkdir(exist_ok=True)
for raw in json.loads(sys.stdin.read()):
 p=inputs/(raw['event_id']+'.json');encoded=json.dumps(raw,sort_keys=True,indent=2)+'\n'
 if p.exists():assert json.loads(p.read_text())==raw
 else:p.write_text(encoded)
 receipt=ledger/(raw['event_id']+'.json')
 if receipt.exists():assert json.loads(receipt.read_text())['raw']==raw
 else:ingest(ledger,p)
print(json.dumps(totals(ledger)))
"""
codex22.ensure_proxy()
r = subprocess.run(
    codex22.ssh_base() + [codex22.remote_command(["python3", "-c", script])],
    input=json.dumps(payload),
    capture_output=True,
    text=True,
    timeout=90,
)
print(r.stdout or r.stderr)
raise SystemExit(r.returncode)
