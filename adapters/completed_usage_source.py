"""Resolve completed usage without rewriting a disconnected dispatch receipt."""

from pathlib import Path
import hashlib, json

ROOT = Path(__file__).resolve().parent


def completed_source(name):
    status_path = ROOT / "delegations" / name / "status.json"
    status_bytes = status_path.read_bytes()
    status = json.loads(status_bytes)
    event = status.get("completed_event")
    if status.get("completed") and (status.get("ssh_exit") == 0 or event):
        if event:
            assert event["session_id"] == status["session_id"] and event["usage"] == status["usage"]
            assert isinstance(event["line_sha256"], str) and len(event["line_sha256"]) == 64
            assert type(event["ordinal"]) is int and event["ordinal"] >= 1
        evidence = dict(
            kind="completed_dispatch_status",
            status_sha256=hashlib.sha256(status_bytes).hexdigest(),
            session_id=status["session_id"],
            completed_event=event,
            transport_completion="observed" if status.get("ssh_exit") == 0 else "not_observed",
        )
        return status, evidence
    if name == "repair016-native-capture-001":
        # SSH output was lost, but the exact server session completed after
        # this delegation began. Keep that distinction in the audit trail.
        path = ROOT / "repair016-native-capture-authoritative-completion-001.json"
        receipt = json.loads(path.read_text())
        assert receipt["original_delegation"] == name
        assert (
            receipt["session_id"] == status["session_id"]
            and receipt["remote_task_completed"] is True
        )
        source = ROOT / receipt["inspection_path"]
        assert (
            source.parent == ROOT
            and hashlib.sha256(source.read_bytes()).hexdigest() == receipt["inspection_sha256"]
        )
        inspected = json.loads(source.read_text())
        assert not inspected["processes"]
        sessions = [r for r in inspected["sessions"] if r["session_id"] == status["session_id"]]
        assert len(sessions) == 1 and len(sessions[0]["files"]) == 1
        events = sessions[0]["files"][0]["events"]
        assert (
            events[-1]["type"] == "task_complete"
            and events[-1]["timestamp"] == receipt["completion_timestamp"]
        )
        from datetime import datetime

        assert datetime.fromisoformat(
            events[-1]["timestamp"].replace("Z", "+00:00")
        ) >= datetime.fromisoformat(status["started_at"])
        raw = next(e["usage"] for e in reversed(events) if e["type"] == "token_count")
        fields = (
            "input_tokens",
            "cached_input_tokens",
            "cache_write_input_tokens",
            "output_tokens",
            "reasoning_output_tokens",
        )
        usage = {k: raw[k] for k in fields}
        assert usage == receipt["cumulative_usage"]
        assert all(type(v) is int and v >= 0 for v in usage.values())
        assert (
            usage["cached_input_tokens"] <= usage["input_tokens"]
            and usage["reasoning_output_tokens"] <= usage["output_tokens"]
        )
        evidence = dict(
            kind="remote_session_task_complete",
            transport_completion="not_observed",
            completion_timestamp=events[-1]["timestamp"],
            receipt_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            inspection_sha256=receipt["inspection_sha256"],
            original_dispatch_status_preserved=True,
        )
        return dict(status, usage=usage), evidence
    # This exceptional recovery is bound to the exact inspected server session.
    if name != "full-encoder-recovery-001":
        raise ValueError("No independently recovered completion for " + name)
    path = ROOT / "full-encoder-recovery-authoritative-completion-001.json"
    receipt = json.loads(path.read_text())
    assert (
        receipt["original_delegation"] == name
        and receipt["original_ssh_exit"] == status["ssh_exit"] == 255
    )
    assert (
        receipt["remote_task_completed"] is True and receipt["session_id"] == status["session_id"]
    )
    source = ROOT / receipt["inspection_path"]
    assert (
        source.parent == ROOT
        and hashlib.sha256(source.read_bytes()).hexdigest() == receipt["inspection_sha256"]
    )
    inspected = json.loads(source.read_text())
    sessions = [r for r in inspected["sessions"] if r["session_id"] == status["session_id"]]
    assert len(sessions) == 1 and len(sessions[0]["files"]) == 1
    row = sessions[0]["files"][0]
    events = row["events"]
    assert events[-1]["type"] == "task_complete"
    usage = next(e["usage"] for e in reversed(events) if e["type"] == "token_count")
    assert usage == receipt["cumulative_usage"]
    fields = (
        "input_tokens",
        "cached_input_tokens",
        "cache_write_input_tokens",
        "output_tokens",
        "reasoning_output_tokens",
    )
    usage = {k: usage[k] for k in fields}
    assert all(type(v) is int and v >= 0 for v in usage.values())
    assert (
        usage["cached_input_tokens"] <= usage["input_tokens"]
        and usage["reasoning_output_tokens"] <= usage["output_tokens"]
    )
    evidence = dict(
        kind="remote_session_task_complete",
        original_ssh_exit=255,
        completion_timestamp=events[-1]["timestamp"],
        receipt_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        inspection_sha256=receipt["inspection_sha256"],
        original_dispatch_status_preserved=True,
    )
    return dict(status, usage=usage), evidence
