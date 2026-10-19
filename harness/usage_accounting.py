"""Transactional updates to the EXISTING roundtrip-development-usage/1 ledger.

No alternate cumulative ledger. Resolver is the local completed_source authority,
not an imported command. Unknown history never implies a zero predecessor.
"""

import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
from contextlib import contextmanager

FIELDS = (
    "input_tokens",
    "cached_input_tokens",
    "cache_write_input_tokens",
    "output_tokens",
    "reasoning_output_tokens",
)


class AccountingError(ValueError):
    pass


def canonical(x):
    return json.dumps(x, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def digest(x):
    return hashlib.sha256(canonical(x)).hexdigest()


def require(x, why):
    if not x:
        raise AccountingError(why)


def counters(x):
    require(isinstance(x, dict) and set(x) == set(FIELDS), "incomplete usage counters")
    require(all(type(v) is int and v >= 0 for v in x.values()), "invalid usage counters")
    require(x["cached_input_tokens"] <= x["input_tokens"], "cached input exceeds input")
    require(x["reasoning_output_tokens"] <= x["output_tokens"], "reasoning exceeds output")
    return x


def validate(ledger):
    require(ledger.get("schema") == "roundtrip-development-usage/1", "unknown authority schema")
    names = set()
    events = set()
    last = {}
    offsets = {}
    for r in ledger["rows"]:
        require(r["delegation"] not in names, "duplicate delegation in authority")
        names.add(r["delegation"])
        sid = r["session_id"]
        raw = counters(r["raw_session_cumulative"])
        inc = counters(r["incremental"])
        require(
            r["uncached_input_tokens"] == inc["input_tokens"] - inc["cached_input_tokens"],
            "uncached mismatch",
        )
        if sid in last:
            require(
                {k: raw[k] - last[sid][k] for k in FIELDS} == inc,
                "authority session delta mismatch",
            )
        else:
            # Historical first-row offsets are authoritative, not reset to zero.
            offset = {k: raw[k] - inc[k] for k in FIELDS}
            require(all(v >= 0 for v in offset.values()), "invalid historical offset")
            offsets[sid] = offset
        if r.get("event_id"):
            require(r["event_id"] not in events, "duplicate completion event in authority")
            events.add(r["event_id"])
        last[sid] = raw
    require(
        ledger["total_incremental"]
        == {
            k: sum(r["incremental"].get(k, 0) for r in ledger["rows"])
            for k in ledger["total_incremental"]
        },
        "authority totals mismatch",
    )
    return offsets


@contextmanager
def locked(path):
    path = Path(path)
    with path.with_name(path.name + ".lock").open("a") as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        yield


def read(path):
    raw = Path(path).read_bytes()
    ledger = json.loads(raw)
    validate(ledger)
    return raw, ledger


def atomic(path, raw):
    path = Path(path)
    fd, tmp = tempfile.mkstemp(prefix=".usage-", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(raw)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
        fd = os.open(str(path.parent), os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def session_origin(path, resume):
    if not resume:
        return dict(kind="new_session")
    with locked(path):
        _, ledger = read(path)
        prior = [r for r in ledger["rows"] if r["session_id"] == resume]
    if not prior:
        return dict(kind="unknown_history", session_id=resume)
    return dict(
        kind="resumed_session",
        session_id=resume,
        predecessor_delegation=prior[-1]["delegation"],
        predecessor_cumulative_sha256=digest(prior[-1]["raw_session_cumulative"]),
    )


def event_identity(name, status, evidence):
    event = status.get("completed_event") or {}
    if event:
        require(
            event.get("session_id") == status["session_id"]
            and event.get("usage") == status["usage"],
            "completed event/status mismatch",
        )
        require(
            re.fullmatch("[0-9a-f]{64}", event.get("line_sha256", "")) is not None,
            "missing completed event identity",
        )
        require(
            type(event.get("ordinal")) is int and event["ordinal"] >= 1,
            "missing completed event ordinal",
        )
        return digest(
            dict(
                session_id=status["session_id"],
                line_sha256=event["line_sha256"],
                ordinal=event["ordinal"],
            )
        )
    # Historical/recovered authority may predate direct event capture. Scope is explicit.
    return digest(
        dict(
            delegation=name,
            session_id=status["session_id"],
            raw_session_cumulative=status["usage"],
            identity_scope="completed_source/historical-completion",
        )
    )


def selected_receipt(raw, ledger, name):
    """Return a source-linked row; the evolution adapter revalidates the full authority."""
    matches = [r for r in ledger["rows"] if r["delegation"] == name]
    require(len(matches) == 1, "missing completed authority row")
    row = matches[0]
    event_id = row.get("event_id") or digest(
        dict(
            delegation=name,
            session_id=row["session_id"],
            raw_session_cumulative=row["raw_session_cumulative"],
            identity_scope="completed_source/historical-completion",
        )
    )
    return dict(
        schema="authoritative-development-increment/1",
        delegation=name,
        session_id=row["session_id"],
        event_id=event_id,
        authority_sha256=hashlib.sha256(raw).hexdigest(),
        row_sha256=digest(row),
        row=row,
        incremental=row["incremental"],
        completed_source=row.get("completed_source") or row.get("completion_evidence"),
        provenance_scope="existing authoritative row; missing historical event details remain unknown"
        if not row.get("completed_source")
        else "completed_source-resolved event",
    )


def read_receipt(path, name):
    with locked(path):
        raw, ledger = read(path)
        return selected_receipt(raw, ledger, name)


def append_completed(path, names, resolver):
    """Serialize resolve/read/delta/append/replace. A batch failure changes no rows."""
    path = Path(path)
    with locked(path):
        before, ledger = read(path)
        rows = ledger["rows"]
        initial = len(rows)
        for name in names:
            require(
                re.fullmatch("[A-Za-z0-9][A-Za-z0-9_.-]*", name) is not None, "invalid delegation"
            )
            status, evidence = resolver(name)
            raw = counters(status["usage"])
            sid = status["session_id"]
            require(
                status.get("completed") is True or evidence is not None,
                "completion authority missing",
            )
            event_id = event_identity(name, status, evidence)
            existing = [r for r in rows if r["delegation"] == name]
            if existing:
                r = existing[0]
                require(
                    r["session_id"] == sid and r["raw_session_cumulative"] == raw,
                    "duplicate delegation changed counters/session",
                )
                require(
                    not r.get("event_id") or r["event_id"] == event_id,
                    "duplicate delegation changed event",
                )
                continue
            require(
                not any(r.get("event_id") == event_id for r in rows),
                "completion event already assigned to another delegation",
            )
            prior = [r for r in rows if r["session_id"] == sid]
            origin = status.get("usage_origin") or {}
            if prior:
                prev = prior[-1]["raw_session_cumulative"]
                if origin:
                    require(
                        origin.get("kind") == "resumed_session"
                        and origin.get("session_id") == sid
                        and origin.get("predecessor_delegation") == prior[-1]["delegation"]
                        and origin.get("predecessor_cumulative_sha256") == digest(prev),
                        "unknown/stale predecessor; reconcile accounting order",
                    )
            else:
                require(
                    origin.get("kind") == "new_session", "unknown prior history; no zero assumed"
                )
                prev = {k: 0 for k in FIELDS}
            delta = {k: raw[k] - prev[k] for k in FIELDS}
            counters(delta)
            row = dict(
                delegation=name,
                session_id=sid,
                account="server22_chatgpt",
                model="gpt-6-astra",
                raw_session_cumulative=raw,
                incremental=delta,
                uncached_input_tokens=delta["input_tokens"] - delta["cached_input_tokens"],
                scope="shared development cost; not per generated episode",
                event_id=event_id,
                completed_source=evidence,
                usage_origin=origin
                or dict(
                    kind="existing_authoritative_predecessor",
                    predecessor_delegation=prior[-1]["delegation"],
                ),
            )
            if evidence is not None:
                row["completion_evidence"] = evidence
            rows.append(row)
        ledger["total_incremental"] = {
            k: sum(r["incremental"].get(k, 0) for r in rows) for k in ledger["total_incremental"]
        }
        validate(ledger)
        if len(rows) != initial:
            atomic(path, json.dumps(ledger, indent=2, allow_nan=False).encode() + b"\n")
        after = path.read_bytes()
        return dict(
            total_incremental=ledger["total_incremental"],
            added=len(rows) - initial,
            receipts=[selected_receipt(after, ledger, n) for n in names],
            authority_sha256=hashlib.sha256(after).hexdigest(),
        )
