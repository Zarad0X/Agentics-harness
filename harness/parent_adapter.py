"""Local controller boundary only. Never imports remote/controller/auth modules."""

import json
from pathlib import Path
from evolution import Store, Invalid, CONTRACT, read_allowed, sha, digest, encoded, brief, assess


def prepare_task(root, name, prompt, stage, signature=""):
    store = Store(root)
    if not store.issues() or not store.records("regression"):
        raise Invalid("initialize durable ledger/registry from reviewed delivery before dispatch")
    result = brief(store, stage, signature, max_tokens=600)
    # Keep the brief/identity, never ingest a model/credential event log.
    store.register(
        dict(
            id="task:" + name,
            kind="task",
            objects=[],
            stage=stage,
            prompt_sha256=sha(prompt.encode()),
            brief=result,
            model="gpt-6-astra",
            account="server22_chatgpt",
        )
    )
    record_outcome(root, "dispatch:" + name, "unknown", "task_pending", domain="orchestration")
    return prompt + "\n\n<engineering_evidence_data>\n" + result[
        "text"
    ] + "\n</engineering_evidence_data>\n", result


def record_outcome(root, attempt_id, outcome, failure, usage=None, domain="transport", wall_s=None):
    """Legacy scalar boundary. Optional usage is UNVERIFIED, never authority credit.

    Patched dispatch uses finish_dispatch/import_usage for development accounting.
    """
    from evolution import adapt

    value = dict(
        schema="evolution-outcome/1",
        attempt_id=attempt_id,
        domain=domain,
        outcome=outcome,
        failure=failure,
        usage=usage,
        wall_s=wall_s,
        scope=dict(
            split="development",
            model="gpt-6-astra",
            stage="dispatch",
            code=None,
            config=None,
            assistance=None,
        ),
        retry_count=None,
    )
    raw = encoded(value)
    key = sha(raw)
    store = Store(root)
    observation = adapt(value, {})
    observation["accounting_status"] = "unverified_legacy" if usage is not None else "unknown"
    return store.append(
        dict(
            id="evidence:" + key,
            kind="observation",
            evidence=key,
            origin="controller-scalar-outcome",
            objects=[key],
            observation=observation,
        ),
        [raw],
    )


def collect(root, run_id, report=None, terminal_state="missing_terminal_data"):
    if report is None:
        return record_outcome(
            root, "collection:" + run_id, "unknown", terminal_state, domain="orchestration"
        )
    raw = encoded(report)
    key = sha(raw)
    store = Store(root)
    from evolution import adapt

    observation = adapt(report, {"run_id": run_id})
    if observation["attempt_id"] != run_id:
        raise Invalid("collector run identity mismatch")
    return store.append(
        dict(
            id="evidence:" + key,
            kind="observation",
            objects=[key],
            evidence=key,
            origin=run_id + "-functional-audit-001.json",
            observation=observation,
        ),
        [raw],
    )


def record_launch(root, run_id, binding_path, config_path):
    """Record the launch request before remote work; missing collection stays unknown."""
    from evolution import run_scope

    binding = read_allowed(binding_path)
    config = read_allowed(config_path)
    cfg = json.loads(config)
    scope = run_scope(run_id, cfg)
    if any(cfg.get(k) != scope[k] for k in ("count", "layout", "seed", "split")):
        raise Invalid("launch intent/config mismatch")
    scope.update(
        code=sha(binding),
        config=sha(config),
        metadata_origin="local sealed launch intent; native start unconfirmed",
    )
    raw = encoded(dict(schema="evolution-launch-intent/1", run_id=run_id, scope=scope))
    key = sha(raw)
    observation = dict(
        attempt_id=run_id,
        strength=0,
        evidence_type="launch-intent",
        scope=scope,
        domain="physical",
        outcome="unknown",
        failure="missing_terminal_data",
        terminal_available=False,
        full_success=False,
        out=None,
        exact_return=None,
        preview_count=None,
        preview_steps=None,
        wall_s=None,
        tokens=None,
        retry_count=None,
        failure_distribution={},
    )
    return Store(root).append(
        dict(
            id="launch:" + run_id,
            kind="observation",
            objects=[key, sha(binding), sha(config)],
            evidence=key,
            origin="local-launch-request",
            observation=observation,
        ),
        [raw, binding, config],
    )


def gate(root, candidate_path, review_path, binding_path, config_path, adoption=False):
    """Recompute assessment at launch time against exact deployment/config bytes."""
    store = Store(root)
    candidate = json.loads(read_allowed(candidate_path))
    review = json.loads(read_allowed(review_path))
    binding = read_allowed(binding_path)
    config = read_allowed(config_path)
    if candidate.get("source_sha256") != sha(binding) or candidate.get("config_sha256") != sha(
        config
    ):
        raise Invalid("actual source/config differs from candidate")
    if candidate.get("baseline_config_sha256") != review.get("baseline_config_sha256"):
        raise Invalid("parent baseline config review mismatch")
    assessment = assess(store, candidate, review)
    raw = encoded(assessment)
    key = sha(raw)
    store.append(
        dict(id="assessment:" + key, kind="assessment", objects=[key], assessment=assessment), [raw]
    )
    if not assessment["diagnostic_launch_eligible"] or adoption:
        raise Invalid(
            "candidate blocked: "
            + (
                "; ".join(assessment["errors"])
                if not adoption
                else "independent native adoption gate remains required"
            )
        )
    return assessment


def import_usage(root, authority_path, name):
    """Import verified increments from the existing authority, never raw CLI usage."""
    from usage_accounting import validate, selected_receipt
    from evolution import token_usage

    raw = read_allowed(authority_path)
    ledger = json.loads(raw)
    validate(ledger)
    receipt = selected_receipt(raw, ledger, name)
    row_sha = receipt["row_sha256"]
    event = receipt["event_id"]
    store = Store(root)
    identity = "usage:" + event
    for r in store.records("observation"):
        if r["id"] == identity:
            if r.get("usage_row_sha256") != row_sha:
                raise Invalid("completion event changed authoritative row")
            return dict(imported=False, id=identity)
    blob = encoded(receipt)
    key = sha(blob)
    row = receipt["row"]
    observation = dict(
        attempt_id="dispatch:" + name,
        strength=4,
        evidence_type="authoritative-completed-increment",
        scope=dict(
            split="development",
            model=row["model"],
            stage="dispatch",
            code=None,
            config=None,
            assistance=None,
        ),
        domain="orchestration",
        outcome="completed",
        failure=None,
        full_success=False,
        terminal_available=False,
        out=None,
        exact_return=None,
        wall_s=None,
        tokens=token_usage(receipt["incremental"]),
        preview_count=None,
        preview_steps=None,
        retry_count=None,
        failure_distribution={},
        accounting_status="verified",
        usage_provenance=receipt,
    )
    payload = dict(
        id=identity,
        kind="observation",
        objects=[sha(raw), key],
        evidence=key,
        origin="development-usage-ledger.json",
        usage_row_sha256=row_sha,
        observation=observation,
    )
    try:
        return store.append(payload, [raw, blob])
    except Invalid:
        # Another completion can grow the authority between equivalent concurrent imports.
        same = next((r for r in store.records("observation") if r["id"] == identity), None)
        if same and same.get("usage_row_sha256") == row_sha:
            return dict(imported=False, id=identity)
        raise


def completion_observed(root, name, session_id, event):
    """Model completion is recorded BEFORE the independently fallible accounting step."""
    if not session_id or not event or event.get("session_id") != session_id:
        raise Invalid("completion identity missing")
    data = dict(session_id=session_id, line_sha256=event["line_sha256"], ordinal=event["ordinal"])
    key = sha(encoded(data))
    store = Store(root)
    observation = dict(
        attempt_id="dispatch:" + name,
        strength=2,
        evidence_type="observed-completion",
        scope=dict(
            split="development",
            model="gpt-6-astra",
            stage="dispatch",
            code=None,
            config=None,
            assistance=None,
        ),
        domain="orchestration",
        outcome="completed",
        failure=None,
        full_success=False,
        terminal_available=False,
        out=None,
        exact_return=None,
        wall_s=None,
        tokens=None,
        preview_count=None,
        preview_steps=None,
        retry_count=None,
        failure_distribution={},
        accounting_status="unknown",
        completion_event=data,
    )
    return store.append(
        dict(
            id="completion:" + name + ":" + key,
            kind="observation",
            objects=[key],
            evidence=key,
            origin="observed-turn-completed",
            observation=observation,
        ),
        [encoded(data)],
    )


def accounting_unknown(root, name, session_id, event):
    # Keep the actual completion and a separate accounting incident, not a model failure.
    completion_observed(root, name, session_id, event)
    return record_outcome(
        root, "accounting:" + name, "unknown", "accounting_unknown", domain="orchestration"
    )


def finish_dispatch(root, parent_root, name, session_id, event, append_names):
    """Called after local status.json is durable. No auth/event logs are read here."""
    completion_observed(root, name, session_id, event)
    try:
        append_names([name])
        result = import_usage(root, Path(parent_root) / "development-usage-ledger.json", name)
        return dict(model_completed=True, accounting_status="verified", receipt=result)
    except (Exception,):
        accounting_unknown(root, name, session_id, event)
        return dict(model_completed=True, accounting_status="unknown", reason="accounting_unknown")
