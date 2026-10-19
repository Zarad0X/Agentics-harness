"""Dispatch a scoped Astra task on server 22; allow the verified data3 work directory and retain coordination metadata."""

import argparse
from datetime import datetime, timezone
import json
import hashlib
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parents[1] / "outputs"))
import codex22
from parent_adapter import (
    prepare_task,
    record_outcome,
    completion_observed,
    accounting_unknown,
    finish_dispatch,
)
from append_completed_usage import append_names
from usage_accounting import session_origin

_ACTIVE_TASK = None
_COMPLETION = None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("prompt", type=Path)
    ap.add_argument("--resume")
    ap.add_argument("--experience-stage", required=True)
    ap.add_argument("--experience-signature", default="")
    ap.add_argument("--harness-store", type=Path, default=ROOT / "evolution-ledger")
    args = ap.parse_args()
    if not args.name.replace("-", "").isalnum():
        raise ValueError("invalid task name")
    logdir = ROOT / "delegations" / args.name
    logdir.mkdir(parents=True, exist_ok=False)
    global _ACTIVE_TASK, _COMPLETION
    _ACTIVE_TASK = (args.harness_store, args.name)
    prompt = args.prompt.read_text()
    prompt, experience = prepare_task(
        args.harness_store, args.name, prompt, args.experience_stage, args.experience_signature
    )
    (logdir / "experience-brief.json").write_text(json.dumps(experience, indent=2))
    (logdir / "prompt.md").write_text(prompt)
    check = subprocess.run(
        [sys.executable, str(ROOT / "account22.py")], capture_output=True, text=True, timeout=150
    )
    if check.returncode:
        raise SystemExit(
            "Server22 official ChatGPT account/allowance preflight failed; no model task started."
        )
    account = json.loads(check.stdout)
    if (
        not account.get("official_provider_verified")
        or account.get("account", {}).get("type") != "chatgpt"
    ):
        raise SystemExit("Server22 official ChatGPT provider not verified; no model task started.")
    (logdir / "account-preflight.json").write_text(json.dumps(account, indent=2))
    codex22.ensure_proxy()
    cwd = "/data4/yuantao/astra-roundtrip-20261009"
    command = [
        "env",
        "-u",
        "OPENAI_API_KEY",
        "-u",
        "CODEX_API_KEY",
        "-u",
        "OPENAI_BASE_URL",
        codex22.REMOTE_CODEX,
        "exec",
        "-C",
        cwd,
        "--add-dir",
        "/data3/yuantao/astra-roundtrip-work",
        "-c",
        'model_provider="openai"',
        "-c",
        'model_reasoning_effort="high"',
        "--model",
        "gpt-6-astra",
    ]
    if args.resume:
        if not codex22.UUID.fullmatch(args.resume):
            raise ValueError("invalid session id")
        known = codex22.load_sessions().get(args.resume)
        if not known or known.get("cwd") != cwd:
            raise ValueError("session not registered to this project")
        command += [
            "resume",
            "--json",
            "--skip-git-repo-check",
            "-c",
            'sandbox_mode="workspace-write"',
            args.resume,
            "-",
        ]
    else:
        command += ["--json", "--skip-git-repo-check", "--sandbox", "workspace-write", "-"]
    meta = dict(
        name=args.name,
        cwd=cwd,
        model="gpt-6-astra",
        session_id=args.resume,
        usage_origin=session_origin(ROOT / "development-usage-ledger.json", args.resume),
        account="server22_chatgpt",
        started_at=datetime.now(timezone.utc).isoformat(),
    )
    (logdir / "status.json").write_text(json.dumps(meta, indent=2))
    proc = subprocess.Popen(
        codex22.ssh_base() + [codex22.remote_command(command)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1,
    )
    proc.stdin.write(prompt)
    proc.stdin.close()
    completed = False
    completion_ordinal = 0
    with (logdir / "events.jsonl").open("x") as log:
        for line in proc.stdout:
            try:
                event = json.loads(line)
            except ValueError:
                continue
            kind = event.get("type")
            if kind == "thread.started":
                meta["session_id"] = event["thread_id"]
                sessions = codex22.load_sessions()
                sessions[event["thread_id"]] = dict(
                    cwd=cwd,
                    sandbox="workspace-write",
                    model="gpt-6-astra",
                    created_at=meta["started_at"],
                )
                codex22.save_sessions(sessions)
            if kind == "item.completed" and event.get("item", {}).get("type") == "agent_message":
                answer = event["item"].get("text", "")
                (logdir / "answer.md").write_text(answer)
                print(answer, flush=True)
            if kind == "turn.completed":
                completed = True
                meta["usage"] = event.get("usage", {})
                completion_ordinal += 1
                meta["completed_event"] = dict(
                    session_id=meta["session_id"],
                    usage=meta["usage"],
                    line_sha256=hashlib.sha256(line.encode()).hexdigest(),
                    ordinal=completion_ordinal,
                )
                _COMPLETION = (meta["session_id"], meta["completed_event"])
                completion_observed(args.harness_store, args.name, *_COMPLETION)
            if kind in ("thread.started", "turn.completed", "turn.failed", "error"):
                log.write(json.dumps(event, ensure_ascii=False) + "\n")
                log.flush()
                print(json.dumps(event, ensure_ascii=False), flush=True)
                (logdir / "status.json").write_text(json.dumps(meta, indent=2))
    meta.update(
        ssh_exit=proc.wait(), completed=completed, ended_at=datetime.now(timezone.utc).isoformat()
    )
    (logdir / "status.json").write_text(json.dumps(meta, indent=2))
    if completed:
        accounting = finish_dispatch(
            args.harness_store, ROOT, args.name, *_COMPLETION, append_names
        )
        (logdir / "accounting-result.json").write_text(json.dumps(accounting, indent=2))
    else:
        record_outcome(
            args.harness_store,
            "dispatch:" + args.name,
            "unknown",
            "dispatch_incomplete",
            domain="orchestration",
        )
        accounting = dict(accounting_status="unknown")
    _ACTIVE_TASK = None
    raise SystemExit(
        0
        if completed and meta["ssh_exit"] == 0 and accounting["accounting_status"] == "verified"
        else 2
        if completed
        else 1
    )


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        if _ACTIVE_TASK is not None:
            ledger, name = _ACTIVE_TASK
            if _COMPLETION is not None:
                accounting_unknown(ledger, name, *_COMPLETION)
            else:
                record_outcome(
                    ledger,
                    "dispatch:" + name,
                    "unknown",
                    "dispatch_interrupted_or_preflight_failed",
                    domain="orchestration",
                )
        raise
