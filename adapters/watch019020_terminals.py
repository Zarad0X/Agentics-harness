"""Finite collector and one postmortem dispatch; never launches/restarts physics."""

from pathlib import Path
import hashlib, json, os, subprocess, sys, time

ROOT = Path(__file__).resolve().parent
RUNS = {f"rt-v36-combined-original10-{v}": v for v in ("019", "020")}
FOLDER = ROOT / "auto-audit019020-001"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
COLLECT = ROOT / "collect_live_combined012_15.py"
paths = [
    COLLECT,
    ROOT / "parent_adapter.py",
    ROOT / "evolution.py",
    ROOT / "dispatch22_data3.py",
    ROOT / "usage_accounting.py",
    ROOT / "append_completed_usage.py",
    ROOT / "completed_usage_source.py",
    ROOT / "account22.py",
    ROOT / "upload_reference22.py",
    ROOT.parent / "astra-direct/remote15.py",
    ROOT.parents[1] / "outputs/codex22.py",
]
for name, v in RUNS.items():
    launch = ROOT / (name + "-launch.json")
    review = ROOT / f"parent-review-original10-{v}.json"
    l = json.loads(launch.read_text())
    r = json.loads(review.read_text())
    assert (
        l["runtime_variant"] == "v36-live-combined-" + v
        and l["runtime_binding_sha256"] == r["runtime_binding_sha256"]
    )
    paths.extend(
        [launch, review, ROOT / ("v36-live-combined-" + v) / "runtime/RUNTIME_BINDING.json"]
    )
pins = {str(p): sha(p) for p in paths}
FOLDER.mkdir()
plan = dict(
    runs=RUNS,
    source_sha256=pins,
    observer_pid=os.getpid(),
    started_unix=time.time(),
    wall_limit_s=266000,
    poll_s=60,
    physics_actions=0,
    postmortem_dispatch_limit=1,
)
(FOLDER / "plan.json").write_text(json.dumps(plan, indent=2) + "\n")
status = {n: dict(state="waiting") for n in RUNS}


def emit(kind, **extra):
    row = dict(kind=kind, observed_unix=time.time(), **extra)
    with (FOLDER / "events.jsonl").open("a") as f:
        f.write(json.dumps(row) + "\n")
    tmp = FOLDER / "status.tmp"
    tmp.write_text(json.dumps(status, indent=2) + "\n")
    tmp.replace(FOLDER / "status.json")
    print(json.dumps(row), flush=True)


probe = r"""
from pathlib import Path
import json,sys
root=Path('/data_ft_6/shaojing/robotwin-astra-roundtrip-20261009/runs');rows=[]
for name in json.load(sys.stdin):
 r=root/name;row=dict(run_id=name)
 for n in ('terminal.json','startup-failure.json','child.json'):
  if (r/n).is_file():row[n]=json.loads((r/n).read_text())
 if 'child.json' in row:
  c=row['child.json']
  try:s=Path('/proc',str(c['pid']),'stat').read_text().rsplit(') ',1)[1].split()
  except FileNotFoundError:s=None
  row['alive_same_start']=s is not None and s[19]==str(c['start_ticks']) and s[0] not in ('Z','X')
 rows.append(row)
print(json.dumps(rows))
"""
emit("observer_started", plan=plan)
deadline = time.monotonic() + plan["wall_limit_s"]
while time.monotonic() < deadline and any(x["state"] == "waiting" for x in status.values()):
    changed = [path for path, h in pins.items() if not Path(path).is_file() or sha(Path(path)) != h]
    if changed:
        for n, s in status.items():
            if s["state"] == "waiting":
                status[n] = dict(
                    state="needs_review", reason="pinned_coordinator_input_changed", paths=changed
                )
        emit("pinned_input_changed", paths=changed)
        break
    try:
        names = [n for n, s in status.items() if s["state"] == "waiting"]
        r = subprocess.run(
            [sys.executable, str(ROOT.parent / "astra-direct/remote15.py"), "python3", "-c", probe],
            input=json.dumps(names),
            capture_output=True,
            text=True,
            timeout=90,
        )
        assert r.returncode == 0, "remote observation unavailable"
        observations = json.loads(r.stdout)
    except (OSError, ValueError, AssertionError, subprocess.TimeoutExpired) as e:
        emit("observation_unavailable", reason=str(e))
        time.sleep(60)
        continue
    for row in observations:
        n = row["run_id"]
        v = RUNS[n]
        if row.get("startup-failure.json") and "child.json" not in row:
            status[n] = dict(state="needs_review", reason="startup_failure", evidence=row)
            emit("needs_review", run_id=n)
            continue
        if "terminal.json" not in row or row.get("alive_same_start"):
            continue
        if "child.json" not in row:
            status[n] = dict(state="needs_review", reason="missing_child_identity")
            emit("needs_review", run_id=n)
            continue
        status[n] = dict(state="auditing", terminal=row["terminal.json"])
        emit("audit_started", run_id=n)
        try:
            command = [
                sys.executable,
                str(COLLECT),
                n,
                "--variant",
                "v36-live-combined-" + v,
                "--review",
                str(ROOT / f"parent-review-original10-{v}.json"),
            ]
            with (FOLDER / (n + ".log")).open("x") as log:
                rc = subprocess.run(
                    command, stdout=log, stderr=subprocess.STDOUT, timeout=86660
                ).returncode
            path = ROOT / (n + "-functional-audit-001.json")
            assert rc == 0 and path.is_file(), (
                "collector incomplete; inspect existing audit process/result before any retry"
            )
            report = json.loads(path.read_text())
            assert report["runtime_variant"] == "v36-live-combined-" + v
            status[n] = dict(
                state="audited",
                report=str(path),
                sha256=sha(path),
                valid_functional_receipts=report.get("valid_functional_receipts"),
                whole_episode_success=report.get("whole_episode_success"),
                errors=report.get("errors"),
            )
            emit("audit_finished", run_id=n, outcome=status[n])
        except (OSError, ValueError, AssertionError, subprocess.TimeoutExpired) as e:
            status[n] = dict(state="needs_review", reason=str(e))
            emit("needs_review", run_id=n, reason=str(e))
    if any(x["state"] == "waiting" for x in status.values()):
        time.sleep(60)
if all(x["state"] == "audited" for x in status.values()):
    try:
        for path, h in pins.items():
            assert sha(Path(path)) == h, "pinned coordinator input changed"
        task = "postmortem019020-001"
        prompt = FOLDER / "postmortem.md"
        text = (
            "Use official server22 ChatGPT gpt-6-astra. Read project AGENTS and the injected persistent experience. Compare all final native019/020 attempts from reference/"
            + ", reference/".join(n + "-functional-audit-001.json" for n in RUNS)
            + ". Verify exact binding/config and all terminal/receipt limitations; invalid audit counts remain unknown. No native/SSH/GPU/additional model launches, no changes to sealed runtimes. In a fresh verifieddata3 harness/postmortem019020-001 directory, retain source-bound observations, confirmed causes vs hypotheses, regression gaps, existing lessons reproduced or contradicted, and the next concrete repair candidate if justified. Preserve allOUT then originalID/position/rawQuaternionRETURN and all physics/forces/budgets. No success/stable claims from best trial or CPU tests. Export append-only new issue/observation records and explicit objects for parent review/import, no authority mutation. CPU tests on one currently idle leased CPU only, aggregate<=180CPU, child<=45CPU/75wall, retain failed attempts. Seal bounded<=12MiB DELIVERY/HANDOFF/REPORT and proposed patch/regressions if useful. Parent owns review and next native launch."
        )
        prompt.write_text(text + "\n")
        emit("postmortem_dispatch_started", task=task)
        with (FOLDER / "postmortem.log").open("x") as log:
            rc = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "dispatch22_data3.py"),
                    task,
                    str(prompt),
                    "--experience-stage",
                    "full-episode",
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                timeout=14400,
            ).returncode
        with (FOLDER / "pending-parent-review.json").open("x") as f:
            json.dump(
                dict(
                    task=task,
                    dispatch_exit_code=rc,
                    delegation=str(ROOT / "delegations" / task),
                    expected_server22_packet="/data3/yuantao/astra-roundtrip-work/harness/postmortem019020-001",
                    new_issue_records_not_yet_imported=True,
                    next_native_requires_parent_review=True,
                    automatic_native_launches=0,
                ),
                f,
                indent=2,
            )
        emit("postmortem_dispatch_finished", exit_code=rc, task=task, review_required=True)
    except (OSError, ValueError, AssertionError, subprocess.TimeoutExpired) as e:
        emit("postmortem_needs_review", reason=str(e))
emit(
    "observer_finished",
    unresolved=[n for n, s in status.items() if s["state"] != "audited"],
    goal_success_not_inferred=True,
)
