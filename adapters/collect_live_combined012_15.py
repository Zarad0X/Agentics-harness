"""Independently audit a terminal combined continuous run on a separately leased core from its bound snapshot."""

from pathlib import Path
import argparse, hashlib, json, re, subprocess, sys
from parent_adapter import collect

ROOT = Path(__file__).resolve().parent
ap = argparse.ArgumentParser()
ap.add_argument("run_id")
ap.add_argument("--variant", required=True)
ap.add_argument("--review", type=Path, required=True)
ap.add_argument("--dry-run", action="store_true")
ap.add_argument("--harness-store", type=Path, default=ROOT / "evolution-ledger")
a = ap.parse_args()
collect(a.harness_store, a.run_id, None, terminal_state="collection_pending")
assert re.fullmatch(r"rt-v36-combined-(original10|n(?:10|15|20)-s(?:101|103))-[0-9]{3}", a.run_id)
assert re.fullmatch(r"v36-live-combined-[0-9]{3}", a.variant)
version = "36"
runtime = ROOT / a.variant / "runtime"
seal = runtime / "RUNTIME_BINDING.json"
review = json.loads(a.review.read_text())
assert review["native_diagnostic_launch_allowed"] is True
assert hashlib.sha256(seal.read_bytes()).hexdigest() == review["runtime_binding_sha256"]
b = json.loads(seal.read_text())
assert b["runtime_variant"] == a.variant
bindings = b["files_sha256"]
launch = json.loads((ROOT / (a.run_id + "-launch.json")).read_text())
assert (
    launch["runtime_variant"] == a.variant
    and launch["runtime_binding_sha256"] == review["runtime_binding_sha256"]
)
script = "\nfrom pathlib import Path\nimport hashlib,json,os,subprocess,sys,fcntl,time\np=json.load(sys.stdin);root=Path('/data_ft_6/shaojing/robotwin-astra-roundtrip-20261009')\nrun=root/'runs'/p['run_id'];m=json.loads((run/'diagnostic-manifest.json').read_text())\nassert m['schema']=='continuous-diagnostic-supervisor/'+p['version'] and m['execution']['mode']=='headless'\nassert m['runtime_variant']==p['variant']\nassert m['execution']['admission_only'] is False,'Admission cannot be audited as a full physical episode'\nfor n,h in m['source_sha256'].items():assert p['bindings']['src/'+n]==h\nfor n,h in m['snapshot_sha256'].items():\n assert p['bindings'][n]==h\n assert hashlib.sha256((run/'snapshot'/n).read_bytes()).hexdigest()==h\nchild=json.loads((run/'child.json').read_text());stat=Path('/proc',str(child['pid']),'stat')\ntry:f=stat.read_text().split()\nexcept FileNotFoundError:f=None\nalive=f is not None and f[21]==child['start_ticks'] and f[2] not in ('Z','X')\nterminal_path=run/'terminal.json';terminal=json.loads(terminal_path.read_text()) if terminal_path.exists() else None\nobservation=dict(run_id=run.name,child_alive_same_start=alive,terminal=terminal,source_verified=True,physics_actions=0,audit_executed=False)\nif p['dry_run'] or alive or terminal is None:\n observation['state']='ready_for_audit' if not alive and terminal is not None else 'waiting_for_terminal'\n print(json.dumps(observation));raise SystemExit(0)\nassert terminal['state']=='terminal'\nout=root/'runs'/(run.name+'-functional-audit-001.json')\nif not out.exists():\n py='/home/shaojing/HanZ/tools/miniforge3/envs/robotwin/bin/python'\n command=[py,'-B',str(run/'snapshot/harness'/('continuous_audit_v'+p['version']+'.py')),str(run),'--output',str(out)]\n audit_tmp=root/'runs'/(run.name+'-audit-scratch-001')\n assert root.resolve()==root\n assert subprocess.check_output(['findmnt','-n','-T',str(root),'-o','TARGET'],text=True).strip()=='/data_ft_6'\n assert audit_tmp.resolve()==audit_tmp and not audit_tmp.is_symlink()\n audit_tmp.mkdir(exist_ok=True)\n env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',TMPDIR=str(audit_tmp),SQLITE_TMPDIR=str(audit_tmp))\n # The audit uses a separately leased currently idle CPU, not the live run core.\n def counters():\n  rows={}\n  for line in Path('/proc/stat').read_text().splitlines():\n   v=line.split()\n   if v and v[0].startswith('cpu') and v[0][3:].isdigit():\n    x=list(map(int,v[1:]));rows[int(v[0][3:])]=(sum(x[:8]),x[3]+x[4])\n  return rows\n audit_lock=(root/'runs'/(run.name+'-functional-audit-001.lock')).open('a')\n fcntl.flock(audit_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)\n assert not out.exists(),'audit completed concurrently; read result on next observation'\n before=counters();time.sleep(1);after=counters();choices=[]\n for cpu in os.sched_getaffinity(0)-{127}:\n  total=after[cpu][0]-before[cpu][0];idle=after[cpu][1]-before[cpu][1]\n  if total and idle/total>=.85:choices.append((idle/total,cpu))\n lease=None\n for ratio,cpu in sorted(choices,reverse=True):\n  f=(root/'runs'/('cpu-%d.lock'%cpu)).open('a')\n  try:fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)\n  except BlockingIOError:f.close();continue\n  lease=f;break\n assert lease is not None,'No free audit CPU; native run not restarted'\n command=['/usr/bin/taskset','-c',str(cpu),'/usr/bin/nice','-n','10']+command\n result=subprocess.run(command,env=env,capture_output=True,text=True,timeout=86400,pass_fds=(lease.fileno(),audit_lock.fileno()))\n log=root/'runs'/(run.name+'-functional-audit-001.execution.json')\n with log.open('x') as stream:json.dump(dict(command=command,exit_code=result.returncode,stdout=result.stdout,stderr=result.stderr),stream,indent=2)\n assert result.returncode in (0,2) and out.is_file(),result.stderr[-2000:]\nassert out.stat().st_size<=16*1024**2,'Report too large for bounded metadata relay'\nreport=json.loads(out.read_text());assert report['schema']=='continuous-functional/'+p['version']\nobservation.update(state='audited',audit_executed=True,report=report,report_sha256=hashlib.sha256(out.read_bytes()).hexdigest())\nprint(json.dumps(observation))\n"
proc = subprocess.run(
    [sys.executable, str(ROOT.parent / "astra-direct/remote15.py"), "python3", "-c", script],
    input=json.dumps(
        dict(
            run_id=a.run_id,
            version=version,
            variant=a.variant,
            bindings=bindings,
            dry_run=a.dry_run,
        )
    ),
    capture_output=True,
    text=True,
    timeout=86600,
)
if proc.returncode != 0:
    collect(a.harness_store, a.run_id, None, terminal_state="collection_transport_or_audit_failure")
    raise SystemExit("Collection failed; pending outcome retained")
result = json.loads(proc.stdout)
if "report" not in result:
    collect(
        a.harness_store, a.run_id, None, terminal_state=result.get("state", "missing_terminal_data")
    )
    print(json.dumps(result))
    raise SystemExit(0)
report = result.pop("report")
collect(a.harness_store, a.run_id, report)
path = ROOT / (a.run_id + "-functional-audit-001.json")
if path.exists():
    assert json.loads(path.read_text()) == report
else:
    with path.open("x") as f:
        json.dump(report, f, indent=2)
sent = subprocess.run(
    [sys.executable, str(ROOT / "upload_reference22.py"), str(path)],
    capture_output=True,
    text=True,
    timeout=180,
)
assert sent.returncode == 0, sent.stderr[-1200:]
verification = report.get("independent_verification") or {}
result.update(
    {
        k: report.get(k)
        for k in (
            "errors",
            "valid_functional_receipts",
            "raw_continuous_completed",
            "functional_gate_passed",
            "whole_episode_success",
            "formal_credit",
        )
    }
)
result["counts"] = {k: verification.get(k) for k in ("extracted_count", "exact_return_count")}
result["raw_failure"] = report.get("raw", {}).get("result", {}).get("failure")
print(json.dumps(result, indent=2))
