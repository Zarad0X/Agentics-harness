"""Launch one fresh full episode using an existing byte-verified deployment; no runtime upload."""

from pathlib import Path
import argparse, base64, hashlib, json, re, subprocess, sys
from parent_adapter import gate, record_launch

ROOT = Path(__file__).resolve().parent
ap = argparse.ArgumentParser()
ap.add_argument("variant")
ap.add_argument("run_id")
ap.add_argument("--review", type=Path, required=True)
ap.add_argument("--count", type=int, choices=[10, 15, 20], default=10)
ap.add_argument("--layout", choices=["original", "generated"], default="original")
ap.add_argument("--seed", type=int, default=2)
ap.add_argument("--harness-store", type=Path, default=ROOT / "evolution-ledger")
ap.add_argument("--candidate", type=Path, required=True)
ap.add_argument("--regression-review", type=Path, required=True)
a = ap.parse_args()
assert re.fullmatch(r"v36-live-combined-[0-9]{3}", a.variant)
assert re.fullmatch(r"rt-v36-combined-(original10|n(?:10|15|20)-s(?:101|103))-[0-9]{3}", a.run_id)
packet = ROOT / a.variant
runtime = packet / "runtime"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
review = json.loads(a.review.read_text())
binding = json.loads((runtime / "RUNTIME_BINDING.json").read_text())
assert review["native_diagnostic_launch_allowed"] is True and review[
    "runtime_binding_sha256"
] == sha(runtime / "RUNTIME_BINDING.json")
assert binding["runtime_variant"] == a.variant and binding["native_qualified"] is False
files = {}
for n, h in dict(
    binding["files_sha256"], **{"RUNTIME_BINDING.json": sha(runtime / "RUNTIME_BINDING.json")}
).items():
    path = runtime / n
    assert runtime.resolve() in path.resolve().parents and sha(path) == h, n
    files[n] = {"sha256": h}
assert sum((runtime / n).stat().st_size for n in files) < 5 * 1024**2
config_name = (
    "continuous-original10-headroom06-k6-reserved-entry-copytrace17.json"
    if a.layout == "original"
    else f"continuous-generated-n{a.count}-s{a.seed}-entry.json"
)
regression_assessment = gate(
    a.harness_store,
    a.candidate,
    a.regression_review,
    runtime / "RUNTIME_BINDING.json",
    runtime / "harness/configs/v36" / config_name,
)
native = json.loads((ROOT / "parent-offline-renderer15-nativeOFF-001.json").read_text())["results"][
    0
]
assert native["exit_code"] == 0 and native["native_binary_equivalence"] is True
p = dict(
    variant=a.variant,
    run_id=a.run_id,
    count=a.count,
    layout=a.layout,
    seed=a.seed,
    admission_only=False,
    timeout_s=86400,
    cpu_seconds=86400,
    max_physics_steps=10000000,
    capture_every_steps=25,
    max_frames=1000000,
    runtime_files=files,
    runtime_binding_sha256=sha(runtime / "RUNTIME_BINDING.json"),
    review_sha256=sha(a.review),
    release="/data_ft_6/shaojing/robotwin-astra-20261008/release",
    engine_binary_hashes={
        Path(n).name: h for n, h in native["validation"]["engine_binary_hashes"].items()
    },
)
for k in ("source", "asset"):
    p[k] = json.loads((ROOT.parent / "astra-direct" / (k + "22-manifest.json")).read_text())
script = "\nfrom pathlib import Path\nimport os,json,hashlib,subprocess,shutil,sys,time,socket,fcntl\np=json.load(sys.stdin);root=Path('/data_ft_6/shaojing/robotwin-astra-roundtrip-20261009');release=Path('/data_ft_6/shaojing/robotwin-astra-20261008/release');assets=Path('/data_ft_8/shaojing/HanZ/RoboTwin/handoff-20260911-validation/work-v21-r3/base/assets/objects')\nassert str(release)==p['release']\ncode=root/'versions'/('implementation-'+p['variant']);run=root/'runs'/p['run_id']\nassert not run.exists(),'preserve existing run'\nfor path,mount in [(root,'/data_ft_6'),(release,'/data_ft_6'),(assets,'/data_ft_8')]:\n assert path.resolve()==path\n assert subprocess.check_output(['findmnt','-n','-T',str(path),'-o','TARGET'],text=True).strip()==mount\nassert (release/'workspace/base/assets/objects').resolve()==assets\nassert shutil.disk_usage(root).free>512*1024**3\nmemory={line.split(':')[0]:int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines()}\nassert memory['MemAvailable']>8*1024**2,'insufficient currently available RAM'\nsha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()\nassert not run.exists(),'preserve existing run'\nassert code.is_dir() and code.resolve()==code,'reviewed runtime must already be deployed'\nfor n,row in p['runtime_files'].items():\n rel=Path(n);assert not rel.is_absolute() and '..' not in rel.parts\n dest=code/rel\n assert dest.is_file() and not dest.is_symlink() and code in dest.resolve().parents\n assert sha(dest)==row['sha256'],'existing deployment differs: '+n\nassert sha(code/'RUNTIME_BINDING.json')==p['runtime_binding_sha256']\nb=json.loads((code/'RUNTIME_BINDING.json').read_text())\nassert b['runtime_variant']==p['variant'] and b['native_qualified'] is False\nfor n,h in b['files_sha256'].items():assert sha(code/n)==h,n\nfor key,base in [('source',release),('asset',assets)]:\n for n,h in p[key].items():\n  rel=Path(n);assert not rel.is_absolute() and '..' not in rel.parts\n  file=base/rel;assert base in file.resolve().parents and sha(file)==h,n\npy='/home/shaojing/HanZ/tools/miniforge3/envs/robotwin/bin/python'\nenv=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONUNBUFFERED='1',CUDA_VISIBLE_DEVICES='',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')\nenv['PATH']=str(Path(py).parent)+':'+env['PATH']\nconfig_name='continuous-original10-headroom06-k6-reserved-entry-copytrace17.json' if p['layout']=='original' else f\"continuous-generated-n{p['count']}-s{p['seed']}-entry.json\"\nconfig=code/'harness/configs/v36'/config_name;cfg=json.loads(config.read_text())\nassert cfg['count']==p['count'] and cfg['seed']==p['seed'] and cfg['layout']==p['layout'] and cfg['split']=='development'\nassert not cfg.get('home_cpu_profile',False),'This launcher admits only independently reviewed profiling-OFF experiments'\nassert cfg.get('sleep_copy_trace_object_id')==(17 if p['layout']=='original' else None)\ndef cpus():\n result={}\n for line in Path('/proc/stat').read_text().splitlines():\n  f=line.split()\n  if f and f[0].startswith('cpu') and f[0][3:].isdigit():\n   v=list(map(int,f[1:]));result[int(f[0][3:])]=(sum(v[:8]),v[3]+v[4])\n return result\nfirst=cpus();time.sleep(1);second=cpus();available=os.sched_getaffinity(0);idle=[]\nfor core in sorted(available-{127,124,123,122,121,119,117,115,114}):\n total=second[core][0]-first[core][0];free=second[core][1]-first[core][1]\n if total>0 and free/total>=.8:idle.append((free/total,core))\nassert idle,'no sufficiently idle CPU in current sample'\nchosen=None\nfor ratio,core in sorted(idle,reverse=True):\n probe=(root/'runs'/('cpu-'+str(core)+'.lock')).open('a')\n try:fcntl.flock(probe,fcntl.LOCK_EX|fcntl.LOCK_NB)\n except BlockingIOError:probe.close();continue\n fcntl.flock(probe,fcntl.LOCK_UN);probe.close();chosen=core;break\nassert chosen is not None,'all candidate project CPU cores locked'\ncmd=[py,'-B',str(code/'harness/continuous_v36.py'),'prepare','--root',str(root/'runs'),'--run-id',p['run_id'],'--config',str(config),'--release',str(release),'--python',py,'--headless','--asset-root',str(assets),'--timeout',str(p['timeout_s']),'--cpu-seconds',str(p['cpu_seconds']),'--max-physics-steps',str(p['max_physics_steps']),'--capture-every-steps',str(p['capture_every_steps']),'--max-frames',str(p['max_frames'])]\nif p['admission_only']:cmd.append('--admission-only')\nprepared=subprocess.run(cmd,env=env,capture_output=True,text=True,timeout=120)\nassert prepared.returncode==0,prepared.stderr[-3000:]\nmanifest=run/'diagnostic-manifest.json';m=json.loads(manifest.read_text());now=time.time()\nassert m['runtime_variant']==p['variant'] and m['loaded_variant']=='B_clearance' and not m['execution']['admission_only']\nassert m['headless_environment']['engine_binary_hashes']==p['engine_binary_hashes'],'unqualified native binary change'\ncache=run/'runtime-cache';cache.mkdir()\nfor name in ('tmp','mpl','torch','cuda','xdg'):(cache/name).mkdir()\nenv.update(TMPDIR=str(cache/'tmp'),MPLCONFIGDIR=str(cache/'mpl'),TORCH_HOME=str(cache/'torch'),CUDA_CACHE_PATH=str(cache/'cuda'),XDG_CACHE_HOME=str(cache/'xdg'))\nwith (run/'parent-cache-environment.json').open('x') as f:json.dump({k:env[k] for k in ('TMPDIR','MPLCONFIGDIR','TORCH_HOME','CUDA_CACHE_PATH','XDG_CACHE_HOME')},f,indent=2)\nassignment=dict(schema='headless-cpu-assignment/36',run_id=p['run_id'],host=socket.gethostname(),output_root=str(root/'runs'),manifest_sha256=sha(manifest),execution=m['execution'],cpu_core=chosen,issued_unix=now,expires_unix=now+1800,parent_reviewed=True,native_authorized=True,data_disk_verified=True,assets_verified=True)\nap=root/(p['run_id']+'-assignment.json')\nwith ap.open('x') as f:json.dump(assignment,f,indent=2)\nwith (root/(p['run_id']+'-supervisor.log')).open('x') as log:\n child=subprocess.Popen([py,'-B',str(run/'snapshot/harness/continuous_v36.py'),'launch',str(run),'--assignment',str(ap)],env=env,cwd=code,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)\n start_ticks=Path('/proc',str(child.pid),'stat').read_text().split()[21]\nreceipt=dict(run_id=p['run_id'],host='15',supervisor_pid=child.pid,supervisor_start_ticks=start_ticks,cpu_core=chosen,code=str(code),config=str(config),launched_unix=time.time(),admission_only=p['admission_only'],execution=m['execution'],continuous_diagnostic=True,reconstructed=False,formal_credit=False,timeout_s=p['timeout_s'],count=p['count'],seed=p['seed'],layout=p['layout'],runtime_variant=p['variant'],runtime_binding_sha256=p['runtime_binding_sha256'],parent_review_sha256=p['review_sha256'])\nwith (root/(p['run_id']+'-launch.json')).open('x') as f:json.dump(receipt,f,indent=2)\nprint(json.dumps(receipt))\n"

record_launch(
    a.harness_store,
    a.run_id,
    runtime / "RUNTIME_BINDING.json",
    runtime / "harness/configs/v36" / config_name,
)
r = subprocess.run(
    [sys.executable, str(ROOT.parent / "astra-direct/remote15.py"), "python3", "-c", script],
    input=json.dumps(p),
    capture_output=True,
    text=True,
    timeout=300,
)
if r.returncode:
    raise SystemExit(
        "Launch unconfirmed; inspect exact version/run/assignment/supervisor before retry: "
        + r.stdout[-1000:]
        + r.stderr[-2200:]
    )
receipt = json.loads(r.stdout)
with (ROOT / (a.run_id + "-launch.json")).open("x") as f:
    json.dump(receipt, f, indent=2)
print(json.dumps(receipt))
