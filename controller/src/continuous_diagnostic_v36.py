"""V36 actual sealed child, rendered by default; explicit headless manifest only."""
import json,os,resource,sys,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'harness'))
from physics_runner import main

def entry():
    # Only the launcher supplies this option; every byte and allocation is checked.
    if '--execution-manifest' not in sys.argv:return main(continuous=True,continuous_version=36)
    i=sys.argv.index('--execution-manifest');path=Path(sys.argv[i+1]).resolve(strict=True);del sys.argv[i:i+2]
    run=path.parent
    from continuous_v36 import validate_manifest
    from headless_resource_v36 import environment,assignment_gate,child_binding
    m=json.loads(path.read_text());validate_manifest(run,m)
    e=m['execution']
    if e['mode']!='headless':raise ValueError('headless_child_mode')
    def arg(n):return Path(sys.argv[sys.argv.index(n)+1]).resolve()
    if arg('--release')!=Path(m['release']) or arg('--output')!=run/'artifacts' or arg('--output-root')!=run or arg('--config')!=run/'config.json':raise ValueError('headless_child_arguments')
    a=assignment_gate(run/'assignment.json',run,m)
    if environment(m['release'],m['headless_environment']['asset_root'])!=m['headless_environment']:raise ValueError('headless_environment_changed')
    resource.setrlimit(resource.RLIMIT_CPU,(e['cpu_seconds'],e['cpu_seconds']))
    os.sched_setaffinity(0,{a['cpu_core']})
    for n in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS','BLIS_NUM_THREADS'):os.environ[n]='1'
    os.environ['CUDA_VISIBLE_DEVICES']='';sys.dont_write_bytecode=True
    from headless_write_boundary import restrict
    from provenance import write_once
    write_once(run/'headless-write-boundary.json',dict(landlock_abi=restrict(run),only_write_root=str(run)))
    from continuous_visual import Visual
    visual=Visual(run/'artifacts',child_binding(run,m),e['max_physics_steps'],e['capture_every_steps'],e['max_frames'],e['admission_only'])
    exception=None
    try:return main(continuous=True,continuous_version=36,visual=visual,loaded_variant=m['loaded_variant'])
    except BaseException as exc:
        exception=dict(type=type(exc).__name__,message=str(exc),traceback=traceback.format_exc());raise
    finally:
        # A logging failure must never replace the original native guard error.
        try:visual.finish(exception)
        except BaseException:
            if exception is None:raise
            traceback.print_exc()

if __name__=='__main__':raise SystemExit(entry())
