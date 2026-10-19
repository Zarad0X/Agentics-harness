"""Explicit finite headless allocation; no native import or device query."""
import importlib.util,importlib.metadata,json,math,os,socket,time
from pathlib import Path
from provenance import file_hash,digest
from continuous_budget_v36 import validate_timeout
ROOT=Path(__file__).resolve().parents[1]
ASSET=Path('workspace/base/assets/objects')

def integer(value,lo,hi,label):
    if type(value) is not int or not lo<=value<=hi:raise ValueError(label)
    return value

def execution(headless,timeout,cpu=None,steps=None,cadence=None,frames=None,admission=False):
    if type(headless) is not bool or type(admission) is not bool:raise ValueError('execution_mode_type')
    if not headless:
        if admission or any(v is not None for v in (cpu,steps,cadence,frames)):raise ValueError('rendered_headless_options')
        return dict(mode='rendered',admission_only=False)
    cpu=integer(cpu,1,timeout,'headless_cpu_budget')
    steps=integer(steps,1,10000000,'headless_step_budget')
    cadence=integer(25 if cadence is None else cadence,1,250,'headless_capture_cadence')
    frames=integer(frames,2,1000000,'headless_frame_budget')
    if frames<(steps+cadence-1)//cadence+2:raise ValueError('frame_allocation_below_step_cadence')
    if admission and (cpu>120 or timeout>180 or steps>20000):raise ValueError('admission_budget')
    return dict(mode='headless',admission_only=admission,cpu_seconds=cpu,max_physics_steps=steps,capture_every_steps=cadence,max_frames=frames)

def validate_execution(m,count):
    timeout=validate_timeout(count,m.get('timeout_s'));e=m.get('execution')
    if not isinstance(e,dict) or e.get('mode') not in ('rendered','headless'):raise ValueError('execution_contract')
    expected=execution(e['mode']=='headless',timeout,e.get('cpu_seconds'),e.get('max_physics_steps'),e.get('capture_every_steps'),e.get('max_frames'),e.get('admission_only'))
    if e!=expected:raise ValueError('execution_contract_fields')
    if e['mode']=='headless':
        if m.get('gpu') is not None or not isinstance(m.get('headless_environment'),dict):raise ValueError('headless_resource_identity')
    elif type(m.get('gpu')) is not int or m['gpu']<0 or m.get('headless_environment') is not None:raise ValueError('rendered_resource_identity')
    return e

def environment(release,asset_root):
    release=Path(release).resolve(strict=True);assets=Path(asset_root).resolve(strict=True)
    if not assets.is_dir() or (release/ASSET).resolve(strict=True)!=assets:raise ValueError('asset_root_anchor')
    mapping=json.loads((ROOT/'src/fixtures/headless-release-bindings.json').read_text())
    for n,h in mapping.items():
        name=Path(n)
        if name.is_absolute() or '..' in name.parts:raise ValueError('unsafe_release_binding')
        p=(release/name).resolve(strict=True);base=assets if name.is_relative_to(ASSET) else release
        if not p.is_relative_to(base) or file_hash(p)!=h:raise ValueError('headless_release_binding:'+n)
    spec=importlib.util.find_spec('sapien')
    if spec is None or not spec.origin:raise ValueError('missing_sapien_distribution')
    package=Path(spec.origin).parent.resolve(strict=True)
    wrappers=json.loads((ROOT/'src/fixtures/headless-wrapper-bindings.json').read_text())
    for n,h in wrappers.items():
        p=(package/n).resolve(strict=True)
        if not p.is_relative_to(package) or file_hash(p)!=h:raise ValueError('headless_wrapper_binding:'+n)
    version=importlib.metadata.version('sapien')
    if version!='3.0.0b1':raise ValueError('headless_native_version')
    binaries={p.name:file_hash(p) for p in sorted(package.glob('*.so'))}
    if not binaries:raise ValueError('missing_native_binary')
    return dict(asset_root=str(assets),release_bindings_sha256=digest(mapping),wrapper_bindings_sha256=digest(wrappers),engine_version=version,engine_binary_hashes=binaries)

def validate_assignment(a,run,m,now=None,host=None):
    e=m['execution'];now=time.time() if now is None else now;host=socket.gethostname() if host is None else host
    required={'schema','run_id','host','output_root','manifest_sha256','execution','cpu_core','issued_unix','expires_unix','parent_reviewed','native_authorized','data_disk_verified','assets_verified'}
    if set(a)!=required or a['schema']!='headless-cpu-assignment/36':raise ValueError('CPU_assignment_schema')
    if (a['run_id']!=run.name or a['host']!=host or Path(a['output_root']).resolve()!=run.parent
        or a['manifest_sha256']!=file_hash(run/'diagnostic-manifest.json') or a['execution']!=e
        or any(a[k] is not True for k in ('parent_reviewed','native_authorized','data_disk_verified','assets_verified'))):raise ValueError('CPU_assignment_binding')
    integer(a['cpu_core'],0,1048575,'CPU_core')
    issued,expires=a['issued_unix'],a['expires_unix']
    if any(type(v) not in (float,int) or not math.isfinite(v) for v in (issued,expires)) or not issued<=now<expires or expires-issued>3600:raise ValueError('CPU_assignment_TTL')
    return a

def assignment_gate(path,run,m):
    a=validate_assignment(json.loads(Path(path).read_text()),run,m)
    if a['cpu_core'] not in os.sched_getaffinity(0):raise ValueError('CPU_core_unavailable')
    return a

def child_binding(run,m):
    return dict(schema='headless-executed-runtime/36',manifest_sha256=file_hash(run/'diagnostic-manifest.json'),implementation_sha256=digest(m['source_sha256']),source_sha256=m['source_sha256'],execution=m['execution'],environment=m['headless_environment'])
