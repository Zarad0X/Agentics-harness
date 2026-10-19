"""Independent structural/source binding check; never a physical acceptance gate."""
import hashlib,json
from pathlib import Path
from runtime_profile import CONTRACT,encode,contract_hash

def inspect(run):
 run=Path(run);errors=[];receipt=None
 try:
  manifest_raw=(run/'diagnostic-manifest.json').read_bytes();m=json.loads(manifest_raw)
  if m.get('runtime_variant')!='v36-live-combined-021' or m.get('profiling')!=CONTRACT:raise ValueError('profile_manifest')
  root=Path(__file__).resolve().parents[1];bp=root/'harness/configs/v36/BINDINGS.json';sealed=json.loads(bp.read_bytes())
  expected_sources=dict(sealed['runtime_dependencies']);expected_sources['harness/configs/v36/BINDINGS.json']=hashlib.sha256(bp.read_bytes()).hexdigest()
  if m['snapshot_sha256']!=expected_sources:raise ValueError('profile_reviewed_sources')
  for n,h in m['snapshot_sha256'].items():
   p=run/'snapshot'/n
   if not p.resolve().is_relative_to((run/'snapshot').resolve()) or hashlib.sha256(p.read_bytes()).hexdigest()!=h:raise ValueError('profile_snapshot_binding')
  if hashlib.sha256((run/'config.json').read_bytes()).hexdigest()!=m['config_sha256']:raise ValueError('profile_config_binding')
  p=run/'artifacts/runtime-profile015.json'
  if p.stat().st_size>CONTRACT['max_bytes']:raise ValueError('profile_size')
  raw=p.read_bytes();receipt=json.loads(raw)
  if raw!=encode(receipt):raise ValueError('profile_canonical')
  expected=dict(run_id=run.name,manifest_sha256=hashlib.sha256(manifest_raw).hexdigest(),runtime_variant=m['runtime_variant'],snapshot_sha256=m['snapshot_sha256'],config_sha256=m['config_sha256'])
  if receipt['binding']!=expected or receipt['contract']!=CONTRACT or receipt['contract_sha256']!=contract_hash() or receipt['schema']!=CONTRACT['schema'] or receipt['native_qualified'] is not False:raise ValueError('profile_binding')
  reasons=receipt['incomplete_reasons']
  if reasons!=sorted(set(reasons)) or not set(reasons)<=set(('clock','capacity','context','stack','io','binding','internal','unfinished','in_progress')) or receipt['complete'] is not (receipt['final'] and not reasons):raise ValueError('profile_completeness')
  if type(receipt['final']) is not bool or type(receipt['open_frames']) is not int or not 0<=receipt['open_frames']<=CONTRACT['max_stack'] or (not receipt['final'])!=('in_progress' in reasons) or (receipt['complete'] and receipt['open_frames']):raise ValueError('profile_finality')
  def bucket(b):return isinstance(b,list) and len(b)==4 and b[0] in CONTRACT['phases'] and b[1] in ('formal','preview') and b[2] in ('out','return') and b[3] in ('none','left','right')
  def uint(v):return type(v) is int and v>=0
  keys=[]
  if len(receipt['rows'])>CONTRACT['max_rows'] or len(receipt['coverage'])>60:raise ValueError('profile_capacity')
  for r in receipt['rows']:
   if r['label'] not in CONTRACT['labels'] or not bucket(r['bucket']):raise ValueError('profile_label')
   keys.append((r['label'],*r['bucket']))
   for k in ('calls','sampled','exceptions','wall_ns','cpu_ns','exclusive_wall_ns','exclusive_cpu_ns','max_wall_ns','max_cpu_ns'):
    if not uint(r[k]):raise ValueError('profile_number')
   if r['sampled']>r['calls'] or r['exceptions']>r['calls']:raise ValueError('profile_counts')
   for c in ('wall','cpu'):
    if not 0<=r['exclusive_'+c+'_ns']<=r[c+'_ns'] or r['max_'+c+'_ns']>r[c+'_ns']:raise ValueError('profile_clock')
   if not r['sampled'] and any(r[k] for k in ('wall_ns','cpu_ns','exclusive_wall_ns','exclusive_cpu_ns','max_wall_ns','max_cpu_ns')):raise ValueError('profile_unsampled')
  if len(keys)!=len(set(keys)):raise ValueError('profile_duplicate')
  seen=[]
  for c in receipt['coverage']:
   if not bucket(c['bucket']) or not all(uint(c[k]) for k in ('ticks','sampled_ticks','unsampled_ticks')) or c['ticks']!=c['sampled_ticks']+c['unsampled_ticks']:raise ValueError('profile_coverage')
   seen.append(tuple(c['bucket']))
   ticks=[r for r in receipt['rows'] if r['label']=='tick' and r['bucket']==c['bucket']]
   if receipt['complete'] and (len(ticks)!=1 or ticks[0]['calls']!=c['ticks'] or ticks[0]['sampled']!=c['sampled_ticks']):raise ValueError('profile_tick_coverage')
  if len(seen)!=len(set(seen)):raise ValueError('profile_coverage_duplicate')
 except Exception as exc:errors.append(type(exc).__name__+':'+str(exc))
 return dict(schema='runtime-profile-audit/015',valid=not errors,complete=not errors and receipt['complete'],errors=errors,receipt=receipt,physical_success_inferred=False,scope='Binding/structure only; timings are self-observations, not independently measured CPU or physical evidence')
