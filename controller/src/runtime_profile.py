"""015 optional aggregate measurement. No scene access; never an acceptance gate."""
import functools
import hashlib
import json
import sys
import time
from pathlib import Path

LABELS=('phase','tick','state','contacts','contact_guard','event','trace','preview','forecast','grasp','plan','placement','unloaded','geometry_point','geometry_certificate','tracking','monitor','home_metadata','copy','witness','controls','canonical','digest','tape_append','tape_flush','tape_data_io','tape_index_io','science','json','write','flush','gzip','lzma','fsync','scene_step','qf','live_before','live_pre','live_observe','placement_tick','continuation','visual')
PHASES=('initialization','extraction','return','final_settle','other')
CONTRACT=dict(schema='runtime-profile/015',enabled=True,labels=list(LABELS),phases=list(PHASES),window_ticks=128,period_ticks=1024,max_rows=1024,max_stack=64,max_bytes=1048576,scope='whole outer calls; nested tick work sampled first128 of each1024 per phase/mode/return/arm bucket; counters reset sampling cursor at each preview',clock='perf_counter_ns/process_time_ns',native_qualified=False,checkpoint='atomic bounded sidecar replacement after preview/phase/unloaded boundary; final on main unwind')
ACTIVE=None

def encode(value):return (json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()
def contract_hash():return hashlib.sha256(encode(CONTRACT)).hexdigest()

class Profile:
 def __init__(self,binding=None,enabled=True,wall=time.perf_counter_ns,cpu=time.process_time_ns):
  self.binding=binding or {};self.enabled=enabled;self.wall=wall;self.cpu=cpu;self.context=lambda:('other',False,False)
  self.rows={};self.coverage={};self.cursors={};self.stack=[];self.arm='none';self.tick_selected=None;self.incomplete=set();self.closed=False
 def problem(self,reason):
  # Fixed error vocabulary; no exception strings or dynamic keys retained.
  self.incomplete.add(reason if reason in ('clock','capacity','context','stack','io','binding','internal','unfinished') else 'internal')
 def bucket(self):
  phase,preview,returning=self.context()
  return (phase if phase in PHASES else 'other','preview' if preview else 'formal','return' if returning else 'out',self.arm)
 def enter(self,label,arm=None):
  if not self.enabled:return None
  if label not in LABELS: self.problem('internal');return None
  previous_arm=self.arm
  if arm in ('left','right'):self.arm=arm
  bucket=self.bucket()
  if label in ('preview','forecast'):bucket=(bucket[0],'preview',bucket[2],bucket[3])
  key=(label,)+bucket
  previous_tick=self.tick_selected
  if label=='preview':self.cursors[bucket]=0
  if label=='tick':
   n=self.cursors.get(bucket,0);self.cursors[bucket]=n+1
   self.tick_selected=n%CONTRACT['period_ticks']<CONTRACT['window_ticks']
   c=self.coverage.setdefault(bucket,[0,0]);c[0]+=1;c[1]+=int(self.tick_selected)
  selected=self.tick_selected is not False
  if key not in self.rows:
   if len(self.rows)>=CONTRACT['max_rows']:self.problem('capacity');selected=False
   else:self.rows[key]=dict(calls=0,sampled=0,exceptions=0,wall_ns=0,cpu_ns=0,exclusive_wall_ns=0,exclusive_cpu_ns=0,max_wall_ns=0,max_cpu_ns=0)
  row=self.rows.get(key)
  if row is not None:row['calls']+=1
  token=dict(label=label,key=key,arm=previous_arm,tick=previous_tick,selected=False)
  if selected:
   if len(self.stack)>=CONTRACT['max_stack']:self.problem('stack')
   else:
    w=self.wall();c=self.cpu();token.update(selected=True,w=w,c=c,child_w=0,child_c=0);self.stack.append(token)
  return token
 def leave(self,t,failed):
  if t is None:return
  try:
   row=self.rows.get(t['key'])
   if row is not None:row['exceptions']+=int(failed)
   if t['selected']:
    w=self.wall()-t['w'];c=self.cpu()-t['c']
    if not self.stack or self.stack[-1] is not t:raise ValueError('stack')
    self.stack.pop()
    if min(w,c,w-t['child_w'],c-t['child_c'])<0:raise ValueError('clock')
    row['sampled']+=1;row['wall_ns']+=w;row['cpu_ns']+=c;row['exclusive_wall_ns']+=w-t['child_w'];row['exclusive_cpu_ns']+=c-t['child_c'];row['max_wall_ns']=max(row['max_wall_ns'],w);row['max_cpu_ns']=max(row['max_cpu_ns'],c)
    if self.stack:self.stack[-1]['child_w']+=w;self.stack[-1]['child_c']+=c
  finally:
   self.arm=t['arm'];self.tick_selected=t['tick']
 def invoke(self,label,fn,args,kwargs,arm=None):
  t=None
  # Profiling failures must never suppress, repeat or replace the real call.
  previous_arm=self.arm;previous_tick=self.tick_selected;depth=len(self.stack)
  try:t=self.enter(label,arm)
  except Exception:self.problem('clock');self.arm=previous_arm;self.tick_selected=previous_tick;del self.stack[depth:]
  failed=True
  try:
   value=fn(*args,**kwargs);failed=False;return value
  finally:
   try:self.leave(t,failed)
   except Exception:self.problem('internal');del self.stack[depth:];self.arm=previous_arm;self.tick_selected=previous_tick
   if label in ('preview','phase','unloaded') and getattr(self,'output',None) is not None:self.checkpoint(self.output)
 def receipt(self,final=True):
  if final and self.stack:self.problem('unfinished')
  reasons=set(self.incomplete)
  if not final:reasons.add('in_progress')
  return dict(schema=CONTRACT['schema'],contract=CONTRACT,contract_sha256=contract_hash(),binding=self.binding,final=final,open_frames=len(self.stack),complete=final and not reasons,incomplete_reasons=sorted(reasons),native_qualified=False,rows=[dict(label=k[0],bucket=list(k[1:]),**v) for k,v in sorted(self.rows.items())],coverage=[dict(bucket=list(k),ticks=v[0],sampled_ticks=v[1],unsampled_ticks=v[0]-v[1]) for k,v in sorted(self.coverage.items())],scope='Measurement only; inclusive totals overlap; exclusive totals subtract measured direct children, include uninstrumented work and timer overhead. Sampled rows do not estimate unsampled CPU.')
 def checkpoint(self,path,final=False):
  try:
   raw=encode(self.receipt(final))
   if len(raw)>CONTRACT['max_bytes']:
    self.problem('capacity');r=self.receipt(final);r['rows']=[];r['coverage']=[];raw=encode(r)
   if len(raw)>CONTRACT['max_bytes']:raise ValueError('profile_sidecar_budget')
   path=Path(path);temporary=path.with_suffix('.tmp')
   with temporary.open('wb') as f:f.write(raw)
   temporary.replace(path)
  except Exception:
   self.problem('io')
   # On failure a prior incomplete checkpoint may remain. Never claim finality.
   try:sys.stderr.write('runtime_profile_incomplete:io\n')
   except Exception:pass
 def finish(self,path):
  if self.closed:return
  self.closed=True;self.checkpoint(path,final=True)

def call(label,fn,*args,**kwargs):
 p=ACTIVE
 return fn(*args,**kwargs) if p is None else p.invoke(label,fn,args,kwargs)

def measured(label):
 def decorate(fn):
  names=fn.__code__.co_varnames[:fn.__code__.co_argcount];index=names.index('arm') if 'arm' in names else None
  @functools.wraps(fn)
  def wrapped(*args,**kwargs):
   p=ACTIVE
   if p is None:return fn(*args,**kwargs)
   arm=kwargs.get('arm',args[index] if index is not None and len(args)>index else None)
   return p.invoke(label,fn,args,kwargs,arm)
  return wrapped
 return decorate

def start(out):
 global ACTIVE
 try:p=Profile();p.output=Path(out)/'runtime-profile015.json';ACTIVE=p
 except Exception:
  ACTIVE=None
  try:sys.stderr.write('runtime_profile_incomplete:startup\n')
  except Exception:pass
  return None
 try:
  run=Path(out).parent;raw=(run/'diagnostic-manifest.json').read_bytes();m=json.loads(raw)
  p.binding=dict(run_id=run.name,manifest_sha256=hashlib.sha256(raw).hexdigest(),runtime_variant=m['runtime_variant'],snapshot_sha256=m['snapshot_sha256'],config_sha256=m['config_sha256'])
  if m.get('profiling')!=CONTRACT:p.problem('binding')
 except Exception:p.problem('binding')
 return p

def session(fn):
 @functools.wraps(fn)
 def wrapped(*args,**kwargs):
  global ACTIVE
  previous=ACTIVE
  try:return fn(*args,**kwargs)
  finally:
   p=ACTIVE
   if p is not None and p is not previous:
    try:p.finish(p.output)
    except Exception:
     try:sys.stderr.write('runtime_profile_incomplete:internal\n')
     except Exception:pass
   ACTIVE=previous
 return wrapped
