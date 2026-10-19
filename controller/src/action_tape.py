"""Lossless queryable append-only action prefixes, bounded memory, no physics.
Reports retain their separate 32MiB/900KiB limits. Tape records are canonical
JSONL; immutable chunks and append-only index permit O(new bytes) checkpoints.
"""
import runtime_profile as timing
import gzip,hashlib,json,os,shutil
from pathlib import Path
from provenance import canonical

CHUNK_RAW=512*1024
RECORD_RAW=64*1024
# One full 10M-tick prefix plus non-tick attachment records. This is an IO
# budget, never permission to execute more physics or alter the 24h supervisor.
MAX_RECORDS=10_000_000+1024
DISK_RESERVE=2*1024**3
# Storage-only profiles. Copy budget covers 10M boundaries with <=20 objects,
# up to two API rows/object plus before/setters/terminal/recheck and terminal.
PROFILES={'copy':(512*1024,1024*1024,450_000_001),
          'summaries':(64*1024,512*1024,10_000_000)}
class TapeFailure(RuntimeError):pass

def hash_file(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for block in iter(lambda:f.read(65536),b''):h.update(block)
 return h.hexdigest()

class Writer:
 def __init__(self,out,tape_id,*,reserve=DISK_RESERVE,profile=None):
  self.out=Path(out);self.id=tape_id;self.reserve=reserve;self.profile=profile
  if profile is not None and profile not in PROFILES:raise TapeFailure('action_tape_profile')
  self.record_limit,self.chunk_limit,self.record_count_limit=PROFILES[profile] if profile else (RECORD_RAW,CHUNK_RAW,MAX_RECORDS)
  self.folder=self.out/'live-action-tapes'/('%08d'%tape_id);self.folder.mkdir(parents=True,exist_ok=False)
  self.buffer=bytearray();self.count=0;self.raw_bytes=0;self.hash=hashlib.sha256();self.index_hash=hashlib.sha256();self.index_bytes=0;self.chunks=0;self.buffer_count=0
  self.index=(self.folder/'index.jsonl').open('xb');self.closed=False
  self._space(self.chunk_limit+self.record_limit)
 def _space(self,needed):
  if shutil.disk_usage(self.out).free<self.reserve+needed:raise TapeFailure('action_tape_data_disk_reserve')
 @timing.measured('tape_append')
 def append(self,raw):
  if self.closed:raise TapeFailure('action_tape_closed')
  if len(raw)>self.record_limit or not raw.endswith(b'\n'):raise TapeFailure('action_tape_record_budget')
  if self.count>=self.record_count_limit:raise TapeFailure('action_tape_episode_record_budget')
  if self.buffer and len(self.buffer)+len(raw)>self.chunk_limit:self.flush()
  self.buffer.extend(raw);self.buffer_count+=1;self.count+=1;self.raw_bytes+=len(raw);self.hash.update(raw)
 @timing.measured('tape_flush')
 def flush(self):
  if not self.buffer:return
  self._space(2*len(self.buffer)+4096)
  raw=bytes(self.buffer);data=timing.call('gzip',gzip.compress,raw,mtime=0);name='%08d.jsonl.gz'%self.chunks
  path=self.folder/name
  try:
   with path.open('xb') as f:timing.call('tape_data_io',f.write,data);timing.call('flush',f.flush);timing.call('fsync',os.fsync,f.fileno())
   path.chmod(0o444)
   row=dict(index=self.chunks,path=name,first_record=self.count-self.buffer_count,count=self.buffer_count,raw_bytes=len(raw),raw_sha256=hashlib.sha256(raw).hexdigest(),compressed_bytes=len(data),compressed_sha256=hashlib.sha256(data).hexdigest())
   line=canonical(row);timing.call('tape_index_io',self.index.write,line);timing.call('flush',self.index.flush);timing.call('fsync',os.fsync,self.index.fileno())
  except OSError as exc:raise TapeFailure('action_tape_write:'+str(exc)) from exc
  self.index_hash.update(line);self.index_bytes+=len(line);self.chunks+=1;self.buffer.clear();self.buffer_count=0
 def checkpoint(self):
  self.flush()
  receipt=dict(encoding='chunked-action-tape/1',tape_id=self.id,count=self.count,sha256=self.hash.hexdigest(),uncompressed_size=self.raw_bytes,chunks=self.chunks,index_bytes=self.index_bytes,index_sha256=self.index_hash.hexdigest(),record_limit=self.record_limit,chunk_limit=self.chunk_limit,episode_record_limit=self.record_count_limit)
  if self.profile:receipt['storage_profile']=self.profile
  return receipt
 def close(self):
  if self.closed:return
  receipt=self.checkpoint();self.index.close();self.closed=True
  with (self.folder/'terminal.json').open('xb') as f:timing.call('tape_data_io',f.write,canonical(receipt));timing.call('flush',f.flush);timing.call('fsync',os.fsync,f.fileno())
  (self.folder/'index.jsonl').chmod(0o444);(self.folder/'terminal.json').chmod(0o444)


def records(out,binding,*,profile=None):
 """Independent streaming decode; binding is the immutable prefix snapshot.
 Index may have a later suffix from the same live prefix. Exactly its bound
 bytes/chunks are consumed; changed/missing/order-shuffled prefixes fail closed.
 """
 def need(ok,why):
  if not ok:raise ValueError('action_tape_'+why)
 b=binding;need(b['encoding']=='chunked-action-tape/1','encoding')
 need(b.get('storage_profile')==profile and (profile is None or profile in PROFILES),'profile')
 record_limit,chunk_limit,record_count_limit=PROFILES[profile] if profile else (RECORD_RAW,CHUNK_RAW,MAX_RECORDS)
 need((b['record_limit'],b['chunk_limit'],b['episode_record_limit'])==(record_limit,chunk_limit,record_count_limit),'declared_limits')
 need(type(b['tape_id']) is int and b['tape_id']>0,'id')
 need(type(b['count']) is int and 0<=b['count']<=record_count_limit,'count')
 need(type(b['chunks']) is int and 0<=b['chunks']<=b['count'],'chunks')
 folder=Path(out)/'live-action-tapes'/('%08d'%b['tape_id']);need(folder.resolve().is_relative_to(Path(out).resolve()),'path')
 total=0;size=0;index_bytes=0;ih=hashlib.sha256();rh=hashlib.sha256()
 with (folder/'index.jsonl').open('rb') as index:
  for i in range(b['chunks']):
   line=index.readline(4097);need(0<len(line)<=4096 and line.endswith(b'\n'),'index_truncated')
   ih.update(line);index_bytes+=len(line);r=json.loads(line);need(canonical(r)==line,'index_canonical')
   need(r['index']==i and r['first_record']==total and r['path']=='%08d.jsonl.gz'%i,'chunk_order')
   need(type(r['raw_bytes']) is int and 0<r['raw_bytes']<=chunk_limit and 0<r['compressed_bytes']<=chunk_limit+1024,'chunk_budget')
   path=folder/r['path'];need(path.resolve().is_relative_to(folder.resolve()),'chunk_path')
   need(path.stat().st_size==r['compressed_bytes'] and hash_file(path)==r['compressed_sha256'],'compressed_binding')
   ch=hashlib.sha256();cs=0;cn=0
   with gzip.open(path,'rb') as stream:
    while True:
     raw=stream.readline(record_limit+1)
     if not raw:break
     need(len(raw)<=record_limit and raw.endswith(b'\n'),'record_truncated')
     cs+=len(raw);need(cs<=r['raw_bytes'],'raw_overflow');ch.update(raw);rh.update(raw);cn+=1;total+=1;size+=len(raw)
     need(total<=b['count'],'record_overflow');row=json.loads(raw);need(canonical(row)==raw,'record_canonical');yield row
   need(cs==r['raw_bytes'] and cn==r['count'] and ch.hexdigest()==r['raw_sha256'],'raw_binding')
 need(index_bytes==b['index_bytes'] and ih.hexdigest()==b['index_sha256'],'index_binding')
 need(total==b['count'] and size==b['uncompressed_size'] and rh.hexdigest()==b['sha256'],'prefix_binding')


def complete_records(out,binding,*,profile):
 """Require a closed stream, exact inventory and terminal, then decode fully."""
 folder=Path(out)/'live-action-tapes'/('%08d'%binding['tape_id'])
 if (folder/'terminal.json').stat().st_size>4096:raise ValueError('stream_terminal_budget')
 if json.loads((folder/'terminal.json').read_bytes())!=binding:raise ValueError('stream_terminal_binding')
 if (folder/'index.jsonl').stat().st_size!=binding['index_bytes']:raise ValueError('stream_index_suffix')
 names={'index.jsonl','terminal.json'}
 # Directory iteration and membership avoid reading any chunk into RAM.
 count=0
 for p in folder.iterdir():
  if not p.is_file() or (p.name not in names and not (p.name.endswith('.jsonl.gz') and p.name[:-9].isdigit() and p.name=='%08d.jsonl.gz'%int(p.name[:-9]) and 0<=int(p.name[:-9])<binding['chunks'])):raise ValueError('stream_inventory')
  count+=1
 if count!=binding['chunks']+2:raise ValueError('stream_inventory_missing')
 yield from records(out,binding,profile=profile)
