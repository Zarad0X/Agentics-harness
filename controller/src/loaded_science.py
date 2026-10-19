"""Separate lossless loaded evidence; original pipeline/capture streams untouched."""
import runtime_profile as timing
import hashlib,json,lzma
from pathlib import Path
from provenance import write_once

CONTRACT='loaded-live-witness/1'
RAW_CAP=1024**4                 # 1 TiB streamed raw, never resident
XZ_CAP=64*1024**3               # 64 GiB compressed per complete episode
ROW_CAP=256*1024**2
ROWS_CAP=10100000               # above aggregate native 10M-step cap + admissions
OUTPUT_CAP=192*1024**3          # whole artifacts; checked at existing 2500-step cadence

class Science:
    def __init__(self,out,variant,context,binding):
        self.out=Path(out);self.path=self.out/'loaded-science.jsonl.xz'
        self.file=self.path.open('xb');self.codec=lzma.LZMACompressor(preset=3)
        self.sha=hashlib.sha256();self.xsha=hashlib.sha256();self.raw=0;self.compressed=0;self.rows=0
        self.variant=variant;self.context=context;self.binding=binding;self.closed=False;self.exhausted=False
        self.append(dict(kind='episode_header',schema=CONTRACT,variant=variant,binding=binding,
                         reconstructed=False,formal_credit=False,full_episode=True,
                         caps=dict(raw=RAW_CAP,xz=XZ_CAP,rows=ROWS_CAP,row=ROW_CAP,artifacts=OUTPUT_CAP)))
    @timing.measured('science')
    def append(self,row):
        if self.closed:raise RuntimeError('loaded_science_closed')
        if self.exhausted and row['kind'] not in ('live_motion_terminal','episode_terminal'):
            raise RuntimeError('loaded_science_budget')
        row=dict(row,sequence=self.rows,context=self.context())
        raw=(timing.call('json',json.dumps,row,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()
        if len(raw)>ROW_CAP:raise RuntimeError('loaded_science_row_budget')
        data=timing.call('lzma',self.codec.compress,raw);timing.call('write',self.file.write,data)
        self.sha.update(raw);self.xsha.update(data);self.raw+=len(raw);self.compressed+=len(data);self.rows+=1
        if self.rows%64==0:timing.call('flush',self.file.flush)
        if not self.exhausted and (self.raw>RAW_CAP or self.compressed>XZ_CAP or self.rows>ROWS_CAP):
            self.exhausted=True;raise RuntimeError('loaded_science_budget')
    def check_output(self):
        if sum(p.stat().st_size for p in self.out.rglob('*') if p.is_file())>OUTPUT_CAP:raise RuntimeError('combined_artifact_budget')
    def finish(self,failure):
        if self.closed:return
        try:self.append(dict(kind='episode_terminal',failure=failure,formal_credit=False,goal_complete=False))
        finally:
            data=self.codec.flush();timing.call('write',self.file.write,data);self.xsha.update(data);self.compressed+=len(data);self.file.close();self.closed=True
            write_once(self.out/'loaded-science-receipt.json',dict(schema=CONTRACT,variant=self.variant,binding=self.binding,
                rows=self.rows,raw_bytes=self.raw,raw_sha256=self.sha.hexdigest(),compressed_bytes=self.compressed,
                compressed_sha256=self.xsha.hexdigest(),failure=failure,exhausted=self.exhausted,
                within_budget=self.raw<=RAW_CAP and self.compressed<=XZ_CAP and self.rows<=ROWS_CAP,
                formal_credit=False,goal_complete=False))
