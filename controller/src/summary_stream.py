"""Episode summary sequence backed by immutable per-boundary files and shared IO.
No report/snapshot history is retained in RAM. Reports keep their own budgets.
"""
import json
from pathlib import Path
from action_tape import Writer,complete_records,PROFILES,TapeFailure
from provenance import canonical
class Summaries:
 def __init__(self,out):
  self.out=Path(out);self.count=0;self.qualified=set();self.writer=Writer(self.out/'live-sleep-summaries',1,profile='summaries')
 def __len__(self):return self.count
 def __getitem__(self,index):
  if type(index) is not int or not 0<=index<self.count:raise IndexError(index)
  return json.loads((self.out/'live-sleep'/('%04d'%(index+1))/'summary.json').read_bytes())
 def __iter__(self):
  for i in range(self.count):yield self[i]
 def append(self,row):
  if row['index']!=self.count+1:raise TapeFailure('summary_sequence')
  self.writer.append(canonical(row));self.count+=1
  if row['status']=='QUALIFIED_FORECAST_PAIR' and row['source_role']=='grasp_preview':self.qualified.add(row['object_id'])
  # Each boundary is durable; never rewrite older summaries.
  self.writer.checkpoint()
 def close(self):
  self.writer.close();return json.loads((self.writer.folder/'terminal.json').read_bytes())
def rows(out,terminal):
 if 'summary_stream' in terminal:
  if 'rows' in terminal:raise ValueError('live_duplicate_summary_encoding')
  yield from complete_records(Path(out)/'live-sleep-summaries',terminal['summary_stream'],profile='summaries')
 else:
  legacy=terminal['rows']
  if not isinstance(legacy,list) or len(legacy)>64:raise ValueError('legacy_live_boundary_cap')
  yield from legacy
