"""Rate-limited release of stale finger preload, never a state/velocity reset.
Only opening target corrections toward measured position minus the EXISTING
0.25mm preload. Existing 0.01m/s target slew, gains, force and travel retained.
Native velocity/contact/pose/dwell acceptance remains in closure_hold.
"""
import numpy as np
from provenance import digest
from preview_errors import CandidateInfeasible

def targets(previous,position):
 previous=np.asarray(previous,float);position=np.asarray(position,float)
 if previous.shape!=(2,) or position.shape!=(2,) or not np.isfinite([previous,position]).all() or np.any(previous<0) or np.any(previous>.04):raise CandidateInfeasible('closure_regulation_invalid_target')
 limit=np.minimum(.04,previous+.01*.004)
 requested=np.maximum(previous,np.minimum(limit,position-.00025))
 native=requested.astype(np.float32)
 native=np.where(native.astype(float)>requested,np.nextafter(native,np.float32(-np.inf)),native).astype(float)
 result=np.maximum(previous,native)
 if np.any(result<previous) or np.any(result>limit) or not np.array_equal(result,result.astype(np.float32).astype(float)):raise CandidateInfeasible('closure_regulation_native_slew')
 return result.tolist()

def wire_controls(rows):
 from live_sleep_readback import force_limit
 return [[float(s),float(d),force_limit(f),str(m),list(map(float,q)),list(map(float,v))] for s,d,f,m,q,v in rows]

class Regulator:
 def __init__(self,joints,slots,controls,emit,source):
  self.joints=joints;self.slots=slots;self.controls=controls;self.emit=emit;self.source=dict(source,slots=slots,controls=wire_controls(controls()),schema='closure-regulation/011')
  self.source['sha256']=digest(self.source);emit('closure_control_source',source=self.source)
 def __call__(self,row,elapsed):
  before=self.controls();previous=[float(j.get_drive_target()[0]) for j in self.joints]
  active=elapsed<100 or max(abs(v) for v in row['finger_velocity'])>.01
  desired=targets(previous,row['finger_position']) if active and row['bilateral_contact'] is True else previous
  if desired==previous:return False
  for i in self.slots:
   if before[i][:4]!=(1000.,200.,4.,'force') or before[i][5]!=(0.,):raise CandidateInfeasible('closure_regulation_actuator_contract')
  expected=list(before)
  for i,value in zip(self.slots,desired):
   old=before[i]
   if old[5]!=(0.,):raise CandidateInfeasible('closure_regulation_velocity_target')
   expected[i]=(*old[:4],(value,),old[5])
  for joint,value in zip(self.joints,desired):joint.set_drive_target(value) # velocity targets remain exactly zero
  after=self.controls()
  if tuple(expected)!=after:raise CandidateInfeasible('closure_regulation_readback_or_other_control')
  self.emit('closure_control_update',source_sha256=self.source['sha256'],elapsed_ticks=elapsed,measurement=row,before_targets=previous,targets=desired,controls=wire_controls(after))
  return True
