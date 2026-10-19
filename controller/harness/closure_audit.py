"""Independent finger-control receipt and unchanged settling predicate checks."""
import numpy as np
from provenance import digest

def verify(events):
 def need(ok,why):
  if not ok:raise ValueError('closure_audit_'+why)
 source=None;controls=None;pending=None;last=-1;dwell=0;accepted=0;updates=0
 for e in events:
  if e['kind'] in ('grasp_preview','return_preview'):
   verify(e.get('separation_trials',e.get('trace',[])))
  if e['kind']=='closure_control_source':
   source=e['source'];need(digest({k:v for k,v in source.items() if k!='sha256'})==source['sha256'],'source_hash')
   need(source['arm']==e['arm'] and source['object_id']==e['object_id'],'identity')
   controls=source['controls'];need(len(source['slots'])==2 and len(set(source['slots']))==2,'slots')
   expected=[7,8] if source['arm']=='left' else [16,17]
   need(source['slots']==expected and len(controls)==18,'selected_arm_slots')
   for i in expected:need(list(controls[i][:4])==[1000.,200.,4.,'force'] and list(controls[i][5])==[0.],'actuator')
   pending=None;last=-1;dwell=0
  if e['kind']=='closure_control_update':
   need(source is not None and pending is None and e['source_sha256']==source['sha256'],'update_source')
   row=e['measurement'];elapsed=e['elapsed_ticks'];slots=source['slots'];old=np.asarray([controls[i][4][0] for i in slots])
   need(elapsed==last+1 and elapsed<1000 and row['bilateral_contact'] is True and (elapsed<100 or max(map(abs,row['finger_velocity']))>.01),'update_stage')
   need(e['before_targets']==old.tolist(),'previous')
   position=np.asarray(row['finger_position']);need(np.isfinite(position).all(),'finite')
   limit=np.minimum(.04,old+.01*.004);request=np.maximum(old,np.minimum(limit,position-.00025));cast=request.astype(np.float32)
   cast=np.where(cast.astype(float)>request,np.nextafter(cast,np.float32(-np.inf)),cast).astype(float)
   target=np.maximum(old,cast).tolist();need(target==e['targets'],'control_formula')
   new=[list(x) for x in controls]
   for i,value in zip(slots,target):new[i][4]=[value]
   need(digest(new)==digest(e['controls']),'other_controls');controls=e['controls'];pending=e;updates+=1
  if e['kind']=='gripper_hold':
   need(source is not None and e['arm']==source['arm'] and e['object_id']==source['object_id'],'final_source')
   need(e['contact_targets_semantics']=='original_contact_latch' and e['retained_targets']==[controls[i][4][0] for i in source['slots']],'final_retained_targets')
  if e['kind']=='gripper_settle':
   need(source is not None and e['arm']==source['arm'] and e['object_id']==source['object_id'],'settle_source')
   elapsed=e['elapsed_ticks'];need(elapsed==last+1 and elapsed<=1000,'clock');last=elapsed
   vals=[*e['finger_velocity'],*e['finger_position'],e['position_error_m'],e['rotation_error_rad']];need(np.isfinite(vals).all(),'finite')
   if e['outcome']=='pose_guard_failed':need(e['position_error_m']>.005 or e['rotation_error_rad']>.04,'pose_failure');continue
   need(e['position_error_m']<=.005 and e['rotation_error_rad']<=.04,'pose')
   if pending is not None:need(all(e[k]==v for k,v in pending['measurement'].items()),'measurement_binding')
   stable=max(map(abs,e['finger_velocity']))<=.01 and e['bilateral_contact'] is True
   dwell=dwell+1 if elapsed and stable and pending is None else 0
   need(e['consecutive_ticks']==dwell,'dwell');settled=elapsed>=100 and dwell>=25
   need((e['outcome']=='settled')==settled,'acceptance');accepted+=int(settled);pending=None
 need(pending is None,'unobserved_update')
 return dict(valid=True,accepted=accepted,updates=updates,native_success=False)
