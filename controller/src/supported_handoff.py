"""Pre-attachment supported grasp drive relief, using REAL bounded commands.
No state setters, loaded exemptions, gain changes, or instantaneous retarget.
A finite unloaded arm transition precedes fresh ordinary loaded admission.
"""
import copy
import numpy as np
import astra_planner as planner
from actuator_targets import emitted,goal,issue
from collision_world import pose_matrix
from provenance import digest
from preview_errors import CandidateInfeasible
from return_clearance import DT,MAX_VELOCITY,MAX_ACCELERATION,TRACK_Q,WORLD_DRIFT_M,OBS_SPEED,OBS_SPIN,MEASURED_SPEED

SUPPORTS=('table','floor','left_tray','right_tray')
MAX_ROUNDS=3
MAX_TICKS=1000
DWELL=25

def need(ok,reason):
 if not ok:raise CandidateInfeasible('supported_handoff_'+reason)

def selected(row,arm):
 c=row['controls'][arm][:7]
 return np.asarray([j['target'][0] for j in c]),np.asarray([j['velocity'][0] for j in c])

def geometry(world,row,arm,target):
 w=world(row['state']);q=np.asarray(row['state']['robots'][arm]['q'][:7]);previous,_=selected(row,arm)
 rel=np.linalg.inv(np.asarray(row['native_ee']))@pose_matrix(row['state']['objects'][str(target)]['pose']);w.held=(arm,str(target),rel)
 fingers=np.clip(row['state']['robots'][arm]['q'][-2:],0,.04).tolist()
 measured=bool(w.free(arm,q.tolist(),target,True,fingers));measured_pair=copy.deepcopy(w.last_collision)
 first=emitted(q,np.asarray(w.model.bounds)+[.03,-.03]);previous=emitted(previous,np.asarray(w.model.bounds)+[.03,-.03])
 prefix=np.linspace(previous,first,max(2,int(np.ceil(np.max(np.abs(first-previous))/.025))+1))
 safe=True;pair=None
 for x in prefix:
  if not w.free(arm,x.tolist(),target,True,fingers):safe=False;pair=copy.deepcopy(w.last_collision);break
 return dict(measured=measured,measured_pair=measured_pair,prefix=safe,pair=pair)

def commands(start,end,bounds):
 # Same quintic interpolation and speed/acceleration limits as careful insertion.
 dense=np.asarray(planner.resample_path([start,end],max_velocity=MAX_VELOCITY,dt=DT,max_acceleration=MAX_ACCELERATION))
 q=emitted(dense,bounds);v=np.vstack((np.diff(dense,axis=0)/DT,np.zeros((1,7)))).astype(np.float32).astype(float)
 need(len(q)<=MAX_TICKS and np.max(np.abs(v))<=MAX_VELOCITY+1e-5,'profile_velocity')
 need(np.max(np.abs(np.diff(np.vstack((np.zeros((1,7)),v)),axis=0)/DT))<=MAX_ACCELERATION+1e-5,'profile_acceleration')
 need(np.max(np.abs(np.diff(q,axis=0)))<=MAX_VELOCITY*DT+1e-6,'profile_slew')
 return q,v

def validate(row,initial,arm,target,world):
 need(row['held_id'] is None and row['drive_active'] is False,'unloaded_required')
 s=row['state'];m=row['measurement'];numbers=[*m['finger_velocity'],*m['finger_position'],m['position_error_m'],m['rotation_error_rad']]
 need(np.isfinite(numbers).all() and len(m['finger_velocity'])==len(m['finger_position'])==2,'measurement')
 need(m['bilateral_contact'] is True and max(map(abs,m['finger_velocity']))<=.01,'finger_stability')
 need(m['position_error_m']<=.005 and m['rotation_error_rad']<=.04,'grasp_pose')
 for side in ('left','right'):
  q=np.asarray(s['robots'][side]['q']);vel=np.asarray(s['robots'][side]['qdot']);need(np.isfinite(q).all() and np.isfinite(vel).all(),'robot_nonfinite')
  need(np.max(np.abs(vel[:7]))<=MEASURED_SPEED and np.max(np.abs(vel[-2:]))<=.01,'robot_speed')
  control=np.asarray([j['target'][0] for j in row['controls'][side][:7]])
  need(np.max(np.abs(q[:7]-control))<=TRACK_Q,'arm_tracking')
 for oid,obj in s['objects'].items():
  before=initial['state']['objects'][oid];t=pose_matrix(obj['pose']);b=pose_matrix(before['pose'])
  need(np.linalg.norm(t[:3,3]-b[:3,3])<=WORLD_DRIFT_M and np.linalg.norm(planner.so3_log(t[:3,:3]@b[:3,:3].T))<=.04,'object_drift')
  need(np.isfinite(obj['linear_velocity']+obj['angular_velocity']).all() and np.linalg.norm(obj['linear_velocity'])<=OBS_SPEED and np.linalg.norm(obj['angular_velocity'])<=OBS_SPIN,'object_speed')
 w=world(s);f=np.clip(s['robots'][arm]['q'][-2:],0,.04).tolist()
 need(w.free(arm,s['robots'][arm]['q'][:7],target,True,f),'measured_unloaded_geometry')
 need(geometry(world,row,arm,target)['measured'],'measured_payload_geometry')

def run(api,arm,target,candidate_sha256):
 """api contains getters, native issue/tick/guard and ordinary world factory."""
 initial=api.observe();need(initial['held_id'] is None and initial['drive_active'] is False,'unloaded_required');g=geometry(api.world,initial,arm,target)
 source=dict(schema='supported-handoff/014',arm=arm,object_id=target,phase=api.phase,returning=api.returning,dt=api.dt,candidate_sha256=candidate_sha256,initial=initial,geometry=g)
 source['sha256']=digest(source);api.emit('supported_handoff_source',source=source)
 ticks=0;rounds=0;status='failed';chain=source['sha256'];last=initial
 def record(kind,**kw):
  nonlocal chain
  row=dict(source_sha256=source['sha256'],previous_sha256=chain,**kw);row['sha256']=digest(row);chain=row['sha256'];api.emit(kind,record=row)
 try:
  # Other initial neighbor cases retain their original separation mechanism.
  if g['prefix'] or g['pair'] not in [['payload',n] for n in SUPPORTS]:status='not_needed';return
  need(g['measured'],'unsafe_measured_start')
  need(abs(api.dt-DT)<=1e-9,'timestep')
  baseline=copy.deepcopy(initial['controls']);bounds=np.asarray(api.model.bounds)+[.03,-.03]
  previous,velocity=selected(initial,arm);emitted(previous,bounds)
  need(np.array_equal(velocity,np.zeros(7)),'initial_velocity')
  validate(initial,initial,arm,target,api.world)
  for rounds in range(1,MAX_ROUNDS+1):
   row=api.observe();need(row['controls']==baseline,'unexpected_controls');previous,velocity=selected(row,arm);need(np.array_equal(velocity,np.zeros(7)),'round_velocity')
   endpoint=goal(row['state']['robots'][arm]['q'][:7],bounds);q,v=commands(previous,endpoint,bounds)
   record('supported_handoff_plan',round=rounds,start=row,endpoint=endpoint.tolist(),count=len(q))
   for index,(command,vel) in enumerate(zip(q,v)):
    need(ticks<MAX_TICKS,'budget');before=api.observe();need(before['controls']==baseline,'unexpected_controls');need(api.guard() is True,'contact');validate(before,initial,arm,target,api.world)
    # Revalidate every impending unloaded target/edge in the actual current
    # world. Object is still supported, fingers fixed; no magnet yet.
    w=api.world(before['state']);f=np.clip(before['state']['robots'][arm]['q'][-2:],0,.04).tolist();old,_=selected(before,arm)
    for x in np.linspace(old,command,max(2,int(np.ceil(np.max(np.abs(command-old))/.002))+1)):
     need(w.free(arm,x.tolist(),target,True,f),'command_geometry')
    expected=copy.deepcopy(baseline)
    for j,a,b in zip(expected[arm][:7],command,vel):j['target']=[float(a)];j['velocity']=[float(b)]
    issue(api.robot,arm,command,vel,bounds)
    readback=api.observe();need(readback['controls']==expected,'readback_or_other_controls');baseline=expected
    record('supported_handoff_command',round=rounds,index=index,ticks=ticks,before=before,targets=command.tolist(),velocities=vel.tolist(),controls=readback['controls'])
    api.tick();ticks+=1;need(api.guard() is True,'contact');last=api.observe();need(last['controls']==baseline,'post_controls');validate(last,initial,arm,target,api.world)
    record('supported_handoff_observation',ticks=ticks,observation=last)
   for dwell in range(DWELL):
    need(ticks<MAX_TICKS,'budget');need(api.guard() is True,'contact');before=api.observe();need(before['controls']==baseline,'hold_controls');validate(before,initial,arm,target,api.world)
    record('supported_handoff_hold',ticks=ticks,before=before)
    api.tick();ticks+=1;need(api.guard() is True,'contact');last=api.observe();need(last['controls']==baseline,'post_hold_controls');validate(last,initial,arm,target,api.world)
    record('supported_handoff_observation',ticks=ticks,observation=last)
   g=geometry(api.world,last,arm,target)
   if g['measured'] and g['prefix']:status='aligned';return
  raise CandidateInfeasible('supported_handoff_not_converged')
 finally:
  api.emit('supported_handoff_terminal',source_sha256=source['sha256'],last_sha256=chain,status=status,ticks=ticks,rounds=rounds,final=api.observe())
