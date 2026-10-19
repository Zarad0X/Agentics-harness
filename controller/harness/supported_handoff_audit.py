"""Ordered pre-attachment transition audit from actual closure/caller evidence.
Reconstruct native commands and geometry; never call the transition controller.
"""
import copy
import numpy as np
from provenance import digest
from collision_world import pose_matrix
import astra_planner as planner

def require(x,why):
 if not x:raise ValueError('supported_handoff_audit_'+why)

def controls_wire(c):
 return [[j['stiffness'],j['damping'],j['force_limit'],j['mode'],j['target'],j['velocity']] for a in ('left','right') for j in c[a]]

def geometry(world,row,arm,target):
 w=world(row['state']);q=np.asarray(row['state']['robots'][arm]['q'][:7]);previous=np.asarray([v['target'][0] for v in row['controls'][arm][:7]])
 bounds=np.asarray(w.model.bounds)+[.03,-.03]
 require(np.isfinite([q,previous]).all() and np.all(previous>=bounds[:,0]) and np.all(previous<=bounds[:,1]),'geometry_bounds')
 q=q.astype(np.float32).astype(float);previous=previous.astype(np.float32).astype(float)
 w.held=(arm,str(target),np.linalg.inv(np.asarray(row['native_ee']))@pose_matrix(row['state']['objects'][str(target)]['pose']))
 fingers=np.clip(row['state']['robots'][arm]['q'][-2:],0,.04).tolist()
 measured=bool(w.free(arm,row['state']['robots'][arm]['q'][:7],target,True,fingers));pair=copy.deepcopy(w.last_collision)
 safe=True;bad=None
 for x in np.linspace(previous,q,max(2,int(np.ceil(np.max(np.abs(q-previous))/.025))+1)):
  if not w.free(arm,x.tolist(),target,True,fingers):safe=False;bad=copy.deepcopy(w.last_collision);break
 return dict(measured=measured,measured_pair=pair,prefix=safe,pair=bad)

def observation(row,initial,arm,target,world):
 require(row['held_id'] is None and row['drive_active'] is False,'unloaded_required')
 m=row['measurement'];require(np.isfinite([*m['finger_velocity'],*m['finger_position'],m['position_error_m'],m['rotation_error_rad']]).all(),'finite')
 require(len(m['finger_velocity'])==len(m['finger_position'])==2 and m['bilateral_contact'] is True and max(map(abs,m['finger_velocity']))<=.01 and m['position_error_m']<=.005 and m['rotation_error_rad']<=.04,'contact_pose_stability')
 s=row['state']
 for a in ('left','right'):
  q=np.asarray(s['robots'][a]['q']);v=np.asarray(s['robots'][a]['qdot']);c=np.asarray([j['target'][0] for j in row['controls'][a][:7]])
  require(np.isfinite(q).all() and np.isfinite(v).all() and np.max(np.abs(v[:7]))<=.15 and np.max(np.abs(v[-2:]))<=.01 and np.max(np.abs(q[:7]-c))<=.01,'tracking_speed')
 for oid,obj in s['objects'].items():
  a=pose_matrix(obj['pose']);b=pose_matrix(initial['state']['objects'][oid]['pose'])
  require(np.linalg.norm(a[:3,3]-b[:3,3])<=.003 and np.linalg.norm(planner.so3_log(a[:3,:3]@b[:3,:3].T))<=.04,'object_drift')
  require(np.isfinite(obj['linear_velocity']+obj['angular_velocity']).all() and np.linalg.norm(obj['linear_velocity'])<=.03 and np.linalg.norm(obj['angular_velocity'])<=.15,'object_speed')
 w=world(s);require(w.free(arm,s['robots'][arm]['q'][:7],target,True,np.clip(s['robots'][arm]['q'][-2:],0,.04).tolist()),'measured_unloaded')
 require(geometry(world,row,arm,target)['measured'],'measured_loaded')

def verify(events,world):
 closure=None;closure_controls=None;settled=None;after=None;plan=None;source=None;terminal=None;chain=None;pending=None;last=None;commands=None;ticks=0;round_no=0;index=0;dwell=0;accepted=0;ended=True
 for e in events:
  k=e['kind']
  if k in ('grasp_preview','return_preview'):verify(e.get('separation_trials',e.get('trace',[])),world)
  if k=='custom_plan':plan=e
  if k=='closure_control_source':closure=e['source'];closure_controls=closure['controls'];settled=None;terminal=None;source=None
  if k=='closure_control_update':closure_controls=e['controls']
  if k=='gripper_settle':settled=e
  if k=='after_closure':after=e
  if k=='supported_handoff_source':
   require(ended,'missing_terminal');ended=False
   s=e['source'];require(digest({k:v for k,v in s.items() if k!='sha256'})==s['sha256'],'source_hash');arm=s['arm'];target=s['object_id']
   require(closure is not None and after is not None and settled is not None and settled['outcome']=='settled','closure_source')
   require((arm,target,s['candidate_sha256'])==(closure['arm'],closure['object_id'],closure['candidate_sha256']),'identity')
   require(plan is not None and plan['arm']==arm and plan['target']==target and (s['phase'],s['returning'])==(plan['caller']['phase'],plan['caller']['returning']),'phase')
   require(s['initial']['state']==after['state'] and controls_wire(s['initial']['controls'])==closure_controls,'initial_source')
   for key,val in s['initial']['measurement'].items():require(settled[key]==val,'closure_measurement')
   require(s['initial']['held_id'] is None and s['initial']['drive_active'] is False,'unloaded_source')
   actual=geometry(world,s['initial'],arm,target);require(s['geometry']==actual,'geometry_source')
   source=s;chain=s['sha256'];last=s['initial'];pending=None;ticks=round_no=index=dwell=0;commands=None;terminal=None
  if k in ('supported_handoff_plan','supported_handoff_command','supported_handoff_hold','supported_handoff_observation'):
   r=e['record'];require(source is not None and r['source_sha256']==source['sha256'] and r['previous_sha256']==chain and digest({k:v for k,v in r.items() if k!='sha256'})==r['sha256'],'chain');chain=r['sha256'];arm=source['arm'];target=source['object_id']
   if k=='supported_handoff_plan':
    require(pending is None and (round_no==0 or (index==len(commands[0]) and dwell==25)) and r['round']==round_no+1<=3 and r['start']==last,'plan_order');round_no+=1;index=dwell=0
    require(abs(source['dt']-.004)<=1e-9,'timestep')
    g=geometry(world,last,arm,target);require(g['measured'] and not g['prefix'],'transition_needed');observation(last,source['initial'],arm,target,world)
    controls=last['controls'];previous=np.asarray([j['target'][0] for j in controls[arm][:7]]);require(all(j['velocity']==[0.] for j in controls[arm][:7]),'start_velocity')
    bounds=np.asarray(world(last['state']).model.bounds)+[.03,-.03];goal=np.asarray(last['state']['robots'][arm]['q'][:7]);lo=bounds[:,0].astype(np.float32);hi=bounds[:,1].astype(np.float32);lo=np.where(lo.astype(float)<bounds[:,0],np.nextafter(lo,np.float32(np.inf)),lo);hi=np.where(hi.astype(float)>bounds[:,1],np.nextafter(hi,np.float32(-np.inf)),hi);goal=np.clip(goal.astype(np.float32).astype(float),lo,hi)
    require(r['endpoint']==goal.tolist(),'endpoint');dense=np.asarray(planner.resample_path([previous,goal],max_velocity=.1,dt=.004,max_acceleration=.4));q=dense.astype(np.float32).astype(float);v=np.vstack((np.diff(dense,axis=0)/.004,np.zeros((1,7)))).astype(np.float32).astype(float);require(r['count']==len(q)<=1000 and np.all(q>=bounds[:,0]) and np.all(q<=bounds[:,1]),'profile');commands=(q,v)
   elif k in ('supported_handoff_command','supported_handoff_hold'):
    require(pending is None and r['ticks']==ticks<1000 and r['before']==last,'before_tick');observation(last,source['initial'],arm,target,world)
    if k=='supported_handoff_command':
     require(commands is not None and r['round']==round_no and r['index']==index<len(commands[0]),'command_order');q,v=commands[0][index],commands[1][index];require(r['targets']==q.tolist() and r['velocities']==v.tolist(),'command_formula')
     expected=copy.deepcopy(last['controls'])
     for j,a,b in zip(expected[arm][:7],q,v):j['target']=[float(a)];j['velocity']=[float(b)]
     require(expected==r['controls'],'controls');w=world(last['state']);old=np.asarray([j['target'][0] for j in last['controls'][arm][:7]]);f=np.clip(last['state']['robots'][arm]['q'][-2:],0,.04).tolist()
     for x in np.linspace(old,q,max(2,int(np.ceil(np.max(np.abs(q-old))/.002))+1)):require(w.free(arm,x.tolist(),target,True,f),'unloaded_edge')
     pending=expected;index+=1
    else:require(commands is not None and index==len(commands[0]) and dwell<25,'hold_order');pending=last['controls'];dwell+=1
   else:
    require(pending is not None and r['ticks']==ticks+1,'post_tick');last=r['observation'];require(last['controls']==pending,'post_controls');observation(last,source['initial'],arm,target,world);pending=None;ticks+=1
  if k=='supported_handoff_terminal':
   ended=True
   require(source is not None and e['source_sha256']==source['sha256'] and e['last_sha256']==chain,'terminal_binding');terminal=e
   require(e['status'] in ('failed','aligned','not_needed') and ticks<=e['ticks']<=ticks+1<=1001,'terminal_clock')
   if e['status']!='failed':
    require(pending is None and e['ticks']==ticks and e['rounds']==round_no and e['final']==last,'terminal_state')
    g=geometry(world,last,source['arm'],source['object_id'])
    if e['status']=='aligned':require(commands is not None and index==len(commands[0]) and dwell==25 and g['prefix'] and g['measured'],'acceptance');accepted+=1
    else:require(ticks==0 and (g['prefix'] or g['pair'] not in [['payload',n] for n in ('table','floor','left_tray','right_tray')]),'not_needed')
  if k=='held_transform_source' and e['origin']=='magnet_attachment':
   require(terminal is not None and terminal['status']!='failed','attachment_without_transition')
   require(e['arm']==source['arm'] and e['object_id']==source['object_id'] and e['state']==terminal['final']['state'] and e['ee_transform']==terminal['final']['native_ee'],'attachment_source');terminal=None
 # A failed prefix may end after a real attempted command; terminal records it.
 require(ended,'incomplete_terminal')
 return dict(valid=True,aligned=accepted,native_success=False)
