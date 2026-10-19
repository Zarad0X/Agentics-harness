"""Derive010 actuator receipt inputs from ordered011 caller/native evidence.
No calls to emitter/Route/goal helpers, and no global pool of path approvals.
"""
import numpy as np
from collision_world import pose_matrix
from provenance import digest

def require(ok,why):
 if not ok:raise ValueError('actuator_source_'+why)

def attachment(e):
 ee=np.asarray(e['ee_transform']);obj=np.asarray(e['object_transform']);relative=np.linalg.inv(ee)@obj
 require(ee.shape==obj.shape==(4,4) and np.isfinite([ee,obj,relative]).all(),'transform')
 require(obj[:3,3].tolist()==e['state']['objects'][str(e['object_id'])]['pose'][:3],'object_translation')
 require(relative.tolist()==e['relative'],'held_formula')
 require(e['origin'] in ('magnet_attachment','settled_separation'),'held_origin')
 return e

def bind(receipt,source,plan,held):
 r=receipt;s=source
 require(s is not None and plan is not None,'missing_source')
 require(digest({k:v for k,v in s.items() if k!='sha256'})==s['sha256'] and r['caller_sha256']==s['sha256'],'source_hash')
 arm=s['arm'];target=s['object_id'];require((r['arm'],r['object_id'])==(arm,target),'identity')
 require(plan['arm']==arm and plan.get('object_id',plan.get('target'))==target,'ordered_plan_identity')
 path=plan['path']['path'] if plan['kind']=='custom_plan' else (plan['candidate']['path'] if plan['kind']=='separation_actuator_source' else plan['path'])
 require(r['path']==s['path']==path and r['state']==s['state'] and r['bridge']==s['bridge'],'path_state')
 args=s['arguments'];profile=dict(max_velocity=args['max_velocity'],dt=.004,max_acceleration=args['acceleration'] if args['smooth'] else None)
 require(r['profile']==profile and r['allow_target_contact']==args['allow_target_contact'],'profile_contact')
 previous=[x['target'][0] for x in s['controls'][arm][:7]]
 require(r['previous']==previous,'retained_controls')
 bias=np.asarray(previous)-np.asarray(path[0]) if args['preserve_bias'] else np.zeros(7)
 require(r['bias']==bias.tolist(),'bias_formula')
 if s['loaded']:
  require(s['held_id']==target and s['drive_active'] is True and held is not None,'loaded_attachment')
  require(held['arm']==arm and held['object_id']==target,'held_identity')
  if r['separation'] is not None:
   relative=(np.linalg.inv(np.asarray(s['native_ee']))@pose_matrix(s['state']['objects'][str(target)]['pose'])).tolist()
  else:relative=held['relative']
  require(r['held_relative']==relative,'held_reference')
 else:require(r['held_relative'] is None,'unloaded_geometry')
 if 'caller' not in plan:require(plan['caller_phase']==s['phase'] and plan['returning']==s['returning'],'separation_phase')
 if 'caller' in plan:
  c=plan['caller'];require(c['phase']==s['phase'] and c['returning']==s['returning'],'ordered_phase');require(c['held_relative']==r['held_relative'] and c['allow_target_contact']==r['allow_target_contact'],'plan_geometry_policy')
  if r['bridge'] is None:require(c['state']==s['state'],'planning_state')
  else:require(path[0]==c['state']['robots'][arm]['q'][:7],'prebridge_reference')
 return True
