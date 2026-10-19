"""Headless presentation/capture adapter; never substitutes simulation or control."""
import hashlib,json,math,pathlib,time
from headless_factory import plain,install as install_factory,Description

def write(path,value):
 with pathlib.Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')
def pose(actor):return plain(actor.get_pose())

class NoVideo:
 def append_data(self,*args):raise RuntimeError('headless_video_callback_forbidden')
 def close(self):pass

class Visual:
 def __init__(self,out,binding,max_steps,cadence=25,max_frames=100000,admission=False):
  if type(cadence) is not int or cadence<1 or type(max_frames) is not int or max_frames<2:raise ValueError('capture_budget')
  self.out=pathlib.Path(out);self.binding=binding;self.max_steps=max_steps;self.cadence=cadence;self.max_frames=max_frames;self.admission=admission
  self.environments={};self.initial={};self.frames=0;self.pending=[];self.last_frame=None;self.last_picture=None;self.stream=None;self.records=[];self.stats={};self.digest=hashlib.sha256();self.closed=False
 def gpu_binding(self):return dict(uuid='',index=None,pci_bus_id=None,headless=True,gpu_queried=False)
 def render_binding(self):return dict(mode='headless',renderer_created=False,video_deferred=True,**self.binding)
 def install(self,sapien,base,pair_probe,cfg):
  if cfg.mode=='layout':raise ValueError('headless_continuous_requires_roundtrip_not_layout_demo')
  roles=('formal','grasp_preview','separation_preview')+ (('continuation_preview',) if pair_probe else ())
  self.sapien=sapien;self.records,self.stats=install_factory(sapien,base,self.max_steps,roles)
 def bind(self,role,env):
  if role in self.environments or env.scene.headless_role!=role:raise RuntimeError('scene_role_binding')
  if any(env.scene.physx_system is e.scene.physx_system for e in self.environments.values()):raise RuntimeError('aliased_scene')
  self.environments[role]=env;self.initial[role]=self.stats['steps_by_role'][role]
 def initialization_steps(self):return sum(self.initial.values())
 def writer(self,out):
  if pathlib.Path(out)!=self.out:raise ValueError('capture_output_binding')
  self.stream=(self.out/'render-state.jsonl').open('x',buffering=1024*1024)
  return NoVideo()
 def capture_contract(self):
  return dict(schema='numeric-render-capture/1',physics_dt=.004,capture_every_steps=self.cadence,nominal_hz=250/self.cadence,maximum_frames=self.max_frames,buffer_frames=32,rgbd_generated=False,video_generated=False,offline_render_required=True,full_rgbd_training_cadence_claim=False,forced_initial_terminal_frames=True,camera_pose_convention='SAPIEN world p + wxyz, +x forward +y left +z up',calibration='width/height/fovy/near/far from original constructor; native K/extrinsic GL/CV pending offline renderer readback; not fabricated',instance_segmentation='semantic object ID -> actor name; native renderer IDs must be exported by offline render, not inferred')
 def picture_checkpoint(self,env,step):
  if step and step%25==0 and self.last_picture!=step:
   # Exact original update preserves any light-randomization RNG draws. No picture.
   env._update_render();self.last_picture=step
 def cameras(self,env):
  c=env.cameras
  # Exact original method maps robot camera link world poses directly, no new offset.
  c.update_wrist_camera(env.robot.left_camera.get_pose(),env.robot.right_camera.get_pose())
  items=[(n,cam,'static_world') for n,cam in zip(c.static_camera_name,c.static_camera_list)]
  if c.collect_wrist_camera:
   items += [('left_camera',c.left_camera,'left:'+env.robot.left_camera.get_name()),('right_camera',c.right_camera,'right:'+env.robot.right_camera.get_name())]
  result={}
  for name,cam,mount in items:
   if name in result or not isinstance(cam,Description) or not hasattr(cam.entity,'pose'):raise RuntimeError('camera_description_missing_or_duplicate')
   k=cam.kwargs
   if not all(n in k for n in ('width','height','fovy','near','far')):raise RuntimeError('camera_calibration_missing')
   result[name]=dict(world_pose=cam.entity.pose,mount=mount,projection={n:k[n] for n in ('width','height','fovy','near','far')})
  return result
 def capture(self,env,step,phase,held,drive,force=False):
  if self.closed:raise RuntimeError('capture_after_close')
  if env is not self.environments['formal']:raise RuntimeError('capture_nonformal_scene')
  self.picture_checkpoint(env,step)
  if not force and step%self.cadence:return
  key=(step,phase,held,drive)
  if self.last_frame==key:return
  if self.frames>=self.max_frames:raise RuntimeError('numeric_capture_frame_budget')
  robots={}
  for arm in ('left','right'):
   a=getattr(env.robot,arm+'_entity')
   robots[arm]=dict(root_pose=plain(a.get_root_pose()),q=plain(a.get_qpos()),qdot=plain(a.get_qvel()),joint_names=[j.get_name() for j in a.get_active_joints()])
  objects={}
  for oid,obj in env.objects.items():
   b=obj.actor.find_component_by_type(self.sapien.pysapien.physx.PhysxRigidDynamicComponent)
   objects[str(oid)]=dict(actor_name=obj.actor.get_name(),pose=pose(obj),linear_velocity=plain(b.get_linear_velocity()),angular_velocity=plain(b.get_angular_velocity()))
  row=dict(schema='numeric-render-frame/1',frame=self.frames,physics_step=step,simulation_time_s=step*.004,phase=phase,held_id=held,drive_active=drive,robots=robots,objects=objects,cameras=self.cameras(env),lights=plain([r for r in self.records if isinstance(r,Description) and 'light' in r.kind and getattr(r,'scene_role',None)=='formal']),ambient=plain(env.scene.ambient_light))
  data=json.dumps(row,sort_keys=True,allow_nan=False)+'\n';self.pending.append(data);self.digest.update(data.encode());self.frames+=1;self.last_frame=key
  if len(self.pending)>=32:self.flush()
 def flush(self):
  if self.pending:
   if self.stream is None:raise RuntimeError('capture_stream_not_open')
   self.stream.writelines(self.pending);self.stream.flush();self.pending=[]
 def manifest(self,identity,config):
  if set(self.environments)!=set(self.stats['scene_roles']):raise RuntimeError('missing_scene_role')
  write(self.out/'visual-definitions.json',dict(schema='offline-render-definitions/1',identity=identity,scene_config=plain(config),descriptions=plain(self.records),bindings=self.binding,contract=self.capture_contract(),robot_visuals='bound unchanged URDF visual nodes/assets; URDF physical creation skips only visual loop'))
  write(self.out/'headless-initialization.json',dict(steps_by_role=self.initial,total=self.initialization_steps(),scene_roles=list(self.environments),independent_systems=True))
  return dict(**self.binding,visual_definitions_sha256=hashlib.sha256((self.out/'visual-definitions.json').read_bytes()).hexdigest(),headless_initialization_sha256=hashlib.sha256((self.out/'headless-initialization.json').read_bytes()).hexdigest(),capture=self.capture_contract(),source_manifest_is_executed_source=True,executed_runner_is_detached_candidate=False,formal_credit=False)
 def admit(self,copy_to_shadow,state):
  source=self.environments['formal'];checks=[]
  for role,destination in self.environments.items():
   if role=='formal':continue
   unchanged=copy_to_shadow(source,destination);unchanged();checks.append(role)
  self.flush()
  write(self.out/'headless-admission.json',dict(schema='headless-continuous-admission/1',copy_checks=checks,source_state=state(source),initialization_steps=self.initial,physics_steps=self.stats,frames=self.frames,accepted=True,formal_credit=False,goal_complete=False,no_formal_grasp=True))
  return 0
 def final_accounting(self,formal_steps,preview_steps):
  counted=self.stats['physics_steps']-self.initialization_steps()
  if counted!=formal_steps+preview_steps:raise RuntimeError('headless_step_accounting_mismatch')
  return dict(**self.binding,scene_steps=self.stats,initialization_steps=self.initial,formal_steps=formal_steps,preview_steps=preview_steps,nested_previews_included_once=True,capture=self.capture_contract())
 def finish(self,exception=None):
  if self.closed:return
  self.flush()
  if self.stream:self.stream.close()
  self.closed=True
  if self.out.exists():write(self.out/'numeric-capture-receipt.json',dict(schema='numeric-render-capture-receipt/1',frames=self.frames,sha256=self.digest.hexdigest(),contract=self.capture_contract(),scene_stats=self.stats,exception=exception,formal_credit=False,goal_complete=False))
