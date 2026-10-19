"""Read-only numeric capture validator/frame exporter. Does not render or simulate."""
import argparse,hashlib,json,math,pathlib

def load(p):return json.loads(p.read_text())
def finite(v):
 if isinstance(v,bool) or v is None or isinstance(v,str):return
 if isinstance(v,(float,int)):
  if not math.isfinite(v):raise ValueError('nonfinite_numeric_state')
 elif isinstance(v,list):
  for x in v:finite(x)
 elif isinstance(v,dict):
  for x in v.values():finite(x)
 else:raise ValueError('invalid_numeric_state')
def pose(p):
 if set(p)!= {'p','q'} or len(p['p'])!=3 or len(p['q'])!=4 or not any(p['q']):raise ValueError('raw_pose_shape')
 finite(p)
def inspect(directory,frame=None):
 d=pathlib.Path(directory);receipt=load(d/'numeric-capture-receipt.json');defs=load(d/'visual-definitions.json');ids=set(defs['identity']);count=0;digest=hashlib.sha256();selected=None;previous=-1
 with (d/'render-state.jsonl').open('rb') as f:
  for line in f:
   digest.update(line);r=json.loads(line);finite(r)
   if r['schema']!='numeric-render-frame/1' or r['frame']!=count or r['physics_step']<previous:raise ValueError('frame_order')
   if set(r['objects'])!=ids or set(r['robots'])!={'left','right'}:raise ValueError('frame_identity')
   for oid,obj in r['objects'].items():
    if obj['actor_name']!=defs['identity'][oid]['actor_name']:raise ValueError('actor_identity')
    pose(obj['pose'])
   for robot in r['robots'].values():
    pose(robot['root_pose'])
    if len(robot['q'])!=9 or len(robot['qdot'])!=9 or len(robot['joint_names'])!=9:raise ValueError('joint_shape')
   for camera in r['cameras'].values():
    pose(camera['world_pose']);p=camera['projection']
    if p['width']<=0 or p['height']<=0 or not 0<p['fovy']<math.pi or not 0<p['near']<p['far']:raise ValueError('projection')
   if count==frame:selected=r
   previous=r['physics_step'];count+=1
 if count!=receipt['frames'] or digest.hexdigest()!=receipt['sha256']:raise ValueError('capture_hash_or_count')
 if frame is not None and selected is None:raise ValueError('missing_frame')
 return dict(schema='offline-render-input/1',frames=count,verified_numeric_sha256=digest.hexdigest(),frame=selected,definitions=defs if selected is not None else None,rendered=False,native_executed=False,formal_credit=False,goal_complete=False,instructions='Apply recorded raw state in a separate render-only scene with bound URDF/visual assets and camera projection/world poses; do not advance physics or interpolate unspecified training frames. Export actual renderer K/extrinsics and per-frame native segmentation-ID mapping. Never label re-simulation as this trajectory.')
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('artifacts',type=pathlib.Path);p.add_argument('--frame',type=int);a=p.parse_args();print(json.dumps(inspect(a.artifacts,a.frame),indent=2,allow_nan=False))
