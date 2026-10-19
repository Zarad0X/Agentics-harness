"""Read-only independent terminal-state audit for assisted Real10 episodes."""
import argparse,json,math,hashlib
from pathlib import Path
import numpy as np

IDS={3,4,8,9,10,11,12,17,18,19}

def transform(pq):
    p=np.asarray(pq,float);q=p[3:]/np.linalg.norm(p[3:]);w,x,y,z=q
    return np.array([[1-2*y*y-2*z*z,2*x*y-2*z*w,2*x*z+2*y*w],
                     [2*x*y+2*z*w,1-2*x*x-2*z*z,2*y*z-2*x*w],
                     [2*x*z-2*y*w,2*y*z+2*x*w,1-2*x*x-2*y*y]]),p[:3]

def audit(run,meshes,expected):
    result=json.loads((run/'result.json').read_text());terminal=json.loads((run/'final-state.json').read_text())
    events=[json.loads(s) for s in (run/'events.jsonl').read_text().splitlines()]
    done=terminal['completed_ids'];errors=[]
    if result.get('geometry_source')=='live_PhysX_cooked_convex_shapes':
        mesh_file=run/'physics-meshes.json'
        if (not mesh_file.is_file() or hashlib.sha256(mesh_file.read_bytes()).hexdigest()!=result.get('physics_mesh_sha256')
                or json.loads(mesh_file.read_text())!=meshes):errors.append('physics_geometry_mismatch')
    if len(done)!=expected or len(set(done))!=len(done) or not set(done)<=IDS:errors.append('completed_id_count')
    if expected==10 and set(done)!=IDS:errors.append('missing_real10_ids')
    if result['failure'] or result['forbidden_legacy_calls']:errors.append('runner_failure_or_legacy_call')
    if terminal['held_id'] is not None or terminal['drive_active']:errors.append('assistance_still_attached')
    if set(map(int,terminal['state']['objects']))!=IDS:errors.append('scene_object_ids_changed')
    initial=next(e['state'] for e in events if e['kind']=='initial_state')
    for arm in ('left','right'):
        q0=initial['robots'][arm]['q'][:7]
        q=terminal['state']['robots'][arm]['q'][:7]
        if not np.isfinite(q).all() or len(q)!=7 or max(abs(a-b) for a,b in zip(q,q0))>.025:errors.append('arm_not_returned:'+arm)
    records=[]
    for n in done:
        o=terminal['state']['objects'][str(n)];r,p=transform(o['pose'])
        points=np.concatenate([np.asarray(part['vertices'])@r.T+p for part in meshes[str(n)]])
        target=np.asarray(terminal['placement_targets'][str(n)])
        in_catch=((-0.85<=p[0]<=-.31) or (.29<=p[0]<=.67)) and -.74<=p[1]<=.34
        error=float(np.linalg.norm(p[:2]-target[:2]));bottom=float(points[:,2].min())
        speed=float(np.linalg.norm(o['linear_velocity']));spin=float(np.linalg.norm(o['angular_velocity']))
        lifted=any(e['kind']=='lifted' and e['object_id']==n and e['bottom_z']>=.925 for e in events)
        attached=any(e['kind']=='magnet_attached' and e['object_id']==n for e in events)
        released=any(e['kind']=='released' and e['object_id']==n and e['settled'] for e in events)
        passed=in_catch and error<=.07 and .737<=bottom<=.753 and speed<=.03 and spin<=.15 and lifted and attached and released
        if not passed:errors.append('placement_failed:'+str(n))
        records.append(dict(object_id=n,passed=bool(passed),slot_error_m=error,bottom_z=bottom,speed=speed,spin=spin,lifted=lifted,attached=attached,released=released))
    return dict(accepted=not errors,full_ten_success=not errors and expected==10,expected_count=expected,errors=errors,objects=records)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('run',type=Path);ap.add_argument('--meshes',required=True,type=Path);ap.add_argument('--expected-count',type=int,default=10);a=ap.parse_args()
    try:r=audit(a.run,json.loads(a.meshes.read_text()),a.expected_count)
    except Exception as e:r=dict(accepted=False,full_ten_success=False,error=type(e).__name__+':'+str(e))
    path=a.run/'independent-verification.json'
    if path.exists():raise SystemExit('preserve prior verification')
    path.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));raise SystemExit(0 if r['accepted'] else 2)
