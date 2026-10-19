"""Frozen opt-in ranking hints. Never a grasp/path or permission cache."""
import json
import numpy as np
from provenance import digest,file_hash


class Experience:
    def __init__(self,path=None,enabled=False):
        self.enabled=enabled;self.records=[];self.snapshot_sha256=None;self.matches=0;self.rank_calls=0;self.used_hints=0;self.reordered_calls=0;self.last_trace={}
        if path is not None:
            self.snapshot_sha256=file_hash(path)
            data=json.loads(path.read_text())
            if data.get('schema') not in ('roundtrip-experience/1','roundtrip-experience/2'):raise ValueError('experience schema')
            if any(r.get('split')=='heldout' for r in data['records']):raise ValueError('heldout records cannot seed policy')
            if data['schema']=='roundtrip-experience/2':
                for r in data['records']:
                    if r.get('split')!='development' or r.get('stage') not in ('extraction','return') or not r.get('provenance'):
                        raise ValueError('unscoped experience')
            self.records=tuple(data['records'])
        if enabled and path is None:raise ValueError('enabled experience requires frozen snapshot')

    def rank(self,candidates,geometry_hash,configuration_hash,scene_descriptor,stage=None,**context):
        self.rank_calls+=1
        self.last_trace=dict(enabled=self.enabled,stage=stage,candidate_count=len(candidates),matched_hints=0,used_hints=0,reordered=False)
        if not self.enabled:return candidates
        x=np.asarray(scene_descriptor,float)
        if x.ndim!=1 or not x.size or not np.isfinite(x).all():return candidates
        matched=[]
        for r in self.records:
            if stage is not None and r.get('stage')!=stage:continue
            y=np.asarray(r['scene_descriptor'],float)
            if r['geometry_sha256']!=geometry_hash or r['configuration_sha256']!=configuration_hash or y.shape!=x.shape:continue
            if not np.isfinite(y).all() or np.max(np.abs(y-x))>.025:continue
            matched.append(r)
        self.matches+=len(matched)
        # Only reorder freshly generated candidates. Failed hints cannot remove
        # the last candidate and every IK/collision/physics check still runs.
        def score(c):
            depth=float(c['surface_depth_m'])
            return sum((1 if r['outcome']=='success' else -1) for r in matched if abs(r['depth_m']-depth)<.001)
        ranked=sorted(candidates,key=lambda c:-score(c))
        used=sum(any(abs(r['depth_m']-float(c['surface_depth_m']))<.001 for c in candidates) for r in matched)
        changed=any(a is not b for a,b in zip(candidates,ranked))
        self.used_hints+=used;self.reordered_calls+=int(changed)
        self.last_trace.update(matched_hints=len(matched),used_hints=used,reordered=changed,
                               source_record_ids=[r.get('record_id') for r in matched])
        return ranked


def policy_hash(config):
    # Retreat-only timing cannot change extraction depth ranking. Full run
    # config and implementation hashes still bind the controller experiment.
    return digest({'compatibility':'stage-depth-ranking/2','settings':{k:v for k,v in config.to_dict().items() if k not in ('experience','optimize_geometry','seed','split','mode','layout','count','smoke_id','retreat_time_scale','high_transfer_branch_seed','velocity_feedback','feedback_scope','feedback_telemetry','vertex_governor','joint_step_refinement','home_coarse','home_contact_envelope','home_progress','home_progress_bound','reserve_home_workspace','home_entry_hold','home_nominal_speed_rad_s','defer_intermediate_home','return_preview_cap','return_time_cap_s')}})


def descriptor(state,target):
    p=np.asarray(state['objects'][str(target)]['pose'][:3]);others=[]
    for oid,o in state['objects'].items():
        if oid!=str(target):others.append(float(np.linalg.norm(np.asarray(o['pose'][:3])-p)))
    return p.tolist()+sorted(others)[:4]+[2.]*(max(0,4-len(others)))
