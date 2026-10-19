"""Independent handoff evidence checks; never grants physical acceptance."""
import copy
import math
from provenance import digest

def require(ok,why):
    if not ok:raise ValueError('capture_handoff_audit_'+why)

def verify_handoff(r):
    require(digest({k:v for k,v in r.items() if k!='sha256'})==r['sha256'],'hash')
    require(r['schema']=='capture-handoff/018-1','schema')
    if r['status']=='failed':
        require(bool(r.get('failure')),'failure_reason');return False
    require(r['status']=='accepted' and r['writes_attempted'] is True,'status')
    require(r['arm'] in ('left','right'),'arm')
    slots=[7,8] if r['arm']=='left' else [16,17]
    require(r['slots']==slots,'slots')
    require(r['context']['held_id'] is None and r['context']['drive_active'] is False,'unloaded')
    before=r['before_controls'];expected=copy.deepcopy(before)
    require(len(before)==18,'controls')
    for i in slots:
        c=before[i]
        require(c[:4] in ([1000.,200.,4.,'force'],[2000.,280.,3.4028234663852886e38,'force']),'input_properties')
        require(len(c[4])==len(c[5])==1 and all(math.isfinite(x) for x in c[4]+c[5]) and
                0<=c[4][0]<=.04 and abs(c[5][0])<=.0020000001,'input_command')
        expected[i]=[1000.,200.,4.,'force',c[4],[0.]]
    require(r['intended_controls']==expected and r['after_controls']==expected,'actual_commands')
    require(r['before_state']==r['after_state'],'physical_state')
    return True

def bind_capture(r,s):
    require(r is not None and verify_handoff(r),'missing_successful_handoff')
    require(s.get('gripper_handoff_sha256')==r['sha256'],'source_handoff_hash')
    require(all(s[k]==r[k] for k in ('arm','object_id','candidate_sha256','slots','context')),'caller')
    require(s['controls']==r['after_controls'],'capture_controls')
    state=r['after_state'];arm=s['arm'];initial=s['initial'];obj=state['objects'][str(s['object_id'])]
    require(initial['finger_position']==state['robots'][arm]['q'][-2:] and
            initial['finger_velocity']==state['robots'][arm]['qdot'][-2:],'native_fingers')
    require(initial['object_pose']==obj['pose'] and initial['object_linear_velocity']==obj['linear_velocity'] and
            initial['object_angular_velocity']==obj['angular_velocity'],'native_object')
