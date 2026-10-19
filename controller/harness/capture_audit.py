"""Independent capture command auditor; no controller imports or simulation.

Native qualification additionally requires parent's bound native runtime,
assets, whole-episode auditors and live success. Missing full capture stream
or missing independent world factory cannot earn capture acceptance.
"""
import copy
import math
import numpy as np
from provenance import digest


def verify(events,world_factory=None):
    def need(ok,why):
        if not ok:raise ValueError('capture_audit_'+why)
    handoff_receipt=None
    source=None;chain=None;seq=0;ticks=0;controls=None;pending=False;last=None
    latches=[None,None];distance=[0.,0.];rounds=0;active=None;hold=0;sweep=0
    feedback=0;terminals=[];settled=False;hold_interrupted=False;recovery_start=None;permit=None
    pending_kind=None;pending_reason=None;pending_expected=None;pending_distance=None
    def validate(row):
        need(row['held_id'] is None and row['drive_active'] is False,'unloaded')
        fields=[*row['finger_position'],*row['finger_velocity'],row['position_error_m'],row['rotation_error_rad'],
                *row['object_pose'],*row['object_linear_velocity'],*row['object_angular_velocity']]
        need(all(math.isfinite(x) for x in fields),'finite')
        need(len(row['finger_position'])==len(row['finger_velocity'])==2 and
             all(0<=q<=.04 for q in row['finger_position']),'shape_or_travel')
        need(row['position_error_m']<=.005 and row['rotation_error_rad']<=.04,'native_gates')
        current=row['object_pose'];initial=source['initial']['object_pose']
        a=np.array(current[3:]);b=np.array(initial[3:]);need(len(current)==7 and np.linalg.norm(a)>0 and np.linalg.norm(b)>0,'quaternion')
        re=2*math.acos(min(1.,abs(float(a@b/(np.linalg.norm(a)*np.linalg.norm(b))))))
        need(np.linalg.norm(np.array(current[:3])-initial[:3])<=.002 and re<=math.radians(2.),'drift')
        sensed=[]
        for points in row['points']:
            need(all(len(p['impulse'])==3 and all(math.isfinite(x) for x in [p['separation'],*p['impulse']]) for p in points),'contact_finite')
            need(all(p['separation']>=-.0003 for p in points),'target_penetration')
            sensed.append(any(-.0003<=p['separation']<=.0001 and math.hypot(*p['impulse'])>1e-6 for p in points))
        need(len(sensed)==2 and sensed==row['touching'],'same_point_contact')
    def feedback_needed(row,old):
        return (max(map(abs,row['finger_velocity']))>.01 or
                np.linalg.norm(row['object_linear_velocity'])>.03 or np.linalg.norm(row['object_angular_velocity'])>.15 or
                any(t and q-u>.0005 for t,q,u in zip(row['touching'],row['finger_position'],old)))
    def feedback_targets(row,old):
        goals=list(old)
        obj=np.linalg.norm(row['object_linear_velocity'])>.03 or np.linalg.norm(row['object_angular_velocity'])>.15
        for i in range(2):
            q=row['finger_position'][i];t=row['touching'][i]
            if (t and q-old[i]>.0005) or row['finger_velocity'][i]<-.01 or (obj and t):
                goals[i]=max(old[i],max(0.,q-.00025))
        # Independently implemented velocity relief; no controller import.
        for i in range(2):
            if row['touching'][i] and row['finger_velocity'][i]<-.01:
                preload=max(0.,.00025-.004*(-row['finger_velocity'][i]-.01))
                goals[i]=max(goals[i],old[i],max(0.,row['finger_position'][i]-preload))
        answer=[]
        for u,g in zip(old,goals):
            g=float(np.float32(g));request=u+max(-.00004,min(.00004,g-u));v=np.float32(request)
            if g<u and float(v)<request:v=np.nextafter(v,np.float32(np.inf))
            if g>u and float(v)>request:v=np.nextafter(v,np.float32(-np.inf))
            answer.append(float(v))
        return answer
    def geometry(record):
        cert=record['geometry'];need(isinstance(cert,dict),'geometry_missing')
        need(digest(cert['state'])==cert['state_sha256'],'geometry_state_hash')
        need(cert['targets']==record['targets'] and cert['previous_targets']==record['previous_targets'],'geometry_command_binding')
        s=cert['state'];arm=source['arm'];target=source['object_id']
        need(s['robots'][arm]['q'][-2:]==record['before']['finger_position'] and
             s['robots'][arm]['qdot'][-2:]==record['before']['finger_velocity'] and
             s['objects'][str(target)]['pose']==record['before']['object_pose'],'geometry_measurement_binding')
        wanted=[]
        for start in (s['robots'][arm]['q'][-2:],record['previous_targets']):
            count=max(2,int(np.ceil(np.max(np.abs(np.asarray(record['targets'])-start))/.00004))+1)
            need(count<=1002,'geometry_budget')
            wanted.extend(np.linspace(start,record['targets'],count).tolist())
        wanted=[list(v) for v in dict.fromkeys(tuple(v) for v in wanted)]
        need(cert['samples']==wanted,'geometry_samples')
        if world_factory is not None:
            w=world_factory(s)
            need(all(w.free(arm,s['robots'][arm]['q'][:7],target,True,f) for f in wanted),'geometry_replay')
    for event in events:
        kind=event['kind']
        if kind=='gripper_capture_handoff':
            from capture_handoff_audit import verify_handoff
            receipt=event['receipt']
            verify_handoff(receipt)
            handoff_receipt=receipt if receipt['status']=='accepted' else None
            continue
        if kind=='closure_control_source' and source is not None:
            c=event['source']
            need(c['candidate_sha256']==source['candidate_sha256'] and c['arm']==source['arm'] and
                 c['object_id']==source['object_id'] and c['slots']==source['slots'] and
                 c['controls']==controls,'closure_caller_binding')
        if kind in ('grasp_preview','return_preview'):
            nested=event.get('separation_trials',event.get('trace',[]))
            if any(e['kind']=='capture_begin' for e in nested):verify(nested,world_factory)
            elif event.get('accepted'):raise ValueError('capture_audit_accepted_preview_without_capture')
        if kind=='magnet_attached':
            need(permit==event['object_id'],'magnet_without_capture');permit=None
        if kind=='gripper_settle' and event.get('outcome')=='settled':settled=True
        if kind=='capture_begin':
            need(source is None,'unterminated_source');source=event['source']
            need(digest({k:v for k,v in source.items() if k!='sha256'})==source['sha256'],'source_hash')
            from capture_handoff_audit import bind_capture
            bind_capture(handoff_receipt,source)
            handoff_receipt=None
            need(source['schema']=='capture-feedback/018-followup001' and source['arm'] in ('left','right'),'schema')
            need(source.get('feedback_policy')==dict(name='velocity-preload-relief/1',horizon_s=.004,max_preload_reduction_m=.00025,speed_trigger_m_s=.01,contact_required=True),'feedback_policy')
            need(source['slots']==([7,8] if source['arm']=='left' else [16,17]),'arm_slots')
            need(source['contact_identity']=='selected_native_child_links_and_target_entity','sensor_identity')
            controls=copy.deepcopy(source['controls']);need(len(controls)==18,'controls')
            for i in source['slots']:need(controls[i][:4]==[1000.,200.,4., 'force'] and controls[i][5]==[0.],'actuator_contract')
            chain=source['sha256'];seq=ticks=hold=sweep=rounds=feedback=0;pending=False;settled=False
            permit=None
            latches=[None,None];distance=[0.,0.];active=None;last=source['initial'];recovery_start=None
            pending_kind=None;pending_reason=None;pending_expected=None;pending_distance=None
            continue  # rejected admission may have no legal command at all
        if not kind.startswith('capture_'):continue
        need(source is not None,'orphan_record');r=event['record']
        need(r['source_sha256']==source['sha256'] and r['previous_sha256']==chain and r['seq']==seq and
             digest({k:v for k,v in r.items() if k!='sha256'})==r['sha256'],'record_chain')
        chain=r['sha256'];seq+=1
        need(r['ticks']==ticks,'tick_clock')
        if kind=='capture_observation':
            need(pending and pending_kind=='stepped','observation_without_command');pending=False;pending_kind=None
            last=r['observation'];validate(last);need(r['controls_sha256']==digest(controls),'post_controls')
        elif kind=='capture_readback':
            need(pending and pending_kind in ('command','relief') and r['operation']==pending_kind,'readback_scope')
            need(r['controls_sha256']==digest(pending_expected),'native_readback_hash');controls=pending_expected
            if pending_distance is not None:
                i,delta=pending_distance;distance[i]+=delta
            if pending_kind=='relief':pending=False;pending_kind=None
            else:pending_kind='verified'
        elif kind=='capture_tick_started':
            need(pending and pending_kind=='verified','tick_without_readback');ticks+=1;need(ticks<=3002,'total_budget')
            if pending_reason=='sweep':sweep+=1
            if pending_reason=='feedback':feedback+=1;need(feedback<=200,'feedback_budget')
            if pending_reason=='hold':hold+=1
            pending_kind='stepped'
        elif kind=='capture_rebase':
            need(not pending,'rebase_during_step');row=r['observation'];validate(row);i=r['finger']
            need(i in (0,1) and latches[i] is not None and row['touching'][i],'rebase_contact')
            need(row['finger_position'][i]-controls[source['slots'][i]][4][0]>.0005,'rebase_trigger')
            need(r['previous_latch']==latches[i] and r['target']==max(latches[i],max(0.,row['finger_position'][i]-.00025)),'rebase_target')
            latches[i]=r['target'];settled=False
        elif kind=='capture_motion_interrupted':
            validate(r['observation']);row=r['observation']
            need(np.linalg.norm(row['object_linear_velocity'])>.03 or np.linalg.norm(row['object_angular_velocity'])>.15,'false_motion')
            settled=False
        elif kind=='capture_latch':
            need(not pending,'latch_during_step');i=r['finger'];validate(r['observation'])
            need(r['observation']['touching'][i] and r['target']==max(0.,min(.04,r['observation']['finger_position'][i]-.00025)),'latch_sensor')
            if r['renewal']:need(active==i and recovery_start is not None and ticks-recovery_start<=100,'renewal_scope')
            else:need(latches[i] is None,'first_latch_repeated')
            latches[i]=r['target'];active=None;recovery_start=None
        elif kind=='capture_loss':
            validate(r['observation']);i=r['finger']
            need(latches[i] is not None and r['stale_latch']==latches[i] and not r['observation']['touching'][i],'loss_sensor')
            active=i;settled=False
        elif kind=='capture_reacquire':
            need(r['finger']==active and active in (0,1),'recovery_scope');rounds+=1
            need(rounds<=2 and r['round']==rounds and r['held_target']==controls[source['slots'][1-active]][4][0],'recovery_bound')
            recovery_start=ticks
        elif kind=='capture_hold_interrupted':
            validate(r['observation']);need(not all(r['observation']['touching']),'false_loss');settled=False
        elif kind in ('capture_command','capture_relief'):
            need(not pending,'command_without_observation');validate(r['before']);geometry(r)
            old=[controls[i][4][0] for i in source['slots']];new=r['targets']
            need(old==r['previous_targets'] and len(new)==2 and all(math.isfinite(x) and 0<=x<=.04 for x in new),'targets')
            need(all(abs(a-b)<=.00004+1e-15 and float(np.float32(b))==b for a,b in zip(old,new)),'slew')
            touch=r['before']['touching']
            if kind=='capture_relief':
                need(all(touch),'unilateral_relief')
                p=np.asarray(r['before']['finger_position']);o=np.asarray(old)
                req=np.maximum(o,np.minimum(np.minimum(.04,o+.00004),p-.00025));n=req.astype(np.float32)
                n=np.where(n.astype(float)>req,np.nextafter(n,np.float32(-np.inf)),n).astype(float)
                need(new==np.maximum(o,n).tolist() and (r['elapsed_ticks']<100 or max(map(abs,r['before']['finger_velocity']))>.01),'relief_formula')
                settled=False
            elif r['reason']=='feedback':
                need(feedback<200 and feedback_needed(r['before'],old),'feedback_scope')
                need(new==feedback_targets(r['before'],old) and all(b>=a for a,b in zip(old,new)),'feedback_formula')
                settled=False
            elif r['reason']=='reacquire':
                need(not feedback_needed(r['before'],old),'feedback_required')
                i=r['active'];need(i==active and recovery_start is not None and ticks-recovery_start<100,'recovery_clock')
                need(touch==[j!=i for j in range(2)] and new[1-i]==old[1-i] and new[i]<=old[i],'one_sided_command')
                need(r['before']['finger_position'][1-i]-old[1-i]<=.0005,'held_preload')
                need(new[i]>=min(old[i],r['before']['finger_position'][i]-.00025),'missing_preload')
                need(distance[i]+old[i]-new[i]<=.001+1e-15,'recovery_distance')
            elif r['reason']=='sweep':
                need(not feedback_needed(r['before'],old),'feedback_required')
                need(sweep<2002,'sweep_budget')
                for i in range(2):
                    need(latches[i] is None or touch[i],'stale_latch_command')
                    goal=0. if latches[i] is None else float(np.float32(latches[i]))
                    need(min(old[i],goal)<=new[i]<=max(old[i],goal),'sweep_direction')
                if any(touch) and not all(touch):
                    i=touch.index(True);need(r['before']['finger_position'][i]-old[i]<=.0005,'sweep_preload')
            elif r['reason']=='hold':
                need(hold<1000 and new==old and all(touch),'hold_contract')
            else:raise ValueError('capture_audit_unknown_command')
            pending_expected=copy.deepcopy(controls)
            for i,v in zip(source['slots'],new):pending_expected[i][4]=[v]
            need(r['controls_sha256']==digest(pending_expected),'command_controls')
            pending=True;pending_kind='relief' if kind=='capture_relief' else 'command';pending_reason=r.get('reason')
            pending_distance=(r['active'],old[r['active']]-new[r['active']]) if r.get('reason')=='reacquire' else None
            if kind=='capture_command':need(ticks<3002,'total_budget')
        elif kind=='capture_terminal':
            need(0<=r['last_observation_tick']<=ticks,'last_observation_clock')
            need(r['sweep_ticks']==sweep and r['hold_ticks']==hold and r['recovery_rounds']==rounds,'terminal_counts')
            need(r['recovery_distance']==distance,'terminal_distance')
            need(r['feedback_ticks']==feedback and feedback<=200,'terminal_feedback')
            if r['status']=='accepted':
                need(r['last_observation_validated'] and not pending and settled and ticks<=3002 and r['last_observation_tick']==ticks and all(r['last_observation']['touching']),'terminal_acceptance')
                validate(r['last_observation']);need(max(map(abs,r['last_observation']['finger_velocity']))<=.01,'final_native_speed')
                need(np.linalg.norm(r['last_observation']['object_linear_velocity'])<=.03 and np.linalg.norm(r['last_observation']['object_angular_velocity'])<=.15,'final_object_speed')
                need(world_factory is not None,'geometry_factory_required_for_acceptance')
                permit=source['object_id']
            else:need(r['status']=='failed' and r['failure'] is not None,'terminal_failure')
            terminals.append(dict(status=r['status'],ticks=ticks));source=None
        else:raise ValueError('capture_audit_unknown_event')
    need(source is None and bool(terminals),'missing_terminal')
    # Retains the independently implemented original closure/dwell checks.
    from closure_audit import verify as closure_verify
    closure_verify(events)
    return dict(valid=True,terminals=terminals,geometry_replayed=world_factory is not None,native_qualified=False)
