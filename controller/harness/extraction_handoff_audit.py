"""Independent extraction handoff identity, route and certificate verification."""
import numpy as np
from provenance import digest
from return_clearance import ReturnClearance
from motion_refinement import verify_mapping

def require(value,reason):
    if not value:raise ValueError('extraction_handoff_'+reason)

def verify(events,world):
    groups=[];active=None
    for row in events:
        if row['kind']=='extraction_release_transition' and row['stage']=='loaded_exit':
            if active is not None:groups.append(active)
            active=[]
        if active is not None:active.append(row)
        if row['kind']=='extracted' and active is not None:groups.append(active);active=None
    if active is not None:groups.append(active)
    reports=[]
    for group in groups:
        observations=[e for e in group if e['kind']=='extraction_release_transition'];first=observations[0];arm=first['arm'];target=first['object_id']
        expected_stages=['loaded_exit','released_open']+['wait_tick']*len([e for e in observations if e['stage']=='wait_tick'])+['retreat_start','retreat_end']
        require([e['stage'] for e in observations]==expected_stages[:len(observations)],'stage_order')
        for e in observations:
            require(e['arm']==arm and e['object_id']==target,'identity')
            require(digest(e['state'])==e['state_sha256'] and digest(e['controls'])==e['controls_sha256'],'readback_hash')
            require(e['drive_active']==(e['stage']=='loaded_exit') and e['held_id']==(target if e['stage']=='loaded_exit' else None),'release_identity')
        opened=[e for e in observations if e['stage']=='released_open'];wait=[e for e in observations if e['stage']=='wait_tick'];starts=[e for e in observations if e['stage']=='retreat_start'];ends=[e for e in observations if e['stage']=='retreat_end']
        require(len(opened)<=1 and len(starts)<=1 and len(ends)<=1 and len(wait)<=250,'stage_count')
        if opened:
            require(opened[0]['physics_step']==first['physics_step'],'release_step')
            for i,e in enumerate(wait):
                expected=first['physics_step']+i
                require(e['attempted_step']==expected,'wait_attempt')
                require(e['physics_step']==expected+1 or (e['failure'] is not None and e['physics_step']==expected),'wait_continuity')
                if e['failure'] is not None:require(i==len(wait)-1 and not starts and not ends,'execution_after_wait_failure')
                require(e['controls']==opened[0]['controls'],'wait_drive_changed')
        if starts:require(len(wait)==250 and starts[0]['state']==wait[-1]['state'] and starts[0]['controls']==wait[-1]['controls'],'retreat_start')
        bindings=[e for e in group if e['kind']=='extraction_retreat_binding'];require(len(bindings)<=1,'binding_count')
        if bindings:
            e=bindings[0];require(bool(starts),'missing_start');require(e['state']==starts[0]['state'],'state_bridge')
            require(digest(e['state'])==e['state_sha256'] and digest(e['controls'])==e['controls_sha256'],'binding_hash')
            converted={a:[dict((('velocity' if k=='velocity_target' else k),v) for k,v in j.items()) for j in js] for a,js in e['controls'].items()}
            require(converted==starts[0]['controls'],'retained_controls')
            path=e['measured_path'];q=e['state']['robots'][arm]['q'][:7]
            require(path[0]==q,'measured_reference')
            drives={j['name']:j['target'][0] for j in e['controls'][arm]};drive=[drives['panda_joint'+str(i)] for i in range(1,8)]
            require(drive==e['drive_q'],'drive_readback')
            expected=path if path[0]==drive else [drive]+path
            require(e['route']==expected and digest(expected)==e['route_sha256'],'explicit_drive_prefix')
            plans=[x for x in group if x['kind']=='custom_plan' and x.get('target')==target]
            require(len(plans)==1 and plans[0]['path']['path']==path and plans[0]['path']['success'],'original_planner_path')
            c=ReturnClearance(world(e['state']),arm,target,True);admission=c.monitor(e['state'],drive)
            require(admission==e['admission'],'initial_monitor')
            if admission['safe']:require(c.certify(expected)==e['certificate'],'swept_certificate')
            else:require(e['certificate'] is None,'unsafe_initial_certificate')
            if not admission['safe'] or not e['certificate']['certified']:
                require(not any(x['kind'] in ('motion_phase_entry','return_motion_plan') and x.get('motion_phase')=='retreat' for x in group),'execution_after_rejection')
            refined=[x for x in group if x['kind']=='motion_refinement_prepared' and x['motion_phase']=='retreat']
            for x in refined:
                require(x['original_vertices']==expected,'refined_origin')
                verify_mapping(expected,x['refined_vertices'],x['mapping']['subdivisions'],x['mapping']['original_vertex_indices'])
                require(c.certify(x['refined_vertices'])==x['fresh_certificate'],'refined_certificate')
                plans=[r for r in group if r['kind']=='return_motion_plan' and r['motion_phase']=='retreat']
                if plans:require(len(plans)==1 and plans[0]['vertices']==x['refined_vertices'],'executed_refined_path')
            if ends:
                require(admission['safe'] and e['certificate']['certified'],'unsafe_completion')
                exits=[x for x in group if x['kind']=='motion_phase_exit' and x['motion_phase']=='retreat']
                require(len(exits)==1 and not exits[0]['failed'] and exits[0]['actual_velocity_targets']==[0.]*7,'strict_retreat_exit')
                require(ends[0]['physics_step']==exits[0]['final_scene_step'],'retreat_end_clock')
                require(ends[0]['state_sha256']==exits[0]['state_sha256'],'retreat_end_state')
                native_controls={a:[{('velocity_target' if k=='velocity' else k):v for k,v in j.items()} for j in rows] for a,rows in ends[0]['controls'].items()}
                require(digest(native_controls)==exits[0]['controls_sha256'],'retreat_end_controls')
                require(all(j['velocity']==[0.] for j in ends[0]['controls'][arm][:7]),'retreat_terminal_velocity')
        if any(e['kind']=='extracted' for e in group):require(len(ends)==1 and len(bindings)==1,'extraction_without_retreat')
        reports.append(dict(object_id=target,wait_ticks=len(wait),retreat_bound=bool(bindings),retreat_completed=bool(ends)))
    require(sum(len([e for e in g if e['kind']=='extraction_retreat_binding']) for g in groups)==len([e for e in events if e['kind']=='extraction_retreat_binding']),'orphan_binding')
    extracted=[e for e in events if e['kind']=='extracted']
    require(len([r for r in reports if r['retreat_completed']])>=len(extracted),'missing_handoff')
    return dict(valid=True,motions=reports,full_trajectory_replay=False)
