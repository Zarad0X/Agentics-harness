"""Independent representation/readback/geometry reconstruction for010 routes.

Does not call actuator_targets goal, emitted, Route, issue or handoff helpers.
"""
import hashlib
import numpy as np
from provenance import digest
from astra_planner import resample_path
from loaded_tracking import POLICY


def require(ok,why):
    if not ok:raise ValueError('actuator_audit_'+why)


def verify_route(r,world_factory):
    require(r['schema']=='native-actuator-route/010' and r['physical_completion_proved'] is False,'schema')
    require(digest({k:v for k,v in r.items() if k!='sha256'})==r['sha256'],'hash')
    require(digest(r['state'])==r['state_sha256'],'state_hash')
    w=world_factory(r['state']);arm=r['arm'];target=r['object_id'];b=np.asarray(w.model.bounds)+[.03,-.03]
    require(arm in ('left','right'),'arm')
    bridge=r['bridge']
    if bridge is None:require(r['path'][0]==r['state']['robots'][arm]['q'][:7],'measured_reference_start')
    else:require(bridge['state_sha256']==r['state_sha256'] and bridge['path_sha256']==digest(r['path']) and bridge['arm']==arm and bridge['object_id']==target and bridge['reused_approval'] is False,'bridge_binding')
    if r['held_relative'] is not None:w.held=(arm,str(target),np.asarray(r['held_relative']))
    profile=r['profile'];require(profile['dt']==.004,'dt')
    dense=np.asarray(resample_path(r['path'],**profile));reference=dense+np.asarray(r['bias'])
    require(reference.ndim==2 and reference.shape[1]==7 and np.isfinite(reference).all(),'reference')
    require(np.all(reference>=b[:,0]) and np.all(reference<=b[:,1]),'reference_bounds')
    q=reference.astype(np.float32).astype(float);previous=np.asarray(r['previous'])
    require(np.isfinite(q).all() and np.all(q>=b[:,0]) and np.all(q<=b[:,1]),'emitted_bounds')
    require(previous.shape==(7,) and np.isfinite(previous).all() and np.array_equal(previous,previous.astype(np.float32).astype(float)) and np.all(previous>=b[:,0]) and np.all(previous<=b[:,1]),'retained_bounds')
    require(digest(q.tolist())==r['native_targets_sha256'] and len(q)==r['command_count'],'native_targets')
    v=np.vstack((np.diff(dense,axis=0)/.004,np.zeros((1,7))))
    require(np.isfinite(v).all() and np.max(np.abs(v))<=profile['max_velocity']+1e-5,'velocity')
    resolution=.025 if r['separation'] is None else .002
    prefix=np.linspace(previous,q[0],max(2,int(np.ceil(np.max(np.abs(q[0]-previous))/resolution))+1))
    fingers=np.clip(r['state']['robots'][arm]['q'][-2:],0,.04).tolist();count=0
    sep=r['separation']
    if sep is not None:
        require(sep['path']==r['path'],'separation_path');prior=dict(sep['initial_clearance']);touching={k for k,v in prior.items() if v<=2e-5};evidence=sep['initial_contacts']
        require(bool(prior) and all(k in evidence and np.isfinite([evidence[k]['separation_min'],evidence[k]['impulse_max']]).all() and evidence[k]['separation_min']>=-.00005 and evidence[k]['impulse_max']<=.002 for k in touching),'separation_contacts')
    for command in list(prefix)+list(q):
        kwargs={}
        if sep is not None:
            thresholds={k:max(1e-5,min(v,.001)-1e-6) for k,v in prior.items()}
            kwargs=dict(payload_clearance=thresholds,start_contact_pairs=touching if np.max(np.abs(command-np.asarray(r['path'][0])))<1e-10 else ())
        count+=1;require(w.free(arm,command.tolist(),target,r['allow_target_contact'],fingers,**kwargs),'geometry')
        if sep is not None:
            measured=w.payload_clearances(arm,command.tolist(),prior);prior={k:max(prior[k],measured[k]) for k in prior}
    if sep is not None:require(min(w.payload_clearances(arm,q[-1].tolist(),prior).values())>=.003 and w.free(arm,q[-1].tolist(),target,r['allow_target_contact'],fingers),'separation_final')
    require(count==r['geometry_checks'],'geometry_count')
    rows=[np.r_[a,np.asarray(z,np.float32).astype(float)].astype('<f4').tobytes() for a,z in zip(q,v)]
    require(hashlib.sha256(b''.join(rows)).hexdigest()==r['commands_sha256'],'commands_hash')
    return rows,q


def verify_handoff(r,world_factory):
    require(r['schema']=='retained-actuator-handoff/010' and r['physical_completion_proved'] is False,'handoff_schema')
    require(digest({k:v for k,v in r.items() if k!='sha256'})==r['sha256'] and digest(r['state'])==r['state_sha256'] and digest(r['controls'])==r['controls_sha256'],'handoff_hash')
    arm=r['arm'];w=world_factory(r['state']);b=np.asarray(w.model.bounds)+[.03,-.03];s=r['state'];rows=r['controls'][arm][:7]
    require(len(rows)==7 and all(x['name']=='panda_joint'+str(i+1) and x['stiffness']==1000. and x['damping']==200. and x['mode']=='force' for i,x in enumerate(rows)),'actuators')
    t=np.asarray([x['target'][0] for x in rows]);v=np.asarray([x['velocity'][0] for x in rows]);q=np.asarray(s['robots'][arm]['q'][:7])
    require(np.isfinite(t).all() and np.isfinite(v).all() and np.max(np.abs(v))<=POLICY['drive_velocity_rad_s'],'handoff_velocity')
    require(np.max(np.abs(t-q))<=POLICY['bias_limit_rad'],'handoff_bias')
    require(np.all(t>=b[:,0]) and np.all(t<=b[:,1]),'handoff_bounds')
    return t


def verify(events,world_factory,loaded_variant='B_clearance'):
    preview_trials={};preview_count=0;verified_candidates=0;bridges=set();separations=set()
    from actuator_source_audit import bind,attachment
    caller=None;plan=None;held=None;source_event=None
    approved=set();routes={};closed=set();last={};handoffs=0;route_count=0
    for e in events:
        kind=e['kind']
        if kind=='held_transform_source':held=attachment(e)
        if kind in ('custom_plan','insertion_plan','separation_plan','separation_actuator_source'):plan=e
        if kind=='actuator_caller_source':
            caller=e['source'];source_event=e
        if kind=='return_preview':verify(e['trace'],world_factory,loaded_variant)

        if kind=='grasp_preview':
            nested=e['separation_trials']
            audit=verify(nested,world_factory,loaded_variant)
            preview_count+=1
            if e['accepted']:
                require(e['failure'] is None and e['source_unchanged'] is True and e['independent_scene'] is True,'preview_status')
                if loaded_variant=='B_clearance':
                    hand=[x for x in nested if x['kind']=='grasp_actuator_handoff']
                    require(audit['handoffs']==1 and hand[0]['object_id']==e['object_id'],'preview_handoff')
                key=(e['object_id'],e['index']);trial=e['trial']
                require(trial in (1,2) and trial not in preview_trials.setdefault(key,set()),'preview_trial')
                preview_trials[key].add(trial)
        if kind=='grasp_candidate_verified':
            key=(e['object_id'],e['index'])
            require(e['independent_trials']==2 and preview_trials.pop(key,set())=={1,2},'preview_selection')
            verified_candidates+=1
        if kind=='separation_actuator_source':
            source=e['candidate'];approved.add(digest(source['path']))
            separations.add(digest({k:source[k] for k in ('path','initial_clearance','initial_contacts')}))
        if kind=='separation_plan' and e.get('needed') and e.get('success'):
            separations.add(digest({k:e[k] for k in ('path','initial_clearance','initial_contacts')}))
        if kind=='return_handoff_approach_checked':
            bridges.add(digest({k:e[k] for k in ('object_id','arm','state_sha256','path_sha256','samples','reused_approval')}))
        if kind=='custom_plan' and e['path']['success']:approved.add(digest(e['path']['path']))
        if kind in ('insertion_plan','separation_plan') and 'path' in e:approved.add(digest(e['path']))
        if kind=='actuator_route_admission':
            r=e['receipt'];bind(r,caller,plan,held)
            require(all(e.get(k)==source_event.get(k) for k in ('phase','physics_step')),'caller_event_context')
            if e.get('phase') is not None:require(caller['phase']==e['phase'],'native_caller_phase')
            caller=None;h=r['sha256'];require(h not in routes or h in closed,'overlapping_route');closed.discard(h);route_count+=1;require(digest(r['path']) in approved,'unbound_planned_path')
            if r['separation'] is not None:require(digest(r['separation']) in separations,'unbound_separation')
            if r['bridge'] is not None:require(digest(r['bridge']) in bridges,'unbound_bridge')
            rows,q=verify_route(r,world_factory);routes[h]=(r,rows,q)
        if kind=='actuator_route_readback':
            h=e['route_sha256'];require(h in routes and h not in closed,'unbound_readback');r,rows,q=routes[h];n=e['verified_readbacks'];attempts=e['attempted']
            require(e['arm']==r['arm'] and e['object_id']==r['object_id'] and e['physical_completion_proved'] is False,'readback_identity')
            require(type(n) is int and 0<=n<=attempts<=len(rows) and attempts-n<=1,'readback_count')
            require(hashlib.sha256(b''.join(rows[:n])).hexdigest()==e['readback_sha256'],'readback_hash')
            require(e['emission_complete']==(not e['failed'] and n==len(rows)),'readback_completion')
            if e['emission_complete']:last[r['arm']]=(r['object_id'],q[-1].tolist())
            closed.add(h)
        if kind=='grasp_actuator_handoff':
            r=e['receipt'];target=verify_handoff(r,world_factory).tolist()
            require(last.get(r['arm'])==(e['object_id'],target),'handoff_retained_source');handoffs+=1
    require(set(routes)==closed,'missing_route_readback')
    return dict(valid=True,routes=route_count,handoffs=handoffs,previews=preview_count,verified_candidates=verified_candidates,physical_completion_proved=False)
