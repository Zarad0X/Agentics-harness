"""Read-only, streamed loaded witness replay; never substitutes physical audit."""
import copy,hashlib,json,lzma,math
from pathlib import Path
from dataclasses import asdict
import numpy as np
from provenance import file_hash,digest as state_digest
from loaded_common import digest
from loaded_clearance import PayloadAdmission,Monitor,Envelope,ee
from live_clearance import PROFILE,SMOOTH_PROFILE,profile_for,fixed_controls
from loaded_geometry import verify as verify_obstacles
from loaded_science import CONTRACT,RAW_CAP,XZ_CAP,ROW_CAP,ROWS_CAP
from direct_core import ForwardModel
from return_world import ReturnWorld
from physics_geometry import bounds_for_parts
from collision_world import pose_matrix
from astra_planner import so3_log
from placement_audit_v30 import expected_binding
from loaded_tracking_audit import Replay,equivalent
from loaded_tracking_audit005 import Replay as Replay005,CONTRACT as CONTRACT005

def streamed_hash(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024**2),b''):h.update(block)
    return h.hexdigest()

def require(ok,label):
    if not ok:raise ValueError(label)

def replay_commands(path,profile):
    # Independent reconstruction of the executed scalar clock and forward
    # difference velocity targets. Do not call runtime resample_path/commands.
    p=np.asarray(path,float);dense=[p[0].tolist()];dt=profile['dt'];vmax=profile['max_velocity'];acc=profile['max_acceleration']
    for a,z in zip(p[:-1],p[1:]):
        d=float(np.max(np.abs(z-a)))
        ratio=d/(vmax*dt) if acc is None else max(1.875*d/vmax,math.sqrt(5.78*d/acc))/dt
        n=max(1,math.ceil(ratio))
        for i in range(1,n+1):
            u=i/n;clock=u if acc is None else u**3*(10-15*u+6*u*u)
            dense.append((z if i==n else a+(z-a)*clock).tolist())
    q=np.asarray(dense);v=np.vstack(((q[1:]-q[:-1])/dt,np.zeros(7)))
    return [dict(q=a.tolist(),v=b.tolist()) for a,b in zip(q,v)]+[dict(q=q[-1].tolist(),v=[0.]*7) for _ in range(1100)]

def start(row,world_factory,variant,expected_profile=None,legacy=False):
    b=row['binding'];s=row['state'];c=row['controls'];arm=row['arm']
    require(row['variant']==variant and row['phase'] in ('transport','lowering'),'loaded_motion_identity')
    require(digest(b)==row['binding_sha256'] and b['state']==digest(s) and b['controls']==digest(c),'loaded_binding_hash')
    profile=profile_for(variant) if expected_profile is None else expected_profile
    require(b['envelope']==asdict(Envelope()) and b['profile']==profile,'loaded_envelope_profile')
    expected=replay_commands(b['path'],profile)
    require(row['commands']==expected and digest(expected)==b['commands_sha256'],'loaded_final_commands')
    require(b['path'][0]==s['robots'][arm]['q'][:7],'loaded_initial_handoff')
    a=PayloadAdmission(world_factory(s),arm,row['target'],b['relative'])
    require(digest(a.world.meshes)==b['geometry'],'loaded_mesh_hash')
    verify_obstacles(a,b,legacy=legacy)
    native=np.asarray(row['native_ee']);relative=np.linalg.inv(native)@pose_matrix(s['objects'][str(row['target'])]['pose'])
    require(np.allclose(relative,b['relative'],rtol=0,atol=1e-12),'loaded_measured_grasp')
    fk=ee(a.world.model,arm,s['robots'][arm]['q'][:7])
    require(np.linalg.norm(fk[:3,3]-native[:3,3])<=2e-5 and np.linalg.norm(so3_log(fk[:3,:3]@native[:3,:3].T))<=1e-4,'loaded_FK_frame')
    if variant=='B_clearance':
        certificate=a.certify(b['path']);require(certificate['safe'] and certificate==row['certificate'],'loaded_certificate_replay')
    else:require(row['certificate']=={'safe':None,'reason':'original_monitor_only'},'loaded_A_not_clearance_claim')
    monitor=Monitor(a);initial=monitor.measure(s,native,expected[0]['q'])
    tracking=(Replay005 if profile['tracking']==CONTRACT005 else Replay)(a,expected,s,c,profile) if 'tracking' in profile else None
    return dict(row=row,a=a,monitor=monitor,commands=expected,index=0,fixed=fixed_controls(c,arm),violations=list(initial['violations']),maxima={},tracking=tracking,previous_state=s,previous_controls=c)

def tick(active,row):
    a=active['a'];i=active['index'];start_row=active['row'];c=row['controls']
    require(row['variant']==start_row['variant'] and row['target']==int(a.target) and row['arm']==a.arm and row['phase']==start_row['phase'],'loaded_tick_identity')
    tracking=active['tracking']
    if tracking is None:
        require(type(row['index']) is int and row['index']==i and i<len(active['commands']) and row['command']==active['commands'][i],'loaded_tick_command')
        reference=row['command']['q']
    else:
        d=tracking.next(active['previous_state']);w=row['tracking']
        require(type(row['index']) is int and row['index']==i and equivalent(d,w['decision']),'loaded_controller_replay')
        require(w['pre_state_sha256']==digest(active['previous_state']) and w['pre_controls_sha256']==digest(active['previous_controls']),'loaded_precommand_binding')
        require(w['ordering']=='command_readback_then_passive_qf_then_readback_then_scene_step','loaded_ordering_contract')
        require(set(w['pre_step_qf'])=={'left','right'} and all(len(w['pre_step_qf'][arm])==len(active['previous_state']['robots'][arm]['q']) and np.isfinite(w['pre_step_qf'][arm]).all() for arm in w['pre_step_qf']),'loaded_qf_readback')
        require(digest(w['applied_controls'])==w['applied_controls_sha256']==digest(c) and w['applied_controls']==c,'loaded_pre_step_readback')
        require(equivalent(row['command'],{k:d[k] for k in ('q','v')}),'loaded_generated_drive')
        reference=d['reference_q']
    require(row['context']['formal_step']==start_row['context']['formal_step']+i+1,'loaded_tick_continuity')
    require(digest(c)==row['controls_sha256'] and fixed_controls(c,a.arm)==active['fixed'],'loaded_fixed_controls')
    for field,key in (('target','q'),('velocity','v')):
        require(np.allclose([j[field][0] for j in c[a.arm][:7]],row['command'][key],atol=2e-7,rtol=0),'loaded_actuator_command')
    measured=active['monitor'].measure(row['state'],row['native_ee'],reference)
    require(measured['violations']==row['metrics']['violations'] and np.allclose(measured['relative'],row['metrics']['relative'],atol=1e-11,rtol=0),'loaded_slip_replay')
    for key,value in measured['values'].items():
        require(abs(value-row['metrics']['values'][key])<=1e-10,'loaded_metric_replay:'+key)
        active['maxima'][key]=max(value,active['maxima'].get(key,0.))
    require(not active['violations'],'loaded_execution_after_bound_violation')
    active['violations'].extend(measured['violations']);active['index']+=1
    if tracking is not None:tracking.accept(d)
    active['previous_state']=row['state'];active['previous_controls']=c

def terminal(active,row):
    require(type(row['completed']) is bool and type(row['ticks']) is int and row['ticks']>=0,'loaded_terminal_types')
    require(row['target']==active['row']['target'] and row['arm']==active['row']['arm'] and row['phase']==active['row']['phase'],'loaded_terminal_identity')
    require(row['ticks']==active['index'],'loaded_terminal_tick_count')
    expected_step=active['row']['context']['formal_step']+active['index']
    require(row['context']['formal_step']>=expected_step,'loaded_terminal_step')
    if active['tracking'] is not None:
        t=row['tracking_terminal'];r=active['tracking']
        require(t['cursor']==r.at and t['index']==r.count and t['budget']==r.budget,'loaded_controller_terminal')
        if row['context']['formal_step']==expected_step:
            require(row['state']==active['previous_state'],'loaded_terminal_state_changed')
        rejected=t['rejected_command']
        if rejected is not None:
            require(not row['completed'] and rejected['index']==active['index'] and rejected['pre_state']==active['previous_state'] and rejected['pre_controls']==active['previous_controls'],'loaded_rejection_inputs')
            try:r.next(rejected['pre_state'])
            except ValueError as exc:
                reason=str(exc).removeprefix('tracking_audit_')
                require(reason in ('predictive_guard','stall_budget','state_budget') and rejected['reason'].startswith('loaded_tracking_'+('time_budget' if reason=='state_budget' else reason)),'loaded_rejection_reason')
            else:raise ValueError('loaded_unreproducible_rejection')
            if not isinstance(r,Replay005):require(equivalent(r.last_trials,rejected['trials']),'loaded_rejected_predicates')
        pending=t['pending_issue']
        if pending is not None:
            require(not row['completed'] and pending['pre_state_sha256']==digest(active['previous_state']) and pending['pre_controls_sha256']==digest(active['previous_controls']) and equivalent(pending['decision'],r.next(active['previous_state'])),'loaded_pending_command_binding')
            if 'applied_controls' in pending:require(digest(pending['applied_controls'])==pending['applied_controls_sha256'],'loaded_pending_readback_hash')
        if row['completed']:require(pending is None and rejected is None,'loaded_pending_command_on_success')
    if row['completed']:
        tracking=active['tracking']
        n=len(active['commands'])-1100 if tracking is None else tracking.arrival
        require(n is not None,'loaded_reference_not_finished')
        extra=active['index']-n-100
        require(row['failure'] is None and not active['violations'] and row['context']['formal_step']==expected_step,'loaded_completed_failure_or_gap')
        require(0<=extra<=1000 and extra%50==0,'loaded_hold_profile')
        native=np.asarray(row['native_ee']);b=active['row']['binding']
        require(np.linalg.norm(native[:3,3]-b['goal_position'])<=.005 and np.linalg.norm(so3_log(np.asarray(b['goal_rotation'])@native[:3,:3].T))<=.04,'loaded_endpoint_tracking')
        require(np.max(np.abs(np.asarray(row['state']['robots'][active['a'].arm]['q'][:7])-b['path'][-1]))<=.025,'loaded_joint_endpoint')
    return dict(target=row['target'],arm=row['arm'],phase=row['phase'],completed=row['completed'],failure=row['failure'],ticks=row['ticks'],
                terminal_step=row['context']['formal_step'],state_sha256=state_digest(row['state']),controls_sha256=state_digest(row['controls']),
                terminal_unobserved_ticks=row['context']['formal_step']-expected_step,violations=active['violations'],maxima=active['maxima'])

def inspect(run,manifest,cfg):
    run=Path(run);out=run/'artifacts';errors=[];motions=[];events=[];extracted=[];canonical=set();variant=manifest.get('loaded_variant');receipt={};count=0;raw_count=0;rolling=hashlib.sha256();active=None
    try:
        receipt=json.loads((out/'loaded-science-receipt.json').read_text());p=out/'loaded-science.jsonl.xz'
        binding=expected_binding(run,manifest);binding['scene_manifest_sha256']=file_hash(out/'scene-manifest.json')
        scene=json.loads((out/'scene-manifest.json').read_text());canonical=set(map(str,scene['selected_ids']))
        legacy=manifest.get('runtime_variant') in ('v36-live-combined-001','v36-live-combined-002')
        require(manifest.get('runtime_variant') in ('v36-live-combined-001','v36-live-combined-002','v36-live-combined-003','v36-live-combined-004','v36-live-combined-005','v36-live-combined-017','v36-live-combined-018','v36-live-combined-021'),'loaded_unsupported_runtime')
        profile=PROFILE if legacy or variant=='A_original' else (profile_for(variant) if manifest['runtime_variant'] in ('v36-live-combined-005','v36-live-combined-017','v36-live-combined-018','v36-live-combined-021') else SMOOTH_PROFILE)
        if variant=='B_clearance' and manifest['runtime_variant']=='v36-live-combined-005':profile=dict(profile,tracking=CONTRACT005)
        require(receipt['schema']==CONTRACT and receipt['variant']==variant and receipt['binding']==binding,'loaded_receipt_binding')
        require(receipt['formal_credit'] is False and receipt['goal_complete'] is False,'loaded_credit')
        require(p.stat().st_size==receipt['compressed_bytes']<=XZ_CAP and streamed_hash(p)==receipt['compressed_sha256'],'loaded_compressed_hash')
        src=run/'snapshot/src';model=ForwardModel(src/'fixtures/panda.urdf');g=json.loads((src/'fixtures/collision-boxes.json').read_text());rv=json.loads((src/'fixtures/robot-vertices.json').read_text());meshes=json.loads((out/'physics-meshes.json').read_text())
        for oid,parts in meshes.items():g['objects'].setdefault(oid,{})['parts']=bounds_for_parts(parts)
        def world(s):
            if legacy:
                # Frozen state() iterates env.objects, bound by selected_ids.
                require(set(map(str,scene['selected_ids']))==set(s['objects']) and len(scene['selected_ids'])==len(s['objects']),'loaded_source_object_order')
                s=dict(s,objects={str(k):s['objects'][str(k)] for k in scene['selected_ids']})
            return ReturnWorld(model,copy.deepcopy(g),s,meshes,rv)
        expected_events=[];saw_terminal=False
        with lzma.open(p,'rb') as stream:
            while True:
                raw=stream.readline(ROW_CAP+1)
                if not raw:break
                require(len(raw)<=ROW_CAP and raw.endswith(b'\n'),'loaded_raw_row_budget')
                raw_count+=len(raw);rolling.update(raw);row=json.loads(raw);require(row['sequence']==count,'loaded_row_sequence');count+=1
                require(raw_count<=RAW_CAP and count<=ROWS_CAP,'loaded_raw_budget')
                kind=row['kind'];ctx=row['context']
                require(not saw_terminal,'loaded_data_after_terminal')
                if count==1:require(kind=='episode_header' and row['binding']==binding and row['variant']==variant and row['full_episode'] is True and row['reconstructed'] is False,'loaded_header')
                if kind in ('live_admission','live_tick','live_motion_terminal'):
                    require(ctx['preview'] is False and str(ctx['held_id'])==str(row['target']) and ctx['drive_active'] is True,'loaded_formal_held_context')
                if kind=='live_admission':
                    require(active is None,'loaded_overlapping_motions');active=start(row,world,variant,profile,legacy)
                    expected_events.append(('loaded_live_admission',row['target'],row['arm'],row['phase'],ctx['formal_step'],row['binding_sha256']))
                elif kind=='live_tick':require(active is not None,'loaded_tick_without_admission');tick(active,row)
                elif kind=='live_motion_terminal':
                    if active is None:require(row['ticks']==0 and not row['completed'],'loaded_terminal_without_admission')
                    else:motions.append(terminal(active,row));active=None
                    expected_events.append(('loaded_live_terminal',row['target'],row['arm'],row['phase'],ctx['formal_step'],row['ticks'],row['completed'],row['failure']))
                elif kind=='episode_terminal':saw_terminal=True;require(row['failure']==receipt['failure'],'loaded_episode_failure')
        require(active is None and saw_terminal,'loaded_missing_terminal')
        require(count==receipt['rows'] and raw_count==receipt['raw_bytes'] and rolling.hexdigest()==receipt['raw_sha256'],'loaded_raw_hash')
        actual=[];extracted=[]
        with (out/'events.jsonl').open() as stream:
            for line in stream:
                e=json.loads(line)
                if e['kind']=='loaded_live_admission':actual.append((e['kind'],e['object_id'],e['arm'],e['loaded_phase'],e['physics_step'],e['binding_sha256']))
                elif e['kind']=='loaded_live_terminal':actual.append((e['kind'],e['object_id'],e['arm'],e['loaded_phase'],e['physics_step'],e['ticks'],e['completed'],e['failure']))
                elif e['kind']=='extracted':extracted.append(e['object_id'])
        require(actual==expected_events,'loaded_event_stream_binding')
        for oid in extracted:
            require([m['phase'] for m in motions if m['target']==oid and m['completed']]==['transport','lowering'],'loaded_extraction_coverage')
        result=json.loads((out/'result.json').read_text());require(result['loaded_variant']==variant and result['failure']==receipt['failure'],'loaded_result_binding')
    except Exception as exc:errors.append(type(exc).__name__+':'+str(exc))
    completed=[m for m in motions if m['completed']]
    expected_count=0 if cfg is None else cfg['count']
    full=not errors and len(canonical)==expected_count and set(map(str,extracted))==canonical and set(str(m['target']) for m in completed)==canonical and len(completed)==2*expected_count and expected_count>0 and all(m['terminal_unobserved_ticks']==0 and not m['violations'] for m in motions)
    return dict(valid=not errors,errors=errors,variant=variant,motions=motions,observed_loaded_ticks=sum(m['ticks'] for m in motions),loaded_physical_ticks=sum(m['ticks']+m['terminal_unobserved_ticks'] for m in motions),
                all_extraction_loaded_motions_witnessed=full,clearance_variant=variant=='B_clearance',
                empirical_envelope_only=True,in_flight_bound_proved=False,return_insertion_covered=False,
                formal_credit=False,goal_complete=False,receipt=receipt)
