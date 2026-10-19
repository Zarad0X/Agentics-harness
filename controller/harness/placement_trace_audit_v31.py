"""Requested diagnostic witness integrity. Never a native physical-success proof."""
import ast,hashlib,json,math
from pathlib import Path
import numpy as np
from provenance import digest,file_hash
from direct_core import ForwardModel
from collision_world import pose_matrix
from placement_audit_v30 import expected_binding
from placement_trace import MAX_ROWS,MAX_BYTES,STAGES


# Audit-only representation envelope; see representation-derivation.md in v31
# evidence. This never replaces, projects, rounds or normalizes recorded data.
FLOAT32_U=2.**-24
FLOAT64_U=2.**-53
def _gamma(n,u):return n*u/(1-n*u)
# Four squares + three additions, sqrt and component division in float32.
# Upper bound on |sum(q_i**2)-1| after one finite normal-range normalization.
QUAT_NORM2_ERROR=(1+FLOAT32_U)**2/((1-_gamma(4,FLOAT32_U))*(1-FLOAT32_U)**2)-1
# Standard quaternion matrix formula: <=3 rounded product/sum operations,
# exact factor 2, plus final diagonal subtraction; |q|^2 <= 1+tau.
MATRIX_ENTRY_ERROR=_gamma(3,FLOAT32_U)*(1+2*(1+QUAT_NORM2_ERROR))
# R(q)=(1+delta)R(q/|q|)-delta I. Frobenius E <=3*entry error.
# Orthogonal float64 inverse/multiply changes neither singular values nor
# Gram error in exact arithmetic; explicit float64 allowance covers that work.
STORED_ROTATION_ERROR=2*QUAT_NORM2_ERROR+3*MATRIX_ENTRY_ERROR+_gamma(128,FLOAT64_U)
STORED_GRAM_ERROR=2*STORED_ROTATION_ERROR+STORED_ROTATION_ERROR**2
STORED_DETERMINANT_ERROR=(1+STORED_ROTATION_ERROR)**3-1
REPRESENTATION='normalized-f64-EE-inverse_times_native-f32-quaternion-matrix/1'


def rigid_transform(transform,representation):
    t=np.asarray(transform,dtype=float)
    if t.shape!=(4,4) or not np.isfinite(t).all():raise ValueError('placement_trace_transform')
    if not np.allclose(t[3],[0,0,0,1],rtol=0,atol=1e-12):raise ValueError('placement_trace_rigid_transform')
    r=t[:3,:3];gram=r.T@r;det=float(np.linalg.det(r))
    if representation=='normalized_pose_f64':
        if not np.allclose(gram,np.eye(3),rtol=0,atol=1e-9) or abs(det-1)>1e-9:raise ValueError('placement_trace_rigid_transform')
    elif representation==REPRESENTATION:
        # Separate orientation, scale/shear (singular values), Gram and volume
        # checks. A proper rotation in the wrong frame is NOT identified by
        # these numerical checks; immutable context/scene/source checks remain.
        if (det<=0 or abs(det-1)>STORED_DETERMINANT_ERROR
                or np.linalg.norm(gram-np.eye(3),ord=2)>STORED_GRAM_ERROR
                or np.max(np.abs(np.linalg.svd(r,compute_uv=False)-1))>STORED_ROTATION_ERROR):
            raise ValueError('placement_trace_rigid_transform')
    else:raise ValueError('placement_trace_unknown_representation')
    return dict(gram_max_abs_error=float(np.max(np.abs(gram-np.eye(3)))),determinant_error=abs(det-1))


def representation_binding(src,source):
    # The native-derived allowance is specific to the exact reviewed producer
    # and normalized-pose converter, not a general transform tolerance.
    src=Path(src)
    for name,expected in REPRESENTATION_SOURCE.items():
        if source.get(name)!=expected or file_hash(src/name)!=expected:raise ValueError('placement_trace_representation_source')
    p=src/'physics_runner.py'
    if source.get('physics_runner.py')!=file_hash(p):raise ValueError('placement_trace_representation_source')
    funcs=[n for n in ast.walk(ast.parse(p.read_text())) if isinstance(n,ast.FunctionDef) and n.name=='execute_grasp']
    if len(funcs)!=1 or hashlib.sha256(ast.dump(funcs[0]).encode()).hexdigest()!=GRASP_AST_SHA256:raise ValueError('placement_trace_representation_producer')

REPRESENTATION_SOURCE={'placement_trace.py': '088853638eaca67d610f0fe90a02f6b05d53460d7f980c3d865931af47124803', 'collision_world.py': '43ab2bdc97f17a3596472a5a3adf7bf62aa7704ef611ef3df71c7947ce3a51a0'}
GRASP_AST_SHA256='2de05ccdc29b308ebb6c9fa0d1093830149e0d1e5f6255bb98806d4537ea2ce0'


def verify_trace(path,receipt,binding,target,model,events,canonical):
    if (receipt.get('schema')!='placement-transition-receipt/30' or receipt.get('binding')!=binding
            or receipt.get('selected_object_id')!=target or receipt.get('formal_credit') is not False
            or receipt.get('physical_completion_proved') is not False or receipt.get('recording_complete') is not True
            or receipt.get('error') is not None or receipt.get('requested_object_reached') is not True):
        raise ValueError('placement_trace_receipt_binding_or_incomplete')
    if path.stat().st_size>MAX_BYTES+16384:raise ValueError('placement_trace_byte_cap')
    h=hashlib.sha256();previous=None;rows=0;size=0;header=None;footer=None;context=None;last_step=None
    stage_rows={};ticks={};exception=False;last_stage=-1;tracking={};max_native_fk_position=0.
    relevant=[e for e in events if e.get('object_id',e.get('target'))==target]
    for e in relevant:
        if e['kind']=='physical_tracking':tracking.setdefault(e['physics_step'],[]).append(e)
    with path.open('rb') as stream:
        for line in stream:
            h.update(line);size+=len(line);row=json.loads(line)
            if type(row['sequence']) is not int or row['sequence']!=rows or row['previous_sha256']!=previous or row['row_sha256']!=digest({k:v for k,v in row.items() if k!='row_sha256'}):
                raise ValueError('placement_trace_row_chain')
            previous=row['row_sha256'];rows+=1
            if rows>MAX_ROWS+2:raise ValueError('placement_trace_row_cap')
            kind=row['kind']
            if footer is not None:raise ValueError('placement_trace_after_footer')
            if rows==1:
                header={k:v for k,v in row.items() if k not in ('sequence','previous_sha256','row_sha256')}
                if (kind!='header' or header['schema']!='placement-transition/30' or header['binding']!=binding
                        or header['selected_object_id']!=target or header['max_measurement_rows']!=MAX_ROWS
                        or header['max_measurement_bytes']!=MAX_BYTES or digest(header)!=receipt['header_sha256']):
                    raise ValueError('placement_trace_header_binding')
                continue
            if kind=='footer':footer=row;continue
            if kind not in ('stage','tick','exception','finalization'):raise ValueError('placement_trace_row_kind')
            o=row['observation'];stage=row['stage']
            if (o['scene_role']!='formal' or o['scene_token']!=header['formal_scene_token']
                    or o['environment_token']!=header['formal_environment_token'] or o['object_id']!=target
                    or o['arm'] not in ('left','right') or o['episode_phase']!='extraction'
                    or type(o['physics_step']) is not int or o['physics_step']<0 or stage not in STAGES):
                raise ValueError('placement_trace_scene_phase_identity')
            if last_step is not None and (o['physics_step']<last_step or (kind=='tick' and o['physics_step']!=last_step+1)):
                raise ValueError('placement_trace_tick_coverage')
            if kind!='tick' and last_step is not None and o['physics_step']!=last_step:
                raise ValueError('placement_trace_unrecorded_ticks')
            last_step=o['physics_step']
            if kind=='stage':
                order=STAGES.index(stage)
                if order!=last_stage+1:raise ValueError('placement_trace_stage_order')
                last_stage=order;stage_rows[stage]=o
                if stage=='lowering':
                    context=row['context']
                    candidates=[e for e in relevant if e['kind']=='transported' and e.get('endpoint_admission_sha256')==context['endpoint_admission_sha256']]
                    if len(candidates)!=1 or candidates[0]['physics_step']!=o['physics_step'] or candidates[0]['arm']!=o['arm']:
                        raise ValueError('placement_trace_transport_context')
                    pre=[e for e in relevant if e['kind']=='placement_preflight' and e.get('endpoint_admission_sha256')==context['endpoint_admission_sha256']]
                    if len(pre)!=1 or any(context[k]!=pre[0][k] for k in ('slot','low','rotation','low_q','retreat_q')):
                        raise ValueError('placement_trace_preflight_context')
                    reservations=[e for e in events if e['kind']=='home_workspace_model']
                    if len(reservations)!=1 or reservations[0]['model_sha256']!=context['reservation_model_sha256'] or reservations[0]['homes']!=context['homes']:
                        raise ValueError('placement_trace_HOME_context')
                    admitted=[e for e in relevant if e['kind']=='home_command_admission' and e.get('stage')=='predicted_placement_route' and e['physics_step']<=o['physics_step']]
                    if not admitted or any(admitted[-1].get(k)!=v for k,v in context['HOME_command_admission'].items()):
                        raise ValueError('placement_trace_HOME_command_context')
                    reserved=[e for e in relevant if e['kind']=='home_workspace_reservation' and e.get('stage')=='predicted_slot' and e['physics_step']<=o['physics_step']]
                    if not reserved or reserved[-1]['predicted_transform']!=context['predicted_transform'] or reserved[-1]['safe'] is not True:
                        raise ValueError('placement_trace_reservation_context')
            if context is None or row['context_sha256']!=digest(context):raise ValueError('placement_trace_context_hash')
            if set(o['robots'])!={'left','right'} or set(o['objects'])!=canonical:raise ValueError('placement_trace_state_coverage')
            for robot in o['robots'].values():
                for key in ('q','qdot'):
                    a=np.asarray(robot[key],float)
                    if a.shape!=(9,) or not np.isfinite(a).all():raise ValueError('placement_trace_joint_observation')
                if robot['fingers']!=robot['q'][-2:] or robot['finger_velocities']!=robot['qdot'][-2:]:raise ValueError('placement_trace_finger_binding')
                controls=robot['controls']
                if len(controls)!=9 or len({c['name'] for c in controls})!=9:raise ValueError('placement_trace_drive_coverage')
                for c in controls:
                    for key in ('target','velocity_target'):
                        if np.asarray(c[key]).shape!=(1,) or not np.isfinite(c[key]).all():raise ValueError('placement_trace_drive_observation')
                    if any(type(c[k]) not in (int,float) or not math.isfinite(c[k]) for k in ('stiffness','damping','force_limit')):raise ValueError('placement_trace_drive_properties')
                    if not isinstance(c['mode'],str):raise ValueError('placement_trace_drive_mode')
            for obj in o['objects'].values():
                pose_matrix(obj['pose'])
                for key in ('linear_velocity','angular_velocity'):
                    if np.asarray(obj[key]).shape!=(3,) or not np.isfinite(obj[key]).all():raise ValueError('placement_trace_object_velocity')
            for contact in o['contacts']:
                if len(contact['bodies'])!=2:raise ValueError('placement_trace_contact_identity')
                for body in contact['bodies']:
                    if not all(isinstance(body.get(k),str) for k in ('kind','python_body_token','actor_name','native_type')):raise ValueError('placement_trace_contact_identity')
                    if body['kind']=='robot_link' and body.get('arm') not in ('left','right'):raise ValueError('placement_trace_contact_arm')
                for point in contact['points']:
                    if not math.isfinite(point['separation']):raise ValueError('placement_trace_contact_separation')
                    for key in ('impulse','normal','position'):
                        if np.asarray(point[key]).shape!=(3,) or not np.isfinite(point[key]).all():raise ValueError('placement_trace_contact_point')
            q=o['robots'][o['arm']]['q'];fk=model.evaluate(o['arm'],q[:7])
            if o['fk_ee']!=dict(position=fk['ee_position_m'],rotation=fk['ee_rotation_world']):raise ValueError('placement_trace_FK_replay')
            native=np.asarray(o['native_ee'],float);stored=np.asarray(o['stored_held_transform'],float)
            if native.shape!=(4,4) or stored.shape!=(4,4) or not np.isfinite(native).all() or not np.isfinite(stored).all():raise ValueError('placement_trace_transform')
            if o['stored_held_transform']!=context['stored_held_transform']:raise ValueError('placement_trace_stored_transform_binding')
            rigid_transform(native,'normalized_pose_f64')
            rigid_transform(stored,REPRESENTATION)
            measured=np.linalg.inv(native)@pose_matrix(o['objects'][str(target)]['pose'])
            if not np.array_equal(measured,np.asarray(o['current_measured_transform'])):raise ValueError('placement_trace_measured_transform')
            max_native_fk_position=max(max_native_fk_position,float(np.linalg.norm(native[:3,3]-fk['ee_position_m'])))
            if type(o['drive_active']) is not bool:raise ValueError('placement_trace_drive_active')
            if STAGES.index(stage)<=STAGES.index('detach_before'):
                if o['held_id']!=target or not o['drive_active']:raise ValueError('placement_trace_held_before_release')
            elif o['held_id'] is not None or o['drive_active']:raise ValueError('placement_trace_held_after_release')
            if kind=='tick':
                ticks[stage]=ticks.get(stage,0)+1
                if row['tick_held_argument'] is not (STAGES.index(stage)<=STAGES.index('detach_before')):raise ValueError('placement_trace_tick_held')
            if kind=='exception':exception=True
            for e in tracking.get(o['physics_step'],[]):
                if e['target_position']==context['low'] and (e['q']!=q or e['actual_ee']!=o['native_ee']):
                    raise ValueError('placement_trace_existing_tracking_binding')
    if footer is None or h.hexdigest()!=receipt['sha256'] or rows!=receipt['rows'] or size!=receipt['bytes'] or previous!=receipt['last_row_sha256']:
        raise ValueError('placement_trace_final_integrity')
    if footer['error'] is not None or footer['requested_object_reached'] is not True or footer['transition_completed'] is not receipt['transition_completed']:
        raise ValueError('placement_trace_final_state')
    if receipt['transition_completed']:
        if tuple(stage_rows)!=STAGES or ticks.get('lowering_hold')!=100 or ticks.get('postrelease_wait')!=250:
            raise ValueError('placement_trace_required_hold_wait_coverage')
    elif not exception and footer['episode_failure'] is None:raise ValueError('placement_trace_missing_failure_finalization')
    return dict(stored_transform_representation=REPRESENTATION,stored_rotation_error_bound=STORED_ROTATION_ERROR,valid=True,requested=True,transition_completed=receipt['transition_completed'],tick_counts=ticks,rows=rows,
        max_native_FK_position_error_m=max_native_fk_position,exception_recorded=exception,physical_completion_proved=False)


def inspect(run,manifest,cfg):
    run=Path(run);a=run/'artifacts';target=cfg.get('placement_trace_object_id')
    paths=[a/'placement-transition.jsonl',a/'placement-transition-receipt.json']
    result=dict(requested=target is not None,valid=False,errors=[],physical_completion_proved=False)
    try:
        if target is None:
            if any(p.exists() for p in paths):raise ValueError('unexpected_placement_trace')
            return dict(result,valid=True)
        if (a/'placement-transition-foreign-scene.json').exists():raise ValueError('placement_trace_foreign_scene')
        representation_binding(run/'snapshot/src',manifest['source_sha256'])
        receipt=json.loads(paths[1].read_text());scene=json.loads((a/'scene-manifest.json').read_text())
        binding=dict(expected_binding(run,manifest),scene_manifest_sha256=file_hash(a/'scene-manifest.json'))
        events=[json.loads(line) for line in (a/'events.jsonl').read_text().splitlines()]
        model=ForwardModel(run/'snapshot/src/fixtures/panda.urdf')
        result.update(verify_trace(paths[0],receipt,binding,target,model,events,set(scene['objects'])))
        result['inputs_sha256']={str(p):file_hash(p) for p in paths}
    except Exception as exc:result['errors'].append(type(exc).__name__+':'+str(exc))
    return result
