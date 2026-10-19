"""Two immutable development branch hints, never joint commands or acceptance.

Lookup examines <=2 records; at most one compatible seed gets one custom IK
call (120 updates, zero restarts). No object identity is used for selection.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
import astra_planner as planner
from provenance import file_hash,digest

CATALOG_SHA256='809fbf83e4a52d6240e6e42f64f32b373d13368c6c5b53778e867a0c2dffb8a9'
MAX_RECORDS=2
MAX_POSITION_DELTA_M=.002
MAX_ROTATION_DELTA_RAD=.01


def residual(model,arm,q,p,r):
    fk=model.evaluate(arm,list(q))
    return dict(position_error_m=float(np.linalg.norm(np.asarray(fk['ee_position_m'])-p)),
        rotation_error_rad=float(np.linalg.norm(planner.so3_log(np.asarray(r)@np.asarray(fk['ee_rotation_world']).T))))


def passes(errors):
    return (errors['position_error_m']<=planner.POSITION_TOLERANCE_M
            and errors['rotation_error_rad']<=planner.ROTATION_TOLERANCE_RAD)


def load_catalog(model,root=None):
    """Pinned artifact plus original algorithm/model and seed FK revalidation.

    Embedded raw events came from the immutable parent subset; the offline
    exporter verifies its receipt and source hashes. No external files are
    consulted at runtime, and no runtime observation can extend this catalog.
    """
    root=Path(root) if root is not None else Path(__file__).parent
    path=root/'fixtures/high-transfer-branch-seeds-v10.json'
    if file_hash(path)!=CATALOG_SHA256:raise ValueError('branch_catalog_integrity')
    data=json.loads(path.read_text())
    if data['split']!='development' or data['online_learning'] is not False or not 0<len(data['records'])<=MAX_RECORDS:
        raise ValueError('branch_catalog_scope')
    if any(file_hash(root/n)!=h for n,h in data['model_source_sha256'].items()):raise ValueError('branch_model_source')
    if model.bounds!=data['model_bounds']:raise ValueError('branch_model_bounds')
    for record in data['records']:
        if record['record_id']!=digest({k:v for k,v in record.items() if k!='record_id'}):raise ValueError('branch_record_integrity')
        events=[]
        for link in record['source_events']:
            if hashlib.sha256(link['raw_line'].encode()).hexdigest()!=link['raw_sha256']:raise ValueError('branch_source_event')
            events.append(json.loads(link['raw_line']))
        payload,plan=events
        if (payload['kind']!='return_payload_transform' or plan['kind']!='custom_plan'
            or payload['object_id']!=plan['target'] or payload['physics_step']!=plan['physics_step']
            or plan['ik']['success'] is not True or plan['path']['success'] is not True
            or record['seed_q']!=plan['ik']['q'] or plan['path']['path'][-1]!=record['seed_q']
            or record['position']!=payload['above'] or not np.allclose(record['rotation'],np.asarray(payload['ee_goal'])[:3,:3],atol=1e-14,rtol=0)):
            raise ValueError('branch_source_success')
        model.check_q(record['seed_q'])
        if not passes(residual(model,record['arm'],record['seed_q'],record['position'],record['rotation'])):
            raise ValueError('branch_source_fk')
    return data


def fallback_ik(failed,model,arm,p,r,rng,event):
    """Return a freshly solved IK result, or the original finite failure.

    Caller alone plans/executes with its measured loaded world and guards.
    Uses the caller RNG with zero restarts (no draws, no RNG reset).
    """
    if failed['success']:return failed
    report=dict(schema='branch-seed-attempt/10',arm=arm,position=np.asarray(p).tolist(),
        rotation=np.asarray(r).tolist(),ordinary_ik=failed,catalog_sha256=CATALOG_SHA256,
        records_examined=0,ik_calls=0,max_updates=120,restarts=0,acceptance=False)
    try:
        data=load_catalog(model)
        compatible=[]
        for record in data['records']:
            report['records_examined']+=1
            if record['arm']!=arm:continue
            dp=float(np.linalg.norm(np.asarray(p)-record['position']))
            dr=float(np.linalg.norm(planner.so3_log(np.asarray(r)@np.asarray(record['rotation']).T)))
            if dp<=MAX_POSITION_DELTA_M and dr<=MAX_ROTATION_DELTA_RAD:
                compatible.append((dp/MAX_POSITION_DELTA_M+dr/MAX_ROTATION_DELTA_RAD,record['record_id'],record))
        if not compatible:
            report['reason']='no_compatible_seed';event('branch_seed_attempt',**report);return failed
        record=min(compatible,key=lambda row:row[:2])[2]
        # Validate again at use; original seed is input only, never returned as a command.
        model.check_q(record['seed_q'])
        report.update(record_id=record['record_id'],seed_q=list(record['seed_q']),
            source_original_events_sha256=data['source_original_events_sha256'],
            source_lines=[x['original_line_1based'] for x in record['source_events']],
            raw_seed_residual=residual(model,arm,record['seed_q'],p,r),ik_calls=1)
        solved=planner.solve_ik(model,arm,p,r,record['seed_q'],rng=rng,max_iters=120,restarts=0)
        report['result']=solved
        model.check_q(solved['q'])
        report['resolved_fk']=residual(model,arm,solved['q'],p,r)
        valid=solved['success'] and passes(report['resolved_fk'])
        report['reason']='resolved_requires_guarded_path' if valid else 'seed_ik_failed'
        event('branch_seed_attempt',**report)
        return solved if valid else failed
    except (ValueError,KeyError,TypeError,IndexError,OSError) as exc:
        report.update(reason='invalid_seed_source_or_model',error=str(exc))
        event('branch_seed_attempt',**report)
        return failed
