"""Source-bound HOME intervention audit; never an identical-motion or formal gate.

Raw physical booleans remain separate from receipt validity. A failed preview
cannot become successful through geometry replay. No simulator is imported.
"""
import argparse
import copy
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from provenance import digest,file_hash,source_files,write_once
from home_trace_v18 import validate_trace
from n17_pair_v19 import observation_checks

IGNORED_PREFIX_FIELDS={'elapsed_s','motion_elapsed_s','time_unix'}


def normalize(value):
    if isinstance(value,dict):return {k:normalize(v) for k,v in value.items() if k not in IGNORED_PREFIX_FIELDS}
    if isinstance(value,list):return [normalize(v) for v in value]
    return value


def prefix(trace):
    out=[]
    for e in trace:
        if e.get('motion_phase')=='home':break
        out.append(e)
    return normalize(out)


def compare(a,b,off,on):
    checks={}
    modified=copy.deepcopy(on);flag=modified.pop('home_contact_envelope',False)
    checks['one_config_intervention']=flag is True and modified==off and not off.get('home_contact_envelope',False)
    checks['independent_unchanged_sources']=all(p.get('source_unchanged') is True and p.get('independent_scene') is True for p in (a,b))
    checks['prefix_before_home_exact']=bool(prefix(a['trace'])) and prefix(a['trace'])==prefix(b['trace'])
    checks['complete_motion_receipts']=all(validate_trace(p['trace'],c)['receipt_complete'] for p,c in ((a,off),(b,on)))
    checks['retreat_and_home_present']=all({e['motion_phase'] for e in p['trace'] if e['kind']=='return_motion_trace'}=={'retreat','home'} for p in (a,b))
    plans=[[e for e in p['trace'] if e['kind']=='return_motion_plan' and e['motion_phase']=='home'] for p in (a,b)]
    models=[e for e in b['trace'] if e['kind']=='home_contact_model']
    checks['one_live_model']=len(models)==1 and models[0]['model'].get('construction')=='live_public_shape_getters' and not models[0]['model'].get('synthetic_cpu_fixture')
    checks['raw_home_plans']=all(len(p)==1 for p in plans)
    changed=None
    if checks['one_live_model'] and checks['raw_home_plans']:
        model=models[0];p,q=plans[0][0],plans[1][0];h=model['model_sha256']
        checks['model_and_state_binding']=digest(model['model'])==h and digest(model['state'])==model['state_sha256']
        checks['entry_state_control_rng_equal']=all(p['planning_start'].get(k)==q['planning_start'].get(k)==model.get(k)
            for k in ('state_sha256','controls_sha256','rng_sha256'))
        checks['entry_step_equal']=p['initial_readback']['scene_step_count']==q['initial_readback']['scene_step_count']==model['scene_step']
        certs=[q['certificate']]+[e[k] for e in b['trace'] if e['kind']=='home_coarse_prepared' for k in ('original_certificate','fresh_certificate')]
        checks['all_home_certificates_bound']=len(certs)==3 and all(c.get('certified') is True and c.get('home_contact_model_sha256')==h for c in certs)
        checks['raw_curve_hashes']=all(digest(plan['vertices'])==plan['path_sha256'] and digest(plan['certificate'])==plan['certificate_sha256'] for plan in (p,q))
        changed=p['path_sha256']!=q['path_sha256']
    else:
        checks.update(model_and_state_binding=False,entry_state_control_rng_equal=False,entry_step_equal=False,
                      all_home_certificates_bound=False,raw_curve_hashes=False)
    if b.get('object_id')==10:
        observers=[e for e in b['trace'] if e['kind']=='n17_observation']
        checks['observer_complete']=len(observers)==1 and observers[0]['mode']=='observe' and all(observation_checks(observers[0],b).values())
        # A complete observer retains all six possible first-event types.
        # Any N17/robot pair contradicts this proposed interaction-exclusion
        # contract even if the physical speed/spin guards happen to pass.
        checks['n17_robot_pairs_absent']=checks['observer_complete'] and not any(
            w['trigger']=='first_robot_contact_record' for w in observers[0]['observation']['windows'])
    return checks,changed


def replay(run,preview):
    """Recheck the actual v20 HOME-entry model/path, without stepping a scene."""
    from home_interaction import ContactEnvelope
    from direct_core import ForwardModel
    from physics_geometry import bounds_for_parts
    from return_world import ReturnWorld
    from return_clearance import ReturnClearance
    from return_reconstruction import load_bundle
    e=next(e for e in preview['trace'] if e['kind']=='home_contact_model')
    plan=next(e for e in preview['trace'] if e['kind']=='return_motion_plan' and e['motion_phase']=='home')
    bundle=load_bundle(run/'snapshot/bundle',preview['object_id'])
    geometry=json.loads((run/'snapshot/src/fixtures/collision-boxes.json').read_text())
    vertices=json.loads((run/'snapshot/src/fixtures/robot-vertices.json').read_text())
    for oid,parts in bundle['meshes'].items():geometry['objects'].setdefault(oid,{})['parts']=bounds_for_parts(parts)
    model=ForwardModel(run/'snapshot/src/fixtures/panda.urdf')
    world=ReturnWorld(model,geometry,e['state'],bundle['meshes'],vertices,optimize=True)
    envelope=ContactEnvelope(e['model'],world)
    certifier=ReturnClearance(envelope.attach(world),e['arm'],e['object_id'])
    result=certifier.certify(plan['vertices'])
    return dict(certificate=result, matches_runtime=result==plan['certificate'], certified=result.get('certified') is True)


def audit(baseline,proposed):
    runs=[Path(baseline),Path(proposed)];checks={};raw=[];errors=[];inputs={};observed={};changed=None;rechecked=None
    read=lambda p:json.loads(p.read_text())
    try:
        manifests=[read(r/'diagnostic-manifest.json') for r in runs];configs=[read(r/'config.json') for r in runs]
        previews=[]
        for r in runs:
            events=[json.loads(line) for line in (r/'artifacts/events.jsonl').read_text().splitlines()]
            ps=[e for e in events if e['kind']=='return_preview']
            if len(ps)!=1:raise ValueError('single_fullpreview_required')
            previews.append(ps[0])
            raw.append(dict(run=str(r),terminal=read(r/'terminal.json'),result=read(r/'artifacts/diagnostic-result.json'),
                preview={k:ps[0].get(k) for k in ('object_id','index','accepted','failure','trial_steps','source_unchanged','independent_scene')},
                guard_outcome=[e for e in ps[0]['trace'] if e['kind'] in ('motion_phase_exit','return_motion_trace','home_contact_rejected','return_motion_rejected')],
                returned=[e for e in ps[0]['trace'] if e['kind']=='returned']))
            for n in ('diagnostic-manifest.json','config.json','artifacts/events.jsonl','artifacts/diagnostic-result.json','terminal.json','started.json','assignment.json','artifacts/reconstruction.json'):
                inputs[str(r/n)]=file_hash(r/n)
        expected=source_files(ROOT/'src')
        checks['current_source_bound']=all({n[4:]:h for n,h in m['snapshot_sha256'].items() if n.startswith('src/')}==expected for m in manifests)
        checks['immutable_snapshots_configs']=all(file_hash(r/'config.json')==m['config_sha256'] and all(file_hash(r/'snapshot'/n)==h for n,h in m['snapshot_sha256'].items()) for r,m in zip(runs,manifests))
        binding=read(ROOT/'harness/configs/v20/BINDINGS.json')
        checks['delivery_bound']=binding['source_sha256']==expected and all(m['config_sha256'] in binding['configs'].values() for m in manifests)
        for key in ('snapshot_sha256','release_sha256','source_fingerprint','target','candidate_index','max_previews','python','gpu','experience','n17_observer','legacy_insertion','fixed_setpoint_diagnostic','continuation_diagnostic'):
            checks['same_'+key]=key in manifests[0] and manifests[0][key]==manifests[1].get(key)
        checks['request']=all((m['target'],m['candidate_index']) in ((10,5),(12,6)) and m['max_previews']==1
            and not any(m.get(k) for k in ('legacy_insertion','fixed_setpoint_diagnostic','continuation_diagnostic')) for m in manifests)
        checks['config_flag_manifest_bound']=all(m.get('home_contact_envelope')==c.get('home_contact_envelope',False) for m,c in zip(manifests,configs))
        recons=[read(r/'artifacts/reconstruction.json') for r in runs]
        checks['same_validated_reconstruction']=recons[0]==recons[1] and all(e.get('validated') is True for e in recons)
        starts=[read(r/'started.json') for r in runs];assignments=[read(r/'assignment.json') for r in runs]
        checks['same_actual_gpu']=bool(assignments[0].get('gpu_uuid')) and assignments[0]['gpu_uuid']==assignments[1].get('gpu_uuid') and all(a['gpu_uuid']==s['gpu']['uuid'] for a,s in zip(assignments,starts))
        checks['terminal']=all(e['terminal'].get('state')=='terminal' and e['terminal'].get('exit_code')==0 for e in raw)
        checks['sequential']=any(starts[i]['time_unix']+raw[i]['terminal']['elapsed_s']<=starts[1-i]['time_unix'] for i in (0,1))
        compared,changed=compare(*previews,*configs);checks.update(compared)
        if checks['one_live_model'] and checks['raw_home_plans']:
            rechecked=replay(runs[1],previews[1]);checks['actual_model_path_replay']=rechecked['certified'] and rechecked['matches_runtime']
        else:checks['actual_model_path_replay']=False
        obs=[e for e in previews[1]['trace'] if e['kind']=='n17_observation']
        if obs:
            o=obs[0].get('observation') or {};rows=o.get('tail',[])+[r for w in o.get('windows',[]) for r in w['rows']]
            body=next((b for b in o.get('metadata',{}).get('bodies',[]) if b['label']==dict(name='216_inner0926_s1c0n17',arm=None)),{})
            initial_sleep=body.get('is_sleeping',{})
            complete=o.get('complete') is True and o.get('truncated_samples')==0
            observed=dict(stayed_asleep=complete and initial_sleep.get('available') is True and initial_sleep.get('value') is True and not any(w['trigger']=='sleep_transition' for w in o.get('windows',[])),
                any_robot_pair=any(w['trigger']=='first_robot_contact_record' for w in o.get('windows',[])) if complete else None,complete=o.get('complete'),samples=o.get('samples'),max_speed_m_s=o.get('max_speed_m_s'),max_spin_rad_s=o.get('max_spin_rad_s'),
                transitions=[dict(trigger=w['trigger'],scene_step=w['scene_step']) for w in o.get('windows',[])],
                retained_robot_pairs=[dict(scene_step=r['scene_step'],contacts=[c for c in r['contacts'] if any(b['arm'] is not None for b in c['bodies'])]) for r in rows if any(any(b['arm'] is not None for b in c['bodies']) for c in r['contacts'])],
                note='No first-robot-contact trigger with complete untruncated observation means no robot pair was observed throughout HOME; retained rows alone are not the whole timeline.')
    except Exception as exc:errors.append(type(exc).__name__+':'+str(exc))
    valid=bool(checks) and all(checks.values()) and not errors
    completed=len(raw)==2 and raw[1]['preview']['accepted'] is True and raw[1]['result'].get('preview_completed') is True
    if len(raw)==2:
        checks['formal_flags_locked']=all(all(e['result'].get(k) is False for k in ('accepted','whole_episode_success','full_ten_success','dataset_success')) for e in raw)
        if completed:
            exits=[e for e in raw[1]['guard_outcome'] if e['kind']=='motion_phase_exit']
            checks['completed_guard_exits']=len(exits)==2 and all(e.get('failed') is False and e.get('reason') is None for e in exits) and len(raw[1]['returned'])==1
        valid=valid and all(checks.values())
    return dict(schema='home-functional/20',checks=checks,errors=errors,inputs_sha256=inputs,raw=raw,
        intentional_home_curve_changed=changed,prefix_ignored_fields=sorted(IGNORED_PREFIX_FIELDS),geometry_replay=rechecked,
        n17_observation=observed,valid_functional_receipts=valid,proposed_fullpreview_completed=completed,
        functional_gate_passed=valid and completed,formal_credit=False,whole_episode_success=False,
        interpretation='Prefix equivalence plus independently certified intentional HOME intervention. Raw failed outcomes remain failures. No proved wake mechanism or formal return credit.')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('baseline',type=Path);parser.add_argument('proposed',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=audit(args.baseline,args.proposed);write_once(args.output,result)
    print(json.dumps({k:result[k] for k in ('checks','errors','intentional_home_curve_changed','valid_functional_receipts','proposed_fullpreview_completed','functional_gate_passed')}))
    raise SystemExit(0 if result['functional_gate_passed'] else 2)
