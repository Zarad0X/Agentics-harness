"""Independent resource/capture provenance checks; never physical success credit."""
import argparse,json
from pathlib import Path
from provenance import file_hash,digest,write_once
from headless_resource_v36 import validate_assignment,child_binding
from render_input_v36 import inspect

def receipts(run,m,assignment,started,result=None,admission=False):
    a=run/'artifacts';read=lambda n:json.loads((a/n).read_text());e=m['execution']
    if e['mode']!='headless' or e['admission_only'] is not admission:raise ValueError('headless_mode_or_admission')
    validate_assignment(assignment,run,m,now=started['time_unix'],host=started['host'])
    if started['gpu']!=dict(headless=True,gpu_queried=False,uuid='',index=None,pci_bus_id=None,cpu_core=assignment['cpu_core']):raise ValueError('headless_started_resource')
    command=started['command']
    if command.count('--execution-manifest')!=1 or Path(command[command.index('--execution-manifest')+1])!=run/'diagnostic-manifest.json':raise ValueError('headless_child_command')
    if Path(command[1])!=run/'snapshot/src/continuous_diagnostic_v36.py':raise ValueError('headless_child_source')
    scene=read('scene-manifest.json');defs=read('visual-definitions.json');capture=read('numeric-capture-receipt.json');init=read('headless-initialization.json')
    binding=child_binding(run,m);contract=capture['contract'];expected=scene['headless_candidate']
    if expected.get('visual_definitions_sha256')!=file_hash(a/'visual-definitions.json') or expected.get('headless_initialization_sha256')!=file_hash(a/'headless-initialization.json'):raise ValueError('headless_definition_hash')
    if set(defs['identity'])!=set(scene['objects']):raise ValueError('headless_object_identity')
    for oid,identity in defs['identity'].items():
        if any(scene['objects'][oid].get(k)!=v for k,v in identity.items()):raise ValueError('headless_geometry_identity')
    if (defs['bindings']!=binding or any(expected.get(k)!=v for k,v in binding.items())
        or expected.get('source_manifest_is_executed_source') is not True or expected.get('executed_runner_is_detached_candidate') is not False
        or scene['source_sha256']!=m['source_sha256'] or expected['capture']!=contract or defs['contract']!=contract):raise ValueError('headless_executed_source_binding')
    if (contract['capture_every_steps']!=e['capture_every_steps'] or contract['maximum_frames']!=e['max_frames']
        or contract['rgbd_generated'] is not False or contract['video_generated'] is not False or contract['offline_render_required'] is not True):raise ValueError('headless_capture_allocation')
    numeric=inspect(a)
    if numeric['frames']<1 or numeric['frames']>e['max_frames'] or capture['exception'] is not None:raise ValueError('headless_capture_incomplete')
    stats=capture['scene_stats'];roles=['formal','grasp_preview','separation_preview']
    if (stats['scene_roles']!=roles or stats['scenes']!=3 or stats['forbidden_calls']!=[]
        or init['scene_roles']!=roles or init['independent_systems'] is not True
        or set(init['steps_by_role'])!=set(roles) or set(stats['steps_by_role'])!=set(roles)):raise ValueError('headless_scene_roles')
    for values in (init['steps_by_role'],stats['steps_by_role']):
        if any(type(v) is not int or v<0 for v in values.values()):raise ValueError('headless_step_type')
    if (sum(init['steps_by_role'].values())!=init['total'] or sum(stats['steps_by_role'].values())!=stats['physics_steps']
        or stats['physics_steps']>e['max_physics_steps'] or any(stats['steps_by_role'][r]<init['steps_by_role'][r] for r in roles)):raise ValueError('headless_step_totals')
    final_step=0 if admission else result['physics_steps']
    regular=-e['capture_every_steps'];last=None
    with (a/'render-state.jsonl').open() as stream:
        for line in stream:
            frame=json.loads(line);step=frame['physics_step']
            if type(step) is not int or not 0<=step<=final_step or frame['simulation_time_s']!=step*.004:raise ValueError('headless_frame_step')
            if step%e['capture_every_steps']==0 and step!=regular:
                if step!=regular+e['capture_every_steps']:raise ValueError('headless_capture_gap')
                regular=step
            if last is None:
                if step!=0:raise ValueError('headless_initial_frame')
                for oid,obj in frame['objects'].items():
                    expected_object=scene['objects'][oid]
                    if obj['pose']['p']+obj['pose']['q']!=expected_object['pose'] or any(obj[k]!=expected_object[k] for k in ('linear_velocity','angular_velocity','actor_name')):raise ValueError('headless_initial_object')
                for arm in ('left','right'):
                    if any(frame['robots'][arm][k]!=scene['robots'][arm][k] for k in ('q','qdot')):raise ValueError('headless_initial_robot')
            last=step
    if last!=final_step or regular!=(final_step//e['capture_every_steps'])*e['capture_every_steps']:raise ValueError('headless_terminal_or_cadence')
    if admission:
        probe=read('headless-admission.json')
        if (probe['copy_checks']!=roles[1:] or probe['accepted'] is not True or probe['no_formal_grasp'] is not True
            or probe['formal_credit'] is not False or probe['goal_complete'] is not False or probe['physics_steps']!=stats
            or probe['initialization_steps']!=init['steps_by_role'] or stats['physics_steps']!=init['total']):raise ValueError('headless_admission_contract')
    else:
        if result is None or (a/'headless-admission.json').exists():raise ValueError('admission_not_full_run')
        r=result['headless_candidate']
        if any(r.get(k)!=v for k,v in binding.items()) or r['scene_steps']!=stats or r['initialization_steps']!=init['steps_by_role'] or r['capture']!=contract:raise ValueError('headless_result_binding')
        if (result['initialization_physics_steps']!=init['total'] or r['formal_steps']!=result['physics_steps'] or r['preview_steps']!=result['physics_preview_steps']
            or stats['steps_by_role']['formal']-init['steps_by_role']['formal']!=result['physics_steps']
            or stats['physics_steps']!=init['total']+result['physics_steps']+result['physics_preview_steps']
            or r['nested_previews_included_once'] is not True):raise ValueError('headless_step_accounting')
        if result['gpu_at_start']!=dict(uuid='',index=None,pci_bus_id=None,headless=True,gpu_queried=False):raise ValueError('headless_result_resource')
    return True

def audit_admission(run):
    from continuous_v36 import validate_manifest
    from continuous_audit_v36 import classify_terminal
    run=Path(run).resolve();errors=[]
    try:
        m=json.loads((run/'diagnostic-manifest.json').read_text());validate_manifest(run,m)
        terminal=json.loads((run/'terminal.json').read_text());classify_terminal(run,m,terminal)
        if terminal['reason']!='exited' or terminal['exit_code']!=0:raise ValueError('admission_terminal_failure')
        receipts(run,m,json.loads((run/'assignment.json').read_text()),json.loads((run/'started.json').read_text()),admission=True)
    except Exception as exc:errors.append(type(exc).__name__+':'+str(exc))
    return dict(schema='headless-admission-audit/36',errors=errors,admission_qualified=not errors,native_grasp_qualified=False,formal_credit=False,full_goal_achieved=False)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args();r=audit_admission(a.run);write_once(a.output,r);print(json.dumps(r));raise SystemExit(2 if r['errors'] else 0)
