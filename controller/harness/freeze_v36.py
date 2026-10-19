"""Write-once v36 integrated runtime bindings, CPU qualification and delivery."""
import argparse,ast,copy,json,shutil,stat,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'harness')]
from provenance import canonical,digest,file_hash,source_files,write_once


def config_values(root=ROOT):
    old=json.loads((root/'harness/configs/v35/BINDINGS.json').read_text())
    return {Path(n).name:(root/n).read_bytes() for n in old['configs']}



def runtime_python():
    # Conservative local import closure of all bound source modules and these
    # actual CLI/auditor/sealer entrypoints. No old evidence/diagnostics/assets.
    roots=[ROOT/'harness'/n for n in ('continuous_v36.py','return_diagnostic_v36.py','diagnostic_dispatch_v36.py',
        'continuous_audit_v36.py','home_cpu_audit_v36.py','freeze_v36.py','validate_config_v36.py','headless_audit_v36.py','render_input_v36.py')]
    paths=set((ROOT/'src').glob('*.py'))|set(roots);pending=list(paths)
    while pending:
        path=pending.pop();tree=ast.parse(path.read_text());names=set()
        for n in ast.walk(tree):
            if isinstance(n,ast.Import):names.update(v.name.split('.')[0] for v in n.names)
            if isinstance(n,ast.ImportFrom) and n.module:names.add(n.module.split('.')[0])
            if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='__import__' and n.args and isinstance(n.args[0],ast.Constant) and isinstance(n.args[0].value,str):names.add(n.args[0].value.split('.')[0])
        for name in names:
            local=next((ROOT/folder/(name+'.py') for folder in ('src','harness') if (ROOT/folder/(name+'.py')).is_file()),None)
            if local is not None and local not in paths:paths.add(local);pending.append(local)
    return paths


def bindings():
    out=ROOT/'harness/configs/v36';out.mkdir()
    for name,data in config_values().items():
        with (out/name).open('xb') as f:f.write(data)
        (out/name).chmod(0o444)
    source=source_files(ROOT/'src');old=json.loads((ROOT/'harness/configs/v35/BINDINGS.json').read_text())
    policy=json.loads((ROOT/old['policy']).read_text());policy['implementation_sha256']=digest(source)
    policy['policy_sha256']=digest({k:v for k,v in policy.items() if k!='policy_sha256'})
    write_once(out/'experience-final-001.json',policy)
    configs={str(p.relative_to(ROOT)):file_hash(p) for p in sorted(out.glob('*.json')) if p.name!='experience-final-001.json'}
    continuous={n:h for n,h in configs.items() if Path(n).name.startswith('continuous-')}
    deps=runtime_python()|{ROOT/'src'/n for n in source}|set(out.glob('*.json'))
    deps.update(ROOT/f'harness/configs/v{v}/BINDINGS.json' for v in (20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35))
    dependencies={str(p.relative_to(ROOT)):file_hash(p) for p in sorted(deps)}
    result=dict(schema='continuous-bindings/36',source_sha256=source,implementation_sha256=digest(source),
        configs=configs,continuous_configs=continuous,runtime_dependencies=dependencies,
        historical_binding_dependencies={str(ROOT.joinpath(f'harness/configs/v{v}/BINDINGS.json').relative_to(ROOT)):file_hash(ROOT/f'harness/configs/v{v}/BINDINGS.json') for v in (20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35)},
        policy='harness/configs/v36/experience-final-001.json',policy_file_sha256=file_hash(out/'experience-final-001.json'),
        original_v24_config_sha256=file_hash(ROOT/'harness/configs/v24/continuous-original10-headroom06-k6-reserved-entry.json'),
        profile_contract=dict(default=False,max_calls=1,first_actual_HOME_only=True,preview=False,physical_options_changed_from_v26=False,closure_hold_ticks_old=500,closure_hold_ticks_new=1000,closure_settled_dwell_ticks=25),
        integration_contract=dict(headless_opt_in=True,executed_source_sealed=True,offline_RGBD_pending=True,preview_accounting_schema='preview-lifecycle/35',sleep_copy_observer_default=None,sleep_copy_observer_target=17,sleep_copy_observer_record_cap=2000,sleep_copy_observer_byte_cap=67108864,native_qualified=False,sleep_copy_helper_sha256='7c4ec2dcefde8bf7f2f596d9aca736c690a1fd07c1caa4386417374cad68e3d9',sleep_copy_observable_only=True,solver_history_cloned=False,HOME_entry_helper_sha256='6ba913c0308321033f978da402de3a5a3864de2e3a2418d6e7376e1a347b421c',HOME_entry_necessary_nominal_start_only=True,inward_tilt_helper_sha256='0f7a4356cdee334a7ed8a42212b0d76943c1924616f8e55f504ded13f8c5df18',inward_tilt_degrees=20,inward_tilt_depth_m=.014,inward_tilt_suffix_cap=8,inward_tilt_extraction_only=True,placement_endpoint_helper_sha256='46c29be0b13f3e12cb51fec693cc7e261ead81dd646c9b8353afb7224503c64e',scale_validation_module_sha256='175d7f36c1396adabc0a77e71bff9cdc6e0b87f60d754fd8ec017b5086c93fbc',placement_trace_default=None,placement_trace_optional_object=10,extraction_halfturn_prefix=True,candidate_replay_logging=True,candidate_visit_cap=24,forecast_visit_cap=6,return_velocity_admission=True,return_speed_m_s=.03,return_spin_rad_s=.15,return_settle_ticks=500,actual_state_retries_added=0,closure_candidate='n9-closure-repair-002',closure_hold_ticks_old=500,closure_hold_ticks_new=1000,closure_settled_dwell_ticks=25,closure_native_dt_absolute_tolerance=1e-9,closure_velocity_m_s=.01,HOME_dense_command_cap=30000,HOME_total_tick_cap=36000,HOME_route_attempt_cap=6,offset_exact_float32=True,physical_convergence_proved=False),
        resource_contract=dict(timeout_caps_s={'10':86400,'15':129600,'20':172800},minimum_s=60,strict_integer=True,resource_allocation_only=True,existing_runs_unchanged=True),
        heldout_enabled=False,production_gate_unlocked=False,formal_credit=False)
    write_once(out/'BINDINGS.json',result);write_once(ROOT/'harness/evidence/v36/source-sha256.json',source)
    return dict(implementation_sha256=digest(source),runtime_dependencies=len(dependencies),configs=len(configs))


def preservation():
    ledger=json.loads((ROOT/'harness/evidence/v36/protected-inputs-001.json').read_text())
    assert all(file_hash(ROOT/n)==v['sha256'] and (ROOT/n).stat().st_size==v['bytes'] and stat.S_IMODE((ROOT/n).stat().st_mode)==v['mode'] for n,v in ledger.items())
    frozen=ROOT/'harness/revisions/v35-final-001'
    before=source_files(frozen/'src');after=source_files(ROOT/'src')
    changed=[n for n in before if before[n]!=after[n]]
    assert changed==['physics_runner.py'],changed
    from headless_delta_v36 import remove_headless
    defs=lambda text:{n.name:ast.dump(n) for n in ast.walk(ast.parse(text)) if isinstance(n,ast.FunctionDef)}
    oldtext=(frozen/'src/physics_runner.py').read_text();newtext=(ROOT/'src/physics_runner.py').read_text()
    a,b=defs(oldtext),defs(newtext);normalized=defs(remove_headless(newtext))
    assert sorted(n for n in a if a[n]!=b[n])==sorted(['main','setup_scene','record_trace','tick'])
    assert all(a[n]==normalized[n] for n in a if n!='main')
    # Main's remaining changes are exactly version dispatch/provenance, reviewed in diff.
    for name in ('sleep_state_copy.py','preview_accounting_v35.py','sleep_copy_observer_v35.py','sleep_copy_observation_v35.py','placement_endpoint.py','home_interaction.py','wall_grasps.py','home_entry_preflight.py'):
        assert file_hash(ROOT/'src'/name)==file_hash(frozen/'src'/name),name
    for name in ('preview_accounting_audit_v35.py','sleep_copy_audit_v35.py','placement_trace_audit_v31.py','home_entry_audit_v33.py'):
        assert (ROOT/'harness'/name).read_bytes()==(frozen/'harness'/name).read_bytes(),name
    old=(frozen/'harness/continuous_audit_v35.py').read_text();new=(ROOT/'harness/continuous_audit_v36.py').read_text()
    resource="        checks['assigned_gpu']=m['gpu']==assignment['gpu'] and assignment['gpu_uuid']==started['gpu']['uuid']==result['gpu_at_start']['uuid'] and assignment['run_id']==m['run_id']==run.name"
    replacement="        if m['execution']['mode']=='headless':\n            from headless_audit_v36 import receipts\n            checks['assigned_headless_cpu']=receipts(run,m,assignment,started,result)\n        else:\n    "+resource
    assert new.count(replacement)==1
    new=new.replace(replacement,resource)
    physical=lambda t:t[t.index("        checks['source_snapshot']"):t.index('        for p in [run/n')]
    assert physical(old)==physical(new)
    assert (ROOT/'harness/continuous_budget_v36.py').read_bytes()==(frozen/'harness/continuous_budget_v35.py').read_bytes()
    for n,h in json.loads((ROOT/'harness/configs/v35/BINDINGS.json').read_text())['configs'].items():
        assert file_hash(ROOT/n.replace('/v35/','/v36/'))==h
    return dict(protected_files=len(ledger),protected_unchanged=True,changed_source_files=changed,new_source_files=sorted(set(after)-set(before)),changed_runner_functions=['main','setup_scene','record_trace','tick'],all_other_functions_AST_exact=True,physical_auditor_predicates_identical_except_resource=True,reviewed_helpers_exact=True,rendered_default_preserved=True,normalized_identity_claim=False,physical_runs=0,formal_credit=False)



def verify(root):
    root=Path(root);out=root/'harness/evidence/v36'
    files=json.loads((out/'runtime-sha256.json').read_text())
    for n,h in files.items():assert file_hash(root/n)==h,n
    assert (out/'runtime-files.txt').read_text().splitlines()==list(files)+['harness/evidence/v36/runtime-sha256.json','harness/evidence/v36/runtime-files.txt']
    b=json.loads((root/'harness/configs/v36/BINDINGS.json').read_text())
    assert source_files(root/'src')==b['source_sha256'] and digest(b['source_sha256'])==b['implementation_sha256']
    for n,h in b['runtime_dependencies'].items():assert file_hash(root/n)==h,n
    final=json.loads((out/'FINAL_BINDING.json').read_text())
    for n,h in final['files_sha256'].items():
        if (root/n).is_file():assert file_hash(root/n)==h,n
        elif (root/'tests').exists():raise AssertionError('missing final file:'+n)
    if (root/'tests').exists():
        for n,h in final['test_sha256'].items():assert file_hash(root/n)==h,n
    if (out/'qualified-regression.json').exists():
        qualified=json.loads((out/'qualified-regression.json').read_text())
        for n,h in qualified['sha256'].items():assert file_hash(root/n)==h,n
    return dict(verified=True,runtime_files=len(files)+2,implementation_sha256=b['implementation_sha256'],
        final_binding_sha256=file_hash(out/'FINAL_BINDING.json'),formal_credit=False)


def seal(log,start,timing,package):
    out=ROOT/'harness/evidence/v36';b=json.loads((ROOT/'harness/configs/v36/BINDINGS.json').read_text())
    for n,h in b['runtime_dependencies'].items():assert file_hash(ROOT/n)==h,n
    for p in (log,start,timing,package):assert p.resolve().parent==out.resolve()
    assert log.read_text().rstrip().endswith('OK'),'full CPU suite did not pass'
    t=json.loads(timing.read_text());assert t['exit_code']==0 and t['physical_runs']==0
    hashes=json.loads(start.read_text());assert all(file_hash(ROOT/n)==h for n,h in hashes.items()),'changed after CPU test start'
    receipt=json.loads(package.read_text());assert receipt['verified'] and receipt['physical_runs']==0
    assert receipt['runtime_dependencies']==b['runtime_dependencies'] and receipt['source_sha256']==b['source_sha256']
    write_once(out/'qualified-regression.json',dict(log=str(log.relative_to(ROOT)),start=str(start.relative_to(ROOT)),timing=str(timing.relative_to(ROOT)),package=str(package.relative_to(ROOT)),
        sha256={str(p.relative_to(ROOT)):file_hash(p) for p in (log,start,timing,package)},formal_credit=False))
    write_once(out/'preservation-001.json',preservation())
    paths={ROOT/n for n in b['runtime_dependencies']}|{ROOT/'harness/configs/v36/BINDINGS.json',ROOT/'HANDOFF.md',ROOT/'harness/parent_commands_v36.md',ROOT/'harness/qualify_v36.py'}|set(out.glob('*'))
    files={str(p.relative_to(ROOT)):file_hash(p) for p in sorted(paths) if p.is_file()}
    tests={str(p.relative_to(ROOT)):file_hash(p) for p in sorted((ROOT/'tests').glob('*.py'))}
    write_once(out/'FINAL_BINDING.json',dict(schema='v36-final-binding/1',implementation_sha256=b['implementation_sha256'],files_sha256=files,test_sha256=tests,
        physical_runs=0,full10_15_20_goals_open=True,production_gate_unlocked=False,formal_credit=False))
    files['harness/evidence/v36/FINAL_BINDING.json']=file_hash(out/'FINAL_BINDING.json');files=dict(sorted(files.items()))
    runtime_names=set(b['runtime_dependencies'])|{'harness/configs/v36/BINDINGS.json','HANDOFF.md','harness/parent_commands_v36.md','harness/evidence/v36/FINAL_BINDING.json'}
    files={n:h for n,h in files.items() if n in runtime_names}
    write_once(out/'runtime-sha256.json',files)
    names=list(files)+['harness/evidence/v36/runtime-sha256.json','harness/evidence/v36/runtime-files.txt']
    with (out/'runtime-files.txt').open('x') as f:f.write('\n'.join(names)+'\n')
    frozen=ROOT/'harness/revisions/v36-final-001';frozen.mkdir()
    for name in sorted(set(names)|set(json.loads((out/'FINAL_BINDING.json').read_text())['files_sha256'])):
        p=frozen/name;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/name,p)
    shutil.copytree(ROOT/'tests',frozen/'tests',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    for folder in (frozen,out):
        for p in folder.rglob('*'):
            if p.is_file():p.chmod(0o444)
    return verify(frozen)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=('bindings','preservation','seal','verify'));p.add_argument('--root',type=Path,default=ROOT)
    for n in ('log','start','timing','package'):p.add_argument('--'+n,type=Path)
    a=p.parse_args()
    print(json.dumps(bindings() if a.action=='bindings' else preservation() if a.action=='preservation' else verify(a.root) if a.action=='verify' else seal(a.log,a.start,a.timing,a.package)))
