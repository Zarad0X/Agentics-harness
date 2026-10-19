"""Versioned bounded layout funnel and immutable held-out evaluation contract."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from provenance import digest, file_hash, write_once, measured_scene_hash, SCENE_SCHEMA, LEGACY_SCENE_SCHEMA, legacy_measured_scene_hash
from run_config import RunConfig


def scene_key(m):
    return measured_scene_hash(m)


def create(output, policy):
    output=Path(output);output.mkdir();policy=Path(policy)
    snapshot=json.loads(policy.read_text())
    if snapshot['schema']!='roundtrip-experience/2' or any(r['split']!='development' for r in snapshot['records']):raise ValueError('policy leakage')
    write_once(output/'experience.json',snapshot)
    entries=[]
    for split,seeds in (('development',[101,102,103]),('heldout',[201,202,203,204,205,206])):
        for count in (10,15,20):
            for seed in seeds:
                for arm in ('layout','baseline','cache','experience'):
                    cfg=RunConfig(mode='layout' if arm=='layout' else 'roundtrip',layout='generated',count=count,seed=seed,split=split,
                                  optimize_geometry=arm in ('cache','experience'),experience=arm=='experience')
                    name=f'{split}-n{count}-s{seed}-{arm}.json'
                    h=write_once(output/name,cfg.to_dict());entries.append(dict(config=name,sha256=h,**dict(split=split,count=count,seed=seed,arm=arm)))
    plan=dict(schema='roundtrip-evaluation/3',policy_sha256=file_hash(output/'experience.json'),entries=entries,
        development_seeds=[101,102,103],heldout_seeds=[201,202,203,204,205,206],counts=[10,15,20],
        target_valid_distinct_per_count=3,max_layout_attempts=27,
        selection='first three independently measured valid distinct scenes in declared seed order; no performance-based replacement',
        execution='one layout attempt per split/count/seed; complete bounded funnel, retain all attempts; no unbounded replacement seeds',
        comparison=['baseline: cache off, experience off','cache: cache on, experience off','experience: cache on, frozen experience on'],
        physical_gate='actual dense roundtrips still require accepted original smoke and whole full10 roundtrip',
        freeze='policy and configs frozen before ANY heldout launch; heldout evidence cannot train this policy',
        accounting='layout validity is not pick/place success; count invalid spawn/layout, initialization/evidence error, timeout, cancellation, launch failure and pending',
        shortfall='fewer than three measured valid distinct scenes is an unresolved target, never success',
        pairing='require exact measured object/robot states, cooked geometry, assets, contacts, container; seed equality alone insufficient')
    write_once(output/'plan.json',plan)
    return plan


def check_plan(config, plan_path, experience=None):
    plan_path=Path(plan_path);plan=json.loads(plan_path.read_text())
    if plan['schema']!='roundtrip-evaluation/3':raise ValueError('evaluation schema')
    if file_hash(plan_path.parent/'experience.json')!=plan['policy_sha256']:raise ValueError('frozen_policy_changed')
    hashes={digest(config.to_dict())}
    # Frozen pre-v9 plans omitted the unchanged scale-1 default. Never admit
    # scale 2 through a historical declaration.
    if not config.high_transfer_branch_seed:
        legacy=config.to_dict();legacy.pop('high_transfer_branch_seed');hashes.add(digest(legacy))
        if config.retreat_time_scale==1:
            legacy.pop('retreat_time_scale');hashes.add(digest(legacy))
    if not any(e['sha256'] in hashes for e in plan['entries']):raise ValueError('config_not_predeclared')
    if config.experience and (experience is None or file_hash(experience)!=plan['policy_sha256']):raise ValueError('policy_not_prefrozen')
    return plan


def funnel(root, plan_path, output, *, fingerprint_schema=SCENE_SCHEMA):
    if fingerprint_schema not in (SCENE_SCHEMA,LEGACY_SCENE_SCHEMA):raise ValueError('unknown_fingerprint_schema')
    legacy=fingerprint_schema==LEGACY_SCENE_SCHEMA
    plan_path=Path(plan_path);plan=json.loads(plan_path.read_text());rows=[];selected={};seen={}
    planned=[e for e in plan['entries'] if e['arm']=='layout']
    # A declared implementation recovery retains the failed physical attempt.
    # It is never a selectable layout or a second successful seed. Bind the
    # actual old receipt bytes, rather than relabeling a same-seed run by name.
    historical=[];prior_ids=set();recovery=plan.get('implementation_recovery',{})
    for prior in recovery.get('prior_attempts',[]):
        run=Path(root)/prior['run_id']
        for name,h in prior['files_sha256'].items():
            p=run/name
            if not p.is_file() or file_hash(p)!=h:raise ValueError('recovery_prior_missing_or_changed:'+prior['run_id']+':'+name)
        prior_ids.add(prior['run_id'])
        historical.append(dict(run_id=prior['run_id'],split=prior['split'],count=prior['count'],seed=prior['seed'],
            status=prior['status'],selected=False,attempt_role='prior_implementation_failure',
            evaluation_plan_sha256=prior['evaluation_plan_sha256']))
    runs={}
    for p in sorted(Path(root).glob('*/config.json')):
        if p.parent.name in prior_ids:continue
        cfg=json.loads(p.read_text())
        if cfg['mode']!='layout':continue
        key=(cfg['split'],cfg['count'],cfg['seed']);runs.setdefault(key,[]).append(p.parent)
    expected={(e['split'],e['count'],e['seed']) for e in planned}
    unexpected=[dict(run_id=r.name,split=k[0],count=k[1],seed=k[2],status='undeclared_attempt',selected=False) for k,rs in runs.items() if k not in expected for r in rs]
    schedule_errors=[]
    for e in planned:
        key=(e['split'],e['count'],e['seed']);group=f'{key[0]}-n{key[1]}'
        selected.setdefault(group,[]);seen.setdefault(group,set());attempts=runs.get(key,[])
        if len(attempts)>1:
            schedule_errors.append('duplicate_seed_attempts:'+str(key))
            for r in attempts:
                summary=json.loads((r/'summary.json').read_text()) if (r/'summary.json').exists() else {}
                rows.append(dict(run_id=r.name,split=key[0],count=key[1],seed=key[2],status='duplicate_attempt',underlying_status=summary.get('layout_status','pending'),selected=False))
            continue
        row=dict(split=key[0],count=key[1],seed=key[2],status='not_attempted',selected=False)
        if recovery:
            row['attempt_role']='implementation_recovery'
            row['prior_attempt_ids']=[r['run_id'] for r in historical if (r['split'],r['count'],r['seed'])==key]
        if attempts:
            run=attempts[0];row['run_id']=run.name
            if digest(json.loads((run/'config.json').read_text()))!=e['sha256']:raise ValueError('attempt_config_changed')
            rm=json.loads((run/'run-manifest.json').read_text())
            if rm.get('evaluation_plan_sha256')!=file_hash(plan_path):raise ValueError('plan_not_frozen_before_attempt')
            terminal=json.loads((run/'terminal.json').read_text()) if (run/'terminal.json').exists() else {}
            summary=json.loads((run/'summary.json').read_text()) if (run/'summary.json').exists() else {}
            reason=terminal.get('reason');row['status']=('timeout' if reason=='timeout' else 'cancelled' if reason=='cancelled' else summary.get('layout_status','launch_failure' if terminal else ('pending' if (run/'started.json').exists() else 'prepared')))
            if summary.get('layout_valid') and terminal.get('exit_code')==0:
                m=json.loads((run/'artifacts/scene-manifest.json').read_text());h=(legacy_measured_scene_hash(m) if legacy else scene_key(m));row['scene_sha256']=h
                if not legacy:row['fingerprint_schema']=SCENE_SCHEMA
                row['status']='valid' if h not in seen[group] else 'duplicate_layout'
                if h not in seen[group] and len(selected[group])<3:
                    entry=dict(run_id=run.name,seed=key[2],scene_sha256=h)
                    if not legacy:entry['fingerprint_schema']=SCENE_SCHEMA
                    selected[group].append(entry);row['selected']=True
                seen[group].add(h)
        rows.append(row)
    # Selection cannot be finalized with an earlier seed still pending/unattempted.
    rows+=unexpected
    complete=not schedule_errors and not unexpected and all(r['status'] not in ('not_attempted','pending','prepared') for r in rows)
    current_attempts=sum(r['status'] not in ('not_attempted','prepared') for r in rows)
    rows=historical+rows
    value=dict(schema='layout-funnel/3' if legacy else 'layout-funnel/5',plan_sha256=file_hash(plan_path),rows=rows,selected=selected,
        generation_attempts=current_attempts+len(historical),version_generation_attempts=current_attempts,
        prior_implementation_failures=len(historical),schedule_complete=complete,schedule_errors=schedule_errors,
        heldout_target_met=complete and all(len(selected[f'heldout-n{n}'])==3 for n in plan['counts']),
        selected_is_final=complete,whole_episode_success=False)
    if not legacy:value['fingerprint_schema']=SCENE_SCHEMA
    return write_once(output,value)


def selected_scene(config, plan_path, selection_path):
    value=json.loads(Path(selection_path).read_text())
    if (value.get('schema')!='layout-funnel/5' or value.get('fingerprint_schema')!=SCENE_SCHEMA
            or any(r.get('fingerprint_schema')!=SCENE_SCHEMA for entries in value['selected'].values() for r in entries)
            or any(r.get('fingerprint_schema')!=SCENE_SCHEMA for r in value.get('rows',[]) if 'scene_sha256' in r)):
        raise ValueError('unknown_or_mixed_selection_fingerprint_schema')
    if not value.get('evidence_verified'):raise ValueError('selection_requires_verified_regeneration')
    if value.get('plan_sha256')!=file_hash(plan_path) or not value.get('selected_is_final'):
        raise ValueError('layout_selection_not_frozen_or_final')
    group=f'{config.split}-n{config.count}'
    entries=[r for r in value['selected'].get(group,[]) if r['seed']==config.seed]
    if len(entries)!=1:raise ValueError('layout_not_selected')
    return dict(scene_sha256=entries[0]['scene_sha256'],fingerprint_schema=SCENE_SCHEMA,layout_run_id=entries[0]['run_id'],
                selection_sha256=file_hash(selection_path),plan_sha256=file_hash(plan_path))


def main():
    ap=argparse.ArgumentParser();sub=ap.add_subparsers(dest='cmd',required=True)
    p=sub.add_parser('create');p.add_argument('output',type=Path);p.add_argument('--policy',type=Path,required=True)
    p=sub.add_parser('funnel');p.add_argument('root',type=Path);p.add_argument('--plan',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=ap.parse_args()
    print(json.dumps(create(a.output,a.policy)) if a.cmd=='create' else funnel(a.root,a.plan,a.output))

if __name__=='__main__':main()
