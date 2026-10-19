"""Standalone parent receipt checks; does not unlock the full-episode auditor."""
import argparse
import copy
import json
from pathlib import Path


def receipt(row):
    assert row['contract']=='separation-sleep-hypotheses/1'
    assert row['source_unchanged'] is True and row['raw_state_discarded'] is False
    assert row['conservative_bound_proved'] is False and row['contact_history_equivalent'] is False
    raw=row['measured_source'];expected=copy.deepcopy(raw)
    ambiguous=[k for k,v in raw.items() if v['sleeping'] and any(v['linear']+v['angular'])]
    assert set(row['ambiguous_ids'])==set(ambiguous)
    assert row['mode'] in ('exact','sleep_zero','awake_raw')
    assert row['approximate'] is (row['mode']!='exact')
    if row['mode']!='exact':assert ambiguous
    if row['mode']=='sleep_zero':
        for k in ambiguous:expected[k].update(linear=[0,0,0],angular=[0,0,0])
    elif row['mode']=='awake_raw':
        for k in ambiguous:expected[k]['sleeping']=False
    else:assert not ambiguous
    assert row['expected_destination']==row['observed_destination']==expected
    return row['mode']


def audit(report):
    assert report['schema']=='native-forecast-pair/1'
    assert report['status']=='PAIRED_TRANSITION_PASS'
    assert report['formal_credit'] is False and report['goal_complete'] is False
    assert report['full_run_authorized_by_probe'] is False
    assert report['scratch_source_before']==report['scratch_source_after']
    assert report['source_pack_before_sha256']==report['source_pack_after_sha256']
    assert report['source_pack_unchanged'] is True
    identity=report['native_system_identity']
    assert identity['independent'] is True
    assert len(identity['roles'])==len(set(identity['retained_wrapper_ids']))==4
    assert identity['repeated_accessor_same']==[[True]*4]*3
    expected_pairs={(identity['roles'][i],identity['roles'][j]) for i in range(4) for j in range(i+1,4)}
    assert len(identity['pairwise'])==6
    assert {(p['left'],p['right']) for p in identity['pairwise']}==expected_pairs
    assert all(p['same_wrapper'] is False and p['same_system_equal'] is False for p in identity['pairwise'])
    results=report['results'];baseline=results['baseline'];candidate=results['candidate']
    assert baseline['failure']=='RuntimeError:forecast_sleep_copy_readback_mismatch'
    assert candidate['failure'] is None and candidate['separation_continuation_completed'] is True
    prefix={k:next(b for b in results[k]['boundaries'] if b['nested']) for k in results}
    assert all(prefix['baseline'][f]==prefix['candidate'][f] for f in ('source','action_sha256','action_count'))
    physics=report['physics']
    assert physics['physics_steps']==sum(physics['steps_by_role'].values())<=40000
    assert physics['steps_by_role']['source_fixture']==4251
    assert sum(r['preview_steps'] for r in results.values())==physics['physics_steps']-4*4251
    assert baseline['scope']=='strict_copy_only_at_same_live_grasp_boundary'
    assert baseline['no_temporary_drive_created'] and baseline['source_unchanged'] and baseline['formal_steps']==0
    assert baseline['source_pack_before_sha256']==baseline['source_pack_after_sha256']
    assert candidate['formal_steps']==0 and candidate['source_restored'] and candidate['source_unchanged'] and candidate['no_live_drive']
    prior=None;trials=[];bundles=[]
    for event in candidate['events']:
        if event['kind']=='forecast_initialization':receipt(event);prior=event
        elif event['kind']=='separation_preview':
            assert prior and prior['mode']==event['initialization']
            trials.append((event,prior));prior=None
        elif event['kind']=='separation_profile_bundle':
            if event['accepted']:
                assert event['required_initializations']==['sleep_zero','awake_raw']
                pair=trials[-2:];assert len(pair)==2
                assert [a['initialization'] for a,b in pair]==event['required_initializations']
                assert pair[0][1]['measured_source']==pair[1][1]['measured_source']
                for a,b in pair:
                    assert a['accepted'] and a['profile']==event['profile'] and a['path_sha256']==event['path_sha256']
                bundles.append(event)
    assert bundles
    assert any(e['kind']=='separation_measured' and e['clear'] and e['position_error_m']<=.005 and e['rotation_error_rad']<=.04 for e in candidate['events'])
    return dict(paired_native_transition_verified=True,full_episode_verified=False,
                contact_history_equivalence_claim=False,formal_credit=False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('report',type=Path)
    args=p.parse_args()
    from forecast_report_io_v36 import load_report
    print(json.dumps(audit(load_report(args.report)),indent=2))
