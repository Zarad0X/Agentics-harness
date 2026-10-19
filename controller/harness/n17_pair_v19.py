"""Strict observer OFF/ON pairing and transition evidence; never acceptance.

This measurement pair resolves temporal observability and checks reproducibility.
It does not experimentally isolate the solver's wake or support mechanism.
"""
import argparse
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from provenance import digest, file_hash, source_files, write_once
from n17_observer import CAPS, NAME
from home_trace_v18 import validate_trace


def observation_checks(receipt, preview):
    checks = {}
    o = receipt.get('observation') or {}
    rows = o.get('tail', [])
    checks['schema_identity'] = (receipt.get('schema') == 'n17-observation/19'
        and receipt.get('actor_name') == NAME and receipt.get('object_id') == 17
        and receipt.get('motion_target') == 10 and receipt.get('motion_phase') == 'home')
    checks['complete'] = o.get('complete') is True and o.get('errors') == []
    checks['bounded'] = (o.get('caps') == CAPS and 0 < len(rows) <= CAPS['tail']
        and len(o.get('windows', [])) <= CAPS['windows']
        and all(len(w['rows']) <= CAPS['pre'] + 1 + CAPS['post'] for w in o.get('windows', [])))
    checks['metadata_binding'] = bool(o.get('metadata')) and digest(o['metadata']) == o.get('metadata_sha256')
    checks['clock_count'] = (bool(rows) and rows[-1]['scene_step'] == receipt.get('final_scene_step')
        and o.get('samples') == receipt.get('final_scene_step', 0) - receipt.get('initial_scene_step', 0) + 1)
    all_windows = [rows] + [w['rows'] for w in o.get('windows', [])]
    checks['consecutive_retained_windows'] = all(all(b['scene_step'] == a['scene_step'] + 1
        for a, b in zip(window, window[1:])) for window in all_windows)
    seen = {}
    for row in (r for window in all_windows for r in window):
        seen.setdefault(row['scene_step'], []).append(row)
    checks['overlap_exact'] = all(all(r == group[0] for r in group) for group in seen.values())
    checks['contacts_exact_identity_and_alignment'] = all(
        r['contact_step'] == r['scene_step'] and not r['contacts_truncated']
        and all(any(b['name'] == NAME and b['arm'] is None for b in c['bodies']) for c in r['contacts'])
        for group in seen.values() for r in group)
    checks['raw_timestep'] = bool(seen) and all(r['scene_timestep_s'] == o['metadata']['scene_timestep_s']
        and r['scene_timestep_s'] > 0 for group in seen.values() for r in group)
    checks['sleep_observable'] = bool(seen) and all(r['is_sleeping'].get('available') is True
        for group in seen.values() for r in group)
    home = [e for e in preview['trace'] if e['kind'] == 'motion_phase_exit' and e['motion_phase'] == 'home']
    checks['motion_alignment'] = len(home) == 1 and all(receipt.get(k) == home[0].get(k)
        for k in ('initial_scene_step', 'final_scene_step', 'reason'))
    if preview.get('failure_state') is not None and rows:
        obj = preview['failure_state']['objects']['17']
        checks['failure_state_exact'] = all(rows[-1][k] == obj[k] for k in
                                            ('pose', 'linear_velocity', 'angular_velocity'))
    return checks


def compare_previews(a, b, config):
    checks = {}
    for key in ('accepted', 'failure', 'trial_steps', 'arm', 'object_id', 'index', 'trial',
                'failure_state', 'source_unchanged', 'independent_scene'):
        checks['same_' + key] = a.get(key) == b.get(key)
    checks['independent_unchanged_sources'] = all(p.get('source_unchanged') is True
        and p.get('independent_scene') is True for p in (a, b))
    for kind, keys in (
        ('motion_phase_entry', ('arm', 'object_id', 'motion_phase', 'scene_step', 'state_sha256', 'controls_sha256')),
        ('return_motion_plan', ('motion_phase', 'path_sha256', 'certificate_sha256', 'dense_command_sha256',
            'dense_commands', 'start_state_sha256', 'controls_sha256', 'rng_sha256', 'initial_readback')),
        ('return_motion_trace', ('motion_phase', 'failed', 'reason', 'measured_trace_sha256', 'packet_count')),
        ('velocity_feedback_receipt', ('motion_phase', 'all_numeric_sha256', 'completed_ticks', 'actual_final_targets')),
        ('motion_phase_exit', ('motion_phase', 'initial_scene_step', 'final_scene_step', 'completed_ticks',
            'reason', 'state_sha256', 'controls_sha256', 'actual_velocity_targets', 'checkpoints'))):
        values = [[{k: e.get(k) for k in keys} for e in p['trace'] if e['kind'] == kind] for p in (a, b)]
        checks['same_' + kind] = len(values[0]) == 2 and values[0] == values[1]
    checks['motion_receipts_complete'] = all(validate_trace(p['trace'], config)['receipt_complete'] for p in (a, b))
    receipts = [[e for e in p['trace'] if e['kind'] == 'n17_observation'] for p in (a, b)]
    checks['observer_off_on'] = (all(len(r) == 1 for r in receipts)
        and [r[0]['mode'] for r in receipts] == ['baseline', 'observe']
        and receipts[0][0].get('observation') is None)
    obs_checks = observation_checks(receipts[1][0], b) if checks['observer_off_on'] else {'receipt_missing': False}
    return checks, obs_checks


def audit(off, on):
    runs = [Path(off), Path(on)]
    read = lambda p: json.loads(p.read_text())
    manifests = [read(r / 'diagnostic-manifest.json') for r in runs]
    configs = [read(r / 'config.json') for r in runs]
    checks = dict(config_identical=configs[0] == configs[1],
                  modes=[m.get('n17_observer') for m in manifests] == ['baseline', 'observe'])
    for k in ('snapshot_sha256', 'release_sha256', 'source_fingerprint', 'target', 'candidate_index',
              'max_previews', 'legacy_insertion', 'fixed_setpoint_diagnostic', 'continuation_diagnostic',
              'python', 'gpu', 'experience', 'config_sha256'):
        checks['same_' + k] = k in manifests[0] and manifests[0][k] == manifests[1].get(k)
    checks['request'] = all(m.get('target') == 10 and m.get('candidate_index') == 5
        and m.get('max_previews') == 1 and not m.get('legacy_insertion')
        and not m.get('fixed_setpoint_diagnostic') and not m.get('continuation_diagnostic') for m in manifests)
    checks['immutable_inputs'] = all(file_hash(r / 'config.json') == m['config_sha256']
        and all(file_hash(r / 'snapshot' / n) == h for n, h in m['snapshot_sha256'].items())
        for r, m in zip(runs, manifests))
    expected = source_files(ROOT / 'src')
    checks['auditor_matches_runtime_source'] = all(expected == {n[4:]: h for n, h in
        m['snapshot_sha256'].items() if n.startswith('src/')} for m in manifests)
    recon = [read(r / 'artifacts/reconstruction.json') for r in runs]
    checks['reconstruction'] = recon[0] == recon[1] and all(r.get('validated') is True for r in recon)
    starts = [read(r / 'started.json') for r in runs]
    ends = [read(r / 'terminal.json') for r in runs]
    assignments = [read(r / 'assignment.json') for r in runs]
    checks['terminal'] = all(e.get('state') == 'terminal' and e.get('reason') == 'exited' and e.get('exit_code') == 0 for e in ends)
    checks['same_actual_gpu'] = (bool(assignments[0].get('gpu_uuid'))
        and assignments[0].get('gpu_uuid') == assignments[1].get('gpu_uuid')
        and all(a.get('gpu_uuid') == s.get('gpu', {}).get('uuid') for a, s in zip(assignments, starts)))
    checks['sequential'] = any(starts[i]['time_unix'] + ends[i]['elapsed_s'] <= starts[1-i]['time_unix'] for i in (0, 1))
    events = [[json.loads(line) for line in (r / 'artifacts/events.jsonl').read_text().splitlines()] for r in runs]
    ps = [[e for e in rows if e['kind'] == 'return_preview'] for rows in events]
    checks['single_preview'] = all(len(p) == 1 and (p[0]['object_id'], p[0]['index'], p[0]['trial']) == (10, 5, 1) for p in ps)
    candidates = [[e for e in rows if e['kind'] == 'diagnostic_candidate'] for rows in events]
    checks['same_candidate'] = all(len(c) == 1 for c in candidates) and all(
        candidates[0][0].get(k) == candidates[1][0].get(k) for k in ('candidate', 'provenance'))
    paired, observed = compare_previews(ps[0][0], ps[1][0], configs[0]) if checks['single_preview'] else ({'missing': False}, {'missing': False})
    receipts = [e for e in ps[1][0]['trace'] if e['kind'] == 'n17_observation'] if checks['single_preview'] else []
    observation = (receipts[0].get('observation') or {}) if len(receipts) == 1 else {}
    transition_windows = [dict(trigger=w['trigger'], scene_step=w['scene_step'], remaining=w['remaining'],
        rows=[r for r in w['rows'] if abs(r['scene_step'] - w['scene_step']) <= 4])
        for w in observation.get('windows', [])]
    reproduced = checks['single_preview'] and all(p[0].get('failure') ==
        'CandidateInfeasible:postrelease_precommand_envelope:object_envelope:17'
        and p[0].get('trial_steps') == 32286 for p in ps)
    return dict(schema='n17-pair/19', checks=checks, paired_motion_checks=paired, observation_checks=observed,
        valid_measurement_pair=all(checks.values()) and all(paired.values()) and all(observed.values()),
        v18_failure_boundary_reproduced=reproduced, transition_windows=transition_windows,
        inputs=[dict(run=str(r), events_sha256=file_hash(r / 'artifacts/events.jsonl'), terminal=e) for r, e in zip(runs, ends)],
        causal_physics_mechanism_established=False, formal_credit=False, whole_episode_success=False,
        limitation='Exact observed-state/control equality is not proof of identical hidden solver state. '
            'A positive-gap zero-impulse pair is not collision proof. OFF/ON isolates measurement only; '
            'sleep/support/contact-offset causal interventions require new evidence and a separate frozen diagnostic.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('off', type=Path)
    parser.add_argument('on', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.off, args.on)
    write_once(args.output, result)
    print(json.dumps({k: v for k, v in result.items() if k.endswith('checks') or k == 'valid_measurement_pair'}))
    raise SystemExit(0 if result['valid_measurement_pair'] else 2)
