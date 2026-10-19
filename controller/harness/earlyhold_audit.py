"""Independent decision reconstruction; no runtime EarlyHold or Tracking imports.
Existing independent Supervisor and PI arithmetic are retained. Requires a real
chronological prefix from admission; checkpoint-seeded probes are not acceptance.
"""
import math,hashlib
from pathlib import Path
from slip_audit import Replay as SlipReplay
from loaded_tracking_audit import Replay as PIReplay,equivalent,require
from loaded_common import digest

CONTRACT=dict(schema='earlyhold-diagnostic/1',trigger='first_proposed_increase_after_prior_reduction_in_lowering',max_interventions=1,hold_max_ticks=250,endpoint_ticks=100,physical_validation=False)

class Replay(SlipReplay):
    def __init__(self,*args,enabled=False,phase='lowering',**kwargs):
        super().__init__(*args,**kwargs)
        require(type(enabled) is bool,'earlyhold_enabled_type')
        self.enabled=enabled;self.phase=phase;self.factor_before=None;self.saw_reduction=False;self.used=0;self.evidence=None
    def checkpoint(self):
        return dict(index=self.count,cursor=self.at,reference=self.ref.tolist(),bias=self.bias.tolist(),last_drive=self.drive.tolist(),previous_q=None if self.previous_q is None else self.previous_q.tolist(),stalls=self.stalls,budget=self.budget)
    def next(self,state):
        before=self.checkpoint()
        self.evidence=None
        # This independently executes hard, history, reserve, recurrence and hold guards first.
        try:normal=super().next(state)
        except ValueError:
            if 'advance_factors' in self.last_slip:
                self.evidence=dict(schema='earlyhold-diagnostic/1',enabled=self.enabled,phase=self.phase,index=self.count,before=before,
                    previous_factor=self.factor_before,reduced=self.saw_reduction,interventions=self.used,supervisor=self.last_slip,
                    normal_proposal=None,trigger=False,selected=None)
            raise
        slip=normal.pop('slip')
        f=normal['trials'][-1]['factor']
        trigger=bool(self.enabled and self.phase=='lowering' and self.stage=='normal' and self.saw_reduction and self.factor_before is not None and f>self.factor_before)
        ev=dict(schema='earlyhold-diagnostic/1',enabled=self.enabled,phase=self.phase,index=self.count,before=before,
            previous_factor=self.factor_before,reduced=self.saw_reduction,interventions=self.used,supervisor=slip,
            normal_proposal=normal,trigger=trigger,selected=None)
        self.evidence=ev
        chosen=normal
        if trigger:
            require(self.used==0 and self.start_hold is None,'earlyhold_recurrent_intervention')
            chosen=PIReplay.next(self,state,advance_factors=[0.])
            self.stage='hold';self.start_hold=self.count;self.brake_count=math.ceil(slip['brake_horizon_s']/.004);self.used+=1
            slip=dict(slip,mode='hold',hold_start=self.count,hold_brake_ticks=self.brake_count,advance_factors=[0.]);self.last_slip=slip
        require(self.checkpoint()==before,'earlyhold_probe_mutation')
        ev.update(selected=chosen,after_probe=self.checkpoint(),interventions_after=self.used)
        return dict(chosen,slip=slip,earlyhold=ev)
    def accept(self,d):
        f=d['trials'][-1]['factor']
        if self.factor_before is not None and f<self.factor_before:self.saw_reduction=True
        self.factor_before=f
        super().accept(d)

def source_binding(root,expected):
    root=Path(root)
    require(bool(expected),'missing_source_binding')
    for rel,h in expected.items():
        p=root/rel
        require(p.is_file() and not p.is_symlink() and hashlib.sha256(p.read_bytes()).hexdigest()==h,'source_binding:'+rel)

def verify_decision(replay,state,issued):
    expected=replay.next(state)
    require(equivalent(expected,issued),'earlyhold_decision_or_history')
    return expected

def start(row,world_factory,enabled):
    # Existing source geometry/admission verifier, not a receipt marker.
    from loaded_audit import base_start as original_start
    require(row['binding'].get('earlyhold')==dict(CONTRACT,enabled=enabled),'earlyhold_admission_contract')
    active=original_start(row,world_factory,'B_clearance')
    active['tracking']=Replay(active['a'],active['commands'],row['state'],row['controls'],row['binding']['profile'],row['native_ee'],enabled=enabled,phase=row['phase'])
    return active

def tick(active,row):
    from loaded_audit import base_tick as original_tick
    validate_contacts(row['contacts']['target_contact_observation'],int(active['a'].target))
    original_tick(active,row)

def terminal(active,row):
    # Preserve original rejection checks and endpoint gates. Rejected normal/zero
    # candidates require independent reproduction, including unissued evidence.
    from loaded_audit import base_terminal as original_terminal
    rejected=row['tracking_terminal']['rejected_command']
    if rejected is not None:
        import copy
        probe=copy.deepcopy(active['tracking'])
        try:probe.next(rejected['pre_state'])
        except ValueError:
            if probe.evidence is not None:
                ev=dict(probe.evidence,rejected_trials=probe.last_trials)
                require(equivalent(ev,rejected.get('earlyhold')),'earlyhold_rejected_evidence')
        else:raise ValueError('earlyhold_unreproducible_rejection')
    validate_contacts(row['contacts']['target_contact_observation'],int(active['a'].target))
    return original_terminal(active,row)

def validate_contacts(obs,target):
    # Structural telemetry audit, not independent physical contact truth.
    require(obs.get('schema')=='target-identity-contacts/1' and obs.get('target')==target,'contact_observation_identity')
    require(obs.get('available') is True,'contact_observation_unavailable')
    pairs=obs['pairs'];require(obs['target_pairs']==len(pairs) and obs['target_points']==sum(len(p['points']) for p in pairs),'contact_observation_count')
    require(len({p['scene_pair_index'] for p in pairs})==len(pairs),'contact_observation_duplicate')
    for p in pairs:
        require(0<=p['scene_pair_index']<obs['total_scene_pairs'] and bool(p['target_body_indices']),'contact_observation_scope')
        require(all(0<=i<len(p['bodies']) and p['bodies'][i]==obs['actor_name'] for i in p['target_body_indices']),'contact_observation_body')
        for x in p['points']:
            require(all(len(x[k])==3 and all(math.isfinite(v) for v in x[k]) for k in ('position','normal','impulse')) and math.isfinite(x['separation']),'contact_observation_point')
    require(obs['force_readback'] is None and obs['configured_limits_are_measurements'] is False,'contact_force_claim')
