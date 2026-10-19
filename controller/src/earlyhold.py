"""One diagnostic early hold; empirical guards stay authoritative.
No native access. A proposal uses a detached controller, never a commit.
"""
import math
from loaded_common import digest
from preview_errors import CandidateInfeasible

CONTRACT=dict(schema='earlyhold-diagnostic/1',trigger='first_proposed_increase_after_prior_reduction_in_lowering',max_interventions=1,hold_max_ticks=250,endpoint_ticks=100,physical_validation=False)

def checkpoint(tracking):
    return dict(index=tracking.index,cursor=tracking.cursor,reference=tracking.reference.tolist(),bias=tracking.bias.tolist(),last_drive=tracking.last_drive.tolist(),previous_q=None if tracking.previous_q is None else tracking.previous_q.tolist(),stalls=tracking.stalls,budget=tracking.budget)

class EarlyHold:
    def __init__(self,enabled,phase):
        if type(enabled) is not bool:raise ValueError('earlyhold_enabled_type')
        self.enabled=enabled;self.phase=phase;self.previous_factor=None;self.reduced=False;self.interventions=0;self.last=None

    def select(self,tracking,supervisor,state,factors,slip):
        # Caller has already evaluated the original Supervisor, including reserve.
        if supervisor.last is None or supervisor.last['index']!=tracking.index or supervisor.last['state_sha256']!=digest(state):
            raise CandidateInfeasible('earlyhold_supervisor_order')
        before=checkpoint(tracking)
        self.last=dict(schema=CONTRACT['schema'],enabled=self.enabled,phase=self.phase,index=tracking.index,
            before=before,previous_factor=self.previous_factor,reduced=self.reduced,interventions=self.interventions,
            supervisor=slip,normal_proposal=None,trigger=False,selected=None)
        try:proposal=tracking.propose(state,advance_factors=factors)
        except CandidateInfeasible as exc:
            self.last['rejected_trials']=getattr(exc,'proposal_trials',[])
            raise
        factor=proposal['trials'][-1]['factor']
        trigger=bool(self.enabled and self.phase=='lowering' and supervisor.mode=='normal' and self.reduced and
                     self.previous_factor is not None and factor>self.previous_factor)
        self.last.update(normal_proposal=proposal,trigger=trigger)
        decision=proposal
        if trigger:
            if self.interventions or supervisor.hold_start is not None:
                raise CandidateInfeasible('earlyhold_recurrent_intervention')
            # Same measurement, original complete Tracking predicates, unchanged PI.
            try:decision=tracking.propose(state,advance_factors=[0.])
            except CandidateInfeasible as exc:
                self.last['rejected_trials']=getattr(exc,'proposal_trials',[])
                raise
            supervisor.mode='hold';supervisor.hold_start=tracking.index
            supervisor.hold_brake_ticks=math.ceil(slip['brake_horizon_s']/.004)
            supervisor.last_witness.update(mode='hold',hold_start=tracking.index,hold_brake_ticks=supervisor.hold_brake_ticks,advance_factors=[0.])
            slip=dict(supervisor.last_witness);self.interventions+=1
        if checkpoint(tracking)!=before:raise RuntimeError('earlyhold_probe_mutated_controller')
        self.last.update(selected=decision,after_probe=checkpoint(tracking),interventions_after=self.interventions)
        return dict(decision,slip=slip,earlyhold=dict(self.last))

    def accept(self,decision):
        # Called only with the actually issued and observed decision.
        factor=decision['trials'][-1]['factor']
        if self.previous_factor is not None and factor<self.previous_factor:self.reduced=True
        self.previous_factor=factor
