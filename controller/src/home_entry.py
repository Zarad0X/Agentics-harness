"""Opt-in retained-target HOME admission dwell using the existing vertex guard.

The path, initial drive target and world origin are never rebased. A stationary
hold is covered by the existing whole-curve certificate (tracking tube plus
step reserve); progress additionally requires the stricter brake admission.
No physical convergence or stopping guarantee is inferred from that geometry.
"""
import numpy as np
from home_progress import POLICY
from return_clearance import LINK_TUBE_M
from vertex_governor import readiness
from preview_errors import CandidateInfeasible


def assessment(certifier,q,state):
    actual=np.asarray(state['robots'][certifier.arm]['q'],float)
    measured=readiness(certifier,q,actual,None)
    if measured['whole_link_m'] is None:raise CandidateInfeasible('home_entry_invalid_geometry')
    fingers=np.clip(actual[-2:],0,.04).tolist()
    shapes=certifier.world.robot_shapes(certifier.arm,actual[:7].tolist(),fingers)
    steps=certifier.step_bounds(certifier.arm,actual[:7].tolist(),fingers,shapes)
    totals={n:measured['link_displacement_m'][n]+POLICY['numerical_m']+steps[n] for n in steps}
    return dict(required=any(v>LINK_TUBE_M for v in totals.values()),
        link_tracking_m=measured['link_displacement_m'],one_step_reserve_m=steps,
        tracking_plus_step_m=totals,limit_m=LINK_TUBE_M,numerical_m=POLICY['numerical_m'])


def prepare(api,governor,progress,certifier,row,trace):
    """Called only before decision zero, after the original hard admission.

    Existing 500 per-vertex / 5000 shared hold budgets and feedback withdrawal
    apply. A failed hold propagates immediately; no cleanup command or tick.
    """
    if not getattr(api,'home_entry_hold',False):return False
    if progress.commands!=0 or not progress.initial:raise CandidateInfeasible('home_entry_not_initial')
    state=api.state();before=assessment(certifier,progress.q,state)
    api.event('home_entry_admission',arm=governor.arm,object_id=governor.target,motion_phase='home',
        scene_step=api.telemetry()['scene_step_count'],reference_q=progress.q.tolist(),assessment=before,state=state,
        policy='existing vertex retained-target recovery; shared budgets; no rebase',formal_credit=False)
    if not before['required']:return False
    checkpoint=dict(vertex=0,reference_index=0,resume_index=1,prefix_packets=trace.count,
        prefix_sha256=trace.hash.hexdigest(),packet_sha256=__import__('provenance').digest(row))
    # No new target: the first certified vertex equals the live retained drive.
    governor.hold(progress.q.copy(),0,1,certifier,checkpoint)
    progress.resumed()
    state=api.state();after=assessment(certifier,progress.q,state)
    api.event('home_entry_ready',arm=governor.arm,object_id=governor.target,motion_phase='home',
        scene_step=api.telemetry()['scene_step_count'],reference_q=progress.q.tolist(),assessment=after,state=state,
        completed_hold_ticks=governor.added,formal_credit=False)
    if after['required']:raise CandidateInfeasible('home_entry_no_brake_margin_after_hold')
    return True
