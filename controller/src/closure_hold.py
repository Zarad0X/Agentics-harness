"""Isolated candidate: bounded guarded physical settling, never command replay.

CPU tests establish control flow only. The 1000-tick cap and 25-tick dwell
need physical validation. No actuator writes or state restoration occur here.
"""
import math
from preview_errors import CandidateInfeasible

MIN_TICKS = 100                 # preserve original mandatory hold
MAX_TICKS = 1000                # 4 s at the required 250 Hz; finite diagnostic cap
DWELL_TICKS = 25                # 0.1 s of consecutive native velocity readbacks
MAX_SPEED = .01                 # unchanged acceptance threshold


def settle_closure(*, tick, measure, guard, controls, emit, dt, regulate=None):
    # Same absolute tolerance as ContactEnvelope and return_motion: native
    # float32 .004 reads back as .004000000189989805. Do not round dt.
    if not math.isfinite(dt) or abs(dt-.004)>1e-9:
        raise CandidateInfeasible('gripper_hold_timestep')
    baseline = controls()
    consecutive = 0
    for elapsed in range(MAX_TICKS+1):
        if controls() != baseline:
            raise CandidateInfeasible('gripper_hold_controls_changed')
        guard()                 # same collision guard before any authorized tick
        row = measure()         # authoritative native velocity, never finite difference
        numbers = [*row['finger_velocity'], *row['finger_position'],
                   row['position_error_m'], row['rotation_error_rad']]
        if len(row['finger_velocity']) != 2 or len(row['finger_position']) != 2 or not all(math.isfinite(v) for v in numbers):
            raise CandidateInfeasible('gripper_hold_nonfinite_measurement')
        if row['position_error_m'] > .005 or row['rotation_error_rad'] > .04:
            emit('gripper_settle', elapsed_ticks=elapsed, outcome='pose_guard_failed', **row)
            raise CandidateInfeasible('gripper_hold_pose_guard')
        stable = max(abs(v) for v in row['finger_velocity']) <= MAX_SPEED and row['bilateral_contact'] is True
        # The initial sample is not a completed physical tick.
        changed = bool(regulate(row,elapsed)) if regulate is not None and elapsed<MAX_TICKS else False
        if changed:baseline=controls()
        consecutive = consecutive+1 if elapsed and stable and not changed else 0
        accepted = elapsed >= MIN_TICKS and consecutive >= DWELL_TICKS
        emit('gripper_settle', elapsed_ticks=elapsed, consecutive_ticks=consecutive,
             outcome='settled' if accepted else 'holding',
             legacy_budget_exhausted_unsettled=(elapsed == 500 and not stable), **row)
        if accepted:
            return dict(elapsed_ticks=elapsed, consecutive_ticks=consecutive)
        if elapsed == MAX_TICKS:
            emit('gripper_settle_failed', elapsed_ticks=elapsed, reason='gripper_not_settled', **row)
            raise CandidateInfeasible('gripper_not_settled')
        tick()                  # exactly one existing guarded physical step; exception propagates
