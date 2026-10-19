"""CPU-only HOME profile admission; existing clocks, caps and guards unchanged.

This is computational admission, not a clearance or physical completion proof.
Do not cache it across measured states, reuse certificates or substitute routes.
"""
import numpy as np
from motion_refinement import vertices, command_count, profile, MAX_VERTICES
from preview_errors import CandidateInfeasible
from provenance import digest


def checked_vertices(path):
    # Bound both dimensions before float conversion, including ragged inputs.
    if not isinstance(path, (list, tuple, np.ndarray)):
        raise CandidateInfeasible('motion_refinement_nonfinite_or_shape')
    if isinstance(path, np.ndarray) and path.ndim != 2:
        raise CandidateInfeasible('motion_refinement_nonfinite_or_shape')
    if not 1 <= len(path) <= MAX_VERTICES:
        raise CandidateInfeasible('motion_refinement_vertex_budget')
    for row in path:
        if isinstance(row, np.ndarray) and row.ndim != 1:
            raise CandidateInfeasible('motion_refinement_nonfinite_or_shape')
        if not isinstance(row, (list, tuple, np.ndarray)) or len(row) != 7:
            raise CandidateInfeasible('motion_refinement_nonfinite_or_shape')
        if any(not isinstance(x, (int, float, np.integer, np.floating)) or isinstance(x, (bool, np.bool_)) for x in row):
            raise CandidateInfeasible('motion_refinement_nonfinite_or_shape')
    try:
        return vertices(path).tolist()
    except (ValueError, TypeError, OverflowError) as exc:
        raise CandidateInfeasible('motion_refinement_nonfinite_or_shape') from exc


def profiles(path, speed):
    """Same profile implementation/order as prepare_coarse, including splices.

    Count BOTH clocks before either dense allocation. No policy escalation.
    Returned arrays and profile receipts match the old accepted inputs exactly.
    """
    original = checked_vertices(path)
    if type(speed) not in (int, float) or speed not in (.1, .06):
        raise CandidateInfeasible('home_coarse_unsupported_nominal_speed')
    command_count(original)
    if speed != .1:
        command_count(original, speed)
    dense, velocity, original_clock = profile(original)
    if speed == .1:
        clock = original_clock
    else:
        dense, velocity, clock = profile(original, speed)
    return original, dense, velocity, original_clock, clock


def admit(path, speed):
    original, _, _, original_clock, clock = profiles(path, speed)
    return dict(schema='home-command-admission/1', path_sha256=digest(original),
                nominal_speed_rad_s=speed, original_profile=original_clock,
                nominal_profile=clock, computational_only=True,
                measured_path_recheck_required=True, physical_completion_proved=False)
