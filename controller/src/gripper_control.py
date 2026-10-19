"""Bound finger drive-target changes, including first-contact preload."""
import math


def is_finger_contact(separation, impulse):
    """Sense a shallow loaded contact, not a speculative or deep pair.

This is a touch sensor only. It does not alter forbidden-collision thresholds.
Separation and impulse must describe the SAME physical contact point.
"""
    return (math.isfinite(separation) and math.isfinite(impulse)
            and -.0003 <= separation <= .0001 and impulse > 1e-6)


def advance_finger_targets(command, goals, *, dt=.004, max_speed=.01):
    if len(command) != 2 or len(goals) != 2:
        raise ValueError('two finger positions required')
    if not math.isfinite(dt) or not math.isfinite(max_speed) or min(dt, max_speed) <= 0:
        raise ValueError('positive finite time step and speed required')
    if any(not math.isfinite(q) or not 0 <= q <= .04 for q in (*command, *goals)):
        raise ValueError('finger target outside physical travel')
    step = dt * max_speed
    result = [q + max(-step, min(step, target - q)) for q, target in zip(command, goals)]
    reached = all(abs(q - target) <= 1e-12 for q, target in zip(result, goals))
    return result, reached
