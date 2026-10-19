"""Destination-only observable sleep transfer, after existing pose/velocity setters.

No steps, pose writes, velocity writes, normalization, or source setters here.
This copies a boolean, not wake counters, contacts, forces or solver history.
"""
import numpy as np


def _read(actor, body):
    pose = actor.get_pose()
    values = tuple(np.array(v, copy=True) for v in
                   (pose.p, pose.q, body.get_linear_velocity(), body.get_angular_velocity()))
    if any(v.shape != (n,) or not np.isfinite(v).all()
           for v, n in zip(values, (3, 4, 3, 3))) or not np.any(values[1]):
        raise RuntimeError('forecast_invalid_object_state')
    sleeping = body.is_sleeping  # Required native property; never infer from speed.
    if not isinstance(sleeping, (bool, np.bool_)):
        raise RuntimeError('forecast_invalid_sleep_flag')
    return values, bool(sleeping)


def _equal(a, b):
    return all(np.array_equal(x, y) for x, y in zip(a, b))


class ObjectSleepCopy:
    """Capture before *any* copy writes; restore once before new constraints.

    Call assert_source_unchanged again after preview execution. Destination flags
    are checked only at the copy boundary, not after attachment or stepping.
    The caller must enforce distinct physical systems (v32 already does).
    """
    def __init__(self, source_objects, destination_objects, dynamic_type):
        if not source_objects or source_objects.keys() != destination_objects.keys():
            raise RuntimeError('forecast_object_layout_mismatch')
        self.source = source_objects
        self.destination = destination_objects
        self.rows = {}
        for key, original in source_objects.items():
            actor = original.actor
            twin = destination_objects[key].actor
            body = actor.find_component_by_type(dynamic_type)
            copied = twin.find_component_by_type(dynamic_type)
            if body is None or copied is None:
                raise RuntimeError('forecast_missing_dynamic_body')
            for method in ('put_to_sleep', 'wake_up'):
                if not callable(getattr(copied, method, None)):
                    raise RuntimeError('forecast_missing_sleep_api:' + method)
            before = _read(actor, body)
            _read(twin, copied)  # Missing destination getter fails before writes.
            self.rows[key] = (actor, body, twin, copied, before)
        # Also reject cross-ID aliasing; source must never receive a setter.
        src = [r[i] for r in self.rows.values() for i in (0, 1)]
        dst = [r[i] for r in self.rows.values() for i in (2, 3)]
        if any(a is b for a in src for b in dst):
            raise RuntimeError('forecast_aliased_object')

    def assert_source_unchanged(self):
        if self.source.keys() != self.rows.keys():
            raise RuntimeError('forecast_modified_source_objects')
        for key, (actor, body, _, _, before) in self.rows.items():
            if self.source[key].actor is not actor:
                raise RuntimeError('forecast_modified_source_objects')
            now = _read(actor, body)
            if not _equal(now[0], before[0]) or now[1] != before[1]:
                raise RuntimeError('forecast_modified_source_object_state')

    def restore(self):
        try:
            if self.destination.keys() != self.rows.keys():
                raise RuntimeError('forecast_object_layout_mismatch')
            # Validate all raw copy readbacks before any sleep/wake calls.
            for key, (_, _, actor, body, before) in self.rows.items():
                if self.destination[key].actor is not actor:
                    raise RuntimeError('forecast_modified_destination_objects')
                if not _equal(_read(actor, body)[0], before[0]):
                    raise RuntimeError('forecast_raw_object_copy_mismatch')
            for _, _, actor, body, before in self.rows.values():
                if _read(actor, body)[1] != before[1]:
                    (body.put_to_sleep if before[1] else body.wake_up)()
            # All-object recheck catches cross-body effects too. Never repair
            # velocities/poses after sleep: that would wake the body again.
            for _, _, actor, body, before in self.rows.values():
                now = _read(actor, body)
                if not _equal(now[0], before[0]) or now[1] != before[1]:
                    raise RuntimeError('forecast_sleep_copy_readback_mismatch')
        finally:
            self.assert_source_unchanged()
