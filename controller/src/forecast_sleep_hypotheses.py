"""Explicit approximate forecast initializations; never a source-state repair.

Only separation forecasts may request these modes. This is empirical two-case
robustness, NOT a conservative bound or contact/solver-history equivalence.
The original exact helper remains mandatory for other copy boundaries.
"""
import copy
import numpy as np
from sleep_state_copy import ObjectSleepCopy, _read, _equal

CONTRACT = 'separation-sleep-hypotheses/1'
MODES = ('sleep_zero', 'awake_raw')


def ambiguous(values, sleeping):
    return sleeping and any(np.any(v != 0) for v in values[2:])


def initialization_modes(objects, dynamic_type):
    rows = [_read(o.actor, o.actor.find_component_by_type(dynamic_type))
            for o in objects.values()]
    return MODES if any(ambiguous(*row) for row in rows) else ('exact',)


def plain(row):
    values, sleeping = row
    return dict(zip(('p','q','linear','angular','sleeping'),
                    [*(v.tolist() for v in values), sleeping]))


class ForecastInitialization(ObjectSleepCopy):
    def __init__(self, source, destination, dynamic_type, mode='exact'):
        if mode not in ('exact', *MODES): raise ValueError('forecast_initialization_mode')
        super().__init__(source, destination, dynamic_type)
        self.mode = mode
        self.ambiguous_ids = [k for k, r in self.rows.items() if ambiguous(*r[4])]
        self.expected = {k: copy.deepcopy(r[4]) for k, r in self.rows.items()}
        if mode != 'exact' and not self.ambiguous_ids:
            raise RuntimeError('forecast_hypothesis_without_ambiguity')
        for k in self.ambiguous_ids:
            values, sleeping = self.expected[k]
            if mode == 'sleep_zero':
                self.expected[k] = ((*values[:2], np.zeros_like(values[2]), np.zeros_like(values[3])), True)
            elif mode == 'awake_raw': self.expected[k] = (values, False)
        self.restored = False

    def restore(self):
        if self.mode == 'exact':
            super().restore()
            self.restored = True
            return
        try:
            if self.destination.keys() != self.rows.keys():
                raise RuntimeError('forecast_object_layout_mismatch')
            # Every original raw setter must still transfer the raw values
            # exactly BEFORE any explicit hypothesis initialization.
            for k, (_, _, actor, body, before) in self.rows.items():
                if self.destination[k].actor is not actor:
                    raise RuntimeError('forecast_modified_destination_objects')
                if not _equal(_read(actor, body)[0], before[0]):
                    raise RuntimeError('forecast_raw_object_copy_mismatch')
            for k, (_, _, actor, body, before) in self.rows.items():
                wanted = self.expected[k][1]
                # sleep_zero always calls the API for ambiguous rows, including
                # an already-sleeping destination with nonzero observed values.
                if _read(actor, body)[1] != wanted or (k in self.ambiguous_ids and self.mode == 'sleep_zero'):
                    (body.put_to_sleep if wanted else body.wake_up)()
            for k, (_, _, actor, body, _) in self.rows.items():
                observed = _read(actor, body)
                wanted = self.expected[k]
                if not _equal(observed[0], wanted[0]) or observed[1] != wanted[1]:
                    raise RuntimeError('forecast_hypothesis_readback_mismatch')
            self.restored = True
        finally:
            self.assert_source_unchanged()

    def assert_scene_state(self, source_state, destination_state):
        """Check the entire existing runner state, with only enumerated deltas."""
        if not self.restored: raise RuntimeError('forecast_initialization_not_restored')
        expected = copy.deepcopy(source_state)
        if self.mode == 'sleep_zero':
            for k in self.ambiguous_ids:
                expected['objects'][str(k)]['linear_velocity'] = self.expected[k][0][2].tolist()
                expected['objects'][str(k)]['angular_velocity'] = self.expected[k][0][3].tolist()
        if expected != destination_state: raise RuntimeError('forecast_state_copy_mismatch')
        self.assert_source_unchanged()

    def receipt(self):
        if not self.restored: raise RuntimeError('forecast_initialization_not_restored')
        return dict(contract=CONTRACT, mode=self.mode, approximate=self.mode != 'exact',
                    ambiguous_ids=list(map(str, self.ambiguous_ids)),
                    measured_source={str(k):plain(r[4]) for k,r in self.rows.items()},
                    expected_destination={str(k):plain(v) for k,v in self.expected.items()},
                    observed_destination={str(k):plain(_read(r[2],r[3])) for k,r in self.rows.items()},
                    source_unchanged=True, raw_state_discarded=False,
                    conservative_bound_proved=False, contact_history_equivalent=False)
