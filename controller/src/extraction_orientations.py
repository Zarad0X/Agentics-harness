"""Append missing gripper orientations to extraction hypotheses only.

Parallel-jaw contact geometry does not imply robot IK or collision symmetry.
Every added hypothesis still needs the caller's full unchanged admission.
This is not an object-goal quaternion equivalence or a physical retry.
"""
import copy
from astra_planner import _rotation


def append_halfturns(candidates):
    original = list(candidates)
    variants = []
    for index, candidate in enumerate(original):
        rotation = _rotation(candidate['ee_rotation_world'])
        rotation[:, 1:] *= -1
        variant = copy.deepcopy(candidate)
        variant['ee_rotation_world'] = rotation.tolist()
        variant['orientation_variant'] = 'approach_axis_halfturn'
        variant['orientation_parent_index'] = index
        variants.append(variant)
    return original + variants
