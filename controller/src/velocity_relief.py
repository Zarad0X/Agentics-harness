"""Experimental capture feedback: reduce preload from observed inward speed.

No force/velocity target writes, state resets or extra ticks. The existing
Capture.step applies float32 slew, geometry, native readback and guarded tick.
One DT of excess inward speed reduces at most the existing 0.25 mm preload.
This is a physical hypothesis, not an inverse dynamics or convergence claim.
"""
POLICY = dict(name='velocity-preload-relief/1', horizon_s=.004,
              max_preload_reduction_m=.00025, speed_trigger_m_s=.01,
              contact_required=True)

def goals(row, previous, position_goals):
    result=list(position_goals)
    for i in range(2):
        if row['touching'][i] and row['finger_velocity'][i]<-.01:
            reduction=min(.00025,.004*(-row['finger_velocity'][i]-.01))
            result[i]=max(result[i],previous[i],max(0.,row['finger_position'][i]-(.00025-reduction)))
    return result
