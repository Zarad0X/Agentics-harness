"""Candidate-boundary observations only; never calls a solver twice or a getter.

A checkpoint holds one measured state and geometry hashes, not mesh copies.
At most eight call pairs per considered extraction candidate; path predicates
retain a count and their first rejection only, never per-tick or per-query logs.
Logging failures propagate; solver exceptions are recorded and re-raised.
"""
import math
import numpy as np
from provenance import digest


def snapshot(value):
    """Detached JSON data, including explicit nonfinite diagnostic values."""
    if isinstance(value,np.ndarray):return snapshot(value.tolist())
    if isinstance(value,np.generic):return snapshot(value.item())
    if isinstance(value,float) and not math.isfinite(value):return {'nonfinite_float':repr(value)}
    if isinstance(value,dict):return {str(k):snapshot(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [snapshot(v) for v in value]
    if value is None or isinstance(value,(str,int,float,bool)):return value
    raise TypeError('unsupported_candidate_checkpoint_type:'+type(value).__name__)


class CandidateReplay:
    def __init__(self,event,rng,state,arm,target,index,candidate):
        self.event=event;self.rng=rng;self.geometry=None
        packet=snapshot(dict(state=state,arm=arm,object_id=target,index=index,candidate=candidate,
                             rng_before=rng.bit_generator.state))
        self.key=digest(packet)
        self.identity=dict(checkpoint_sha256=self.key,arm=arm,object_id=target,index=index)
        event('candidate_checkpoint',schema='extraction-candidate-replay/1',**packet,checkpoint_sha256=self.key)

    def world_state(self,state):
        # The caller supplies the existing measured getter's result; no reread.
        packet=snapshot(state)
        self.event('candidate_geometry_state',**self.identity,state=packet,state_sha256=digest(packet))

    def free(self,world,*args):
        answer=world.free(*args)
        self.geometry['checks']+=1
        if not answer and self.geometry['first_rejection'] is None:
            self.geometry['first_rejection']=snapshot(dict(arguments=args,collision=world.last_collision))
        return answer

    def call(self,stage,operation,**inputs):
        self.geometry=dict(checks=0,first_rejection=None)
        self.event('candidate_call_started',**self.identity,stage=stage,inputs=snapshot(inputs),
                   rng_before=snapshot(self.rng.bit_generator.state))
        try:
            result=operation()
        except Exception as exc:
            message=str(exc)
            self.event('candidate_call_finished',**self.identity,stage=stage,outcome='exception',
                       exception_type=type(exc).__name__,message=message[:1000],message_truncated=len(message)>1000,
                       geometry=snapshot(self.geometry),rng_after=snapshot(self.rng.bit_generator.state))
            raise
        success=result['success'] if isinstance(result,dict) else result
        self.event('candidate_call_finished',**self.identity,stage=stage,
                   outcome='accepted' if success else 'rejected',result=snapshot(result),
                   geometry=snapshot(self.geometry),rng_after=snapshot(self.rng.bit_generator.state))
        return result
