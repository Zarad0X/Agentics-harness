"""Experimental proactive held-slip supervisor; never resets an admission.

Braking prediction is an empirical budget, not identified native dynamics.
Only requests existing governor factors; every original predicate still applies.
One bounded hold, then slow continuation or explicit failure. No native setters.
"""
import math
import numpy as np
from loaded_common import digest
from astra_planner import so3_log
from preview_errors import CandidateInfeasible

POLICY=dict(schema='held-slip-supervisor/002-experimental',dt=.004,
    reaction_s=.060,assumed_joint_deceleration_rad_s2=.4,
    rotation_uncertainty_rad=.00005,position_uncertainty_m=.000005,
    rotation_rate_uncertainty_rad_s=.001,position_rate_uncertainty_m_s=.0001,
    rate_window_ticks=16,warning_reserve_multiple=2.,max_brake_s=1.,
    hold_max_ticks=250,settle_window_ticks=15,settled_joint_speed_rad_s=.01,
    slow_factors=[.25,.125,0.],max_interventions=1,
    physical_effect='unvalidated; retain hard envelopes and all native checks')

def need(ok,reason):
    if not ok:raise CandidateInfeasible('loaded_slip_'+reason)

def observation(index,state,native,metrics,arm,target):
    """Packet from existing post-step Monitor; pre-step caller rechecks hashes."""
    try:
        qd=np.asarray(state['robots'][arm]['qdot'][:7],float)
        values=np.array([metrics['values']['slip_rotation_rad'],metrics['values']['slip_position_m']])
        rel=np.asarray(metrics['relative'],float);native=np.asarray(native,float)
        need(qd.shape==(7,) and rel.shape==native.shape==(4,4) and
             all(np.isfinite(x).all() for x in (qd,values,rel,native)),'nonfinite_measurement')
        return dict(index=index,arm=arm,target=int(target),state_sha256=digest(state),native_sha256=digest(native.tolist()),
                    rotation_rad=float(values[0]),position_m=float(values[1]),relative=rel.tolist(),joint_speed=float(np.max(np.abs(qd))))
    except (KeyError,TypeError,ValueError) as exc:
        raise CandidateInfeasible('loaded_slip_invalid_measurement') from exc

class Supervisor:
    def __init__(self,arm,target,envelope):
        self.arm=arm;self.target=int(target);self.limits=np.array([envelope.slip_rotation_rad,envelope.slip_position_m])
        self.last=None;self.rates=[];self.poses=[];self.mode='normal';self.hold_start=None
        self.hold_brake_ticks=None;self.last_witness=None

    def decide(self,packet,state_sha256,native_sha256,index,previous_reference_v):
        self.last_witness=dict(schema=POLICY['schema'],mode=self.mode,index=index)
        p=POLICY
        need(type(index) is int and index>=0 and type(packet.get('index')) is int and packet['index']==index,'stale_index')
        need(packet.get('arm')==self.arm and packet.get('target')==self.target,'identity')
        need(packet.get('state_sha256')==state_sha256 and packet.get('native_sha256')==native_sha256,'stale_measurement')
        need(index==(0 if self.last is None else self.last['index']+1),'nonconsecutive_measurement')
        try:
            values=np.array([packet['rotation_rad'],packet['position_m']],float)
            rel=np.array(packet['relative'],float);refv=np.asarray(previous_reference_v,float)
            speed=float(packet['joint_speed'])
        except (KeyError,TypeError,ValueError) as exc:raise CandidateInfeasible('loaded_slip_invalid_measurement') from exc
        need(values.shape==(2,) and rel.shape==(4,4) and refv.shape==(7,) and
             all(np.isfinite(x).all() for x in (values,rel,refv)) and math.isfinite(speed),'nonfinite_measurement')
        need(np.all(values>=0) and speed>=0,'invalid_measurement')
        need(np.allclose(rel[3],[0,0,0,1],atol=1e-10,rtol=0) and
             np.max(np.abs(rel[:3,:3].T@rel[:3,:3]-np.eye(3)))<1e-6 and abs(np.linalg.det(rel[:3,:3])-1)<1e-6,'invalid_relative_pose')
        need(np.all(values<=self.limits),'hard_bound')
        if self.last is not None:
            old=np.array([self.last['rotation_rad'],self.last['position_m']])
            self.rates.append(np.maximum((values-old)/p['dt'],0.))
            self.rates=self.rates[-p['rate_window_ticks']:]
        upper=np.max(self.rates,axis=0) if self.rates else np.zeros(2)
        upper+=np.array([p['rotation_rate_uncertainty_rad_s'],p['position_rate_uncertainty_m_s']])
        horizon=p['reaction_s']+max(speed,float(np.max(np.abs(refv))))/p['assumed_joint_deceleration_rad_s2']
        reserve=np.array([p['rotation_uncertainty_rad'],p['position_uncertainty_m']])+upper*horizon
        remaining=self.limits-values;warning=bool(np.any(remaining<=p['warning_reserve_multiple']*reserve))
        self.last_witness.update(measurement=packet,headroom=remaining.tolist(),rate_upper=upper.tolist(),
                                 brake_horizon_s=horizon,reserve=reserve.tolist(),warning=warning)
        need(horizon<=p['max_brake_s'],'unsafe_braking_horizon')
        need(np.all(remaining>reserve),'insufficient_braking_headroom')
        self.poses.append(rel.copy());self.poses=self.poses[-(p['settle_window_ticks']+1):]
        if self.mode=='normal' and warning:
            self.mode='hold';self.hold_start=index;self.hold_brake_ticks=math.ceil(horizon/p['dt'])
        elif self.mode=='slow' and warning:
            need(False,'recurrent_risk_requires_fresh_plan')
        if self.mode=='hold':
            elapsed=index-self.hold_start
            need(elapsed<p['hold_max_ticks'],'hold_timeout_requires_fresh_plan')
            if elapsed>=self.hold_brake_ticks+p['settle_window_ticks']:
                dr=max(float(np.linalg.norm(so3_log(rel[:3,:3]@f[:3,:3].T))) for f in self.poses)
                dp=max(float(np.linalg.norm(rel[:3,3]-f[:3,3])) for f in self.poses)
                quiet=(dr<=p['rotation_uncertainty_rad'] and dp<=p['position_uncertainty_m'])
                self.last_witness.update(hold_window_rotation_rad=dr,hold_window_position_m=dp)
                need(quiet,'hold_drift_not_arrested')
                if speed<=p['settled_joint_speed_rad_s']:self.mode='slow'
        factors=None if self.mode=='normal' else ([0.] if self.mode=='hold' else p['slow_factors'].copy())
        self.last=dict(packet);self.last_witness.update(mode=self.mode,hold_start=self.hold_start,
            hold_brake_ticks=self.hold_brake_ticks,advance_factors=factors)
        return factors,dict(self.last_witness)
