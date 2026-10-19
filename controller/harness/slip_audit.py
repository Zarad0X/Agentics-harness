"""Independent slip-policy state transition plus existing independent PI replay.
No imports from slip_supervisor or runtime Tracking. No physics commands.
"""
import math
import numpy as np
from loaded_common import digest
from collision_world import pose_matrix
from loaded_tracking_audit import Replay as Base

CONTRACT={'schema':'held-slip-supervisor/002-experimental','dt':.004,'reaction_s':.060,
 'assumed_joint_deceleration_rad_s2':.4,'rotation_uncertainty_rad':.00005,'position_uncertainty_m':.000005,
 'rotation_rate_uncertainty_rad_s':.001,'position_rate_uncertainty_m_s':.0001,'rate_window_ticks':16,
 'warning_reserve_multiple':2.,'max_brake_s':1.,'hold_max_ticks':250,'settle_window_ticks':15,
 'settled_joint_speed_rad_s':.01,'slow_factors':[.25,.125,0.],'max_interventions':1,
 'physical_effect':'unvalidated; retain hard envelopes and all native checks'}

def angle(r):
    return math.atan2(float(np.linalg.norm([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]]))/2,float(np.clip((np.trace(r)-1)/2,-1,1)))

class Replay(Base):
    def __init__(self,a,commands,state,controls,profile,native):
        super().__init__(a,commands,state,controls,profile)
        if profile.get('slip_supervisor')!=CONTRACT:raise ValueError('tracking_audit_slip_policy')
        self.stage='normal';self.history=[];self.frames=[];self.prior=None;self.start_hold=None;self.brake_count=None
        self.last_slip=None;self.ref_velocity=[0.]*7
        self.observe(state,native,0)

    def observe(self,state,native,index):
        n=np.asarray(native);h=np.linalg.inv(n)@pose_matrix(state['objects'][self.a.target]['pose'])
        h0=self.a.relative
        self.packet=dict(index=index,arm=self.a.arm,target=int(self.a.target),state_sha256=digest(state),native_sha256=digest(n.tolist()),
            rotation_rad=angle(h[:3,:3]@h0[:3,:3].T),position_m=float(np.linalg.norm(h[:3,3]-h0[:3,3])),
            relative=h.tolist(),joint_speed=float(max(abs(v) for v in state['robots'][self.a.arm]['qdot'][:7])))

    def next(self,state):
        self.last_trials=[];z=self.packet;self.last_slip=dict(schema=CONTRACT['schema'],mode=self.stage,index=self.count)
        def need(ok,why):
            if not ok:raise ValueError('tracking_audit_slip_'+why)
        need(z['index']==self.count,'stale_index');need(z['state_sha256']==digest(state),'stale_measurement')
        vals=[z['rotation_rad'],z['position_m']];limits=[self.a.env.slip_rotation_rad,self.a.env.slip_position_m]
        need(all(math.isfinite(v) and 0<=v for v in vals+[z['joint_speed']]),'nonfinite_measurement')
        need(all(x<=y for x,y in zip(vals,limits)),'hard_bound')
        if self.prior is not None:
            self.history.append([max(0.,(vals[j]-self.prior[j])/.004) for j in range(2)])
            self.history=self.history[-16:]
        rates=[max([0.]+[r[j] for r in self.history])+e for j,e in enumerate([.001,.0001])]
        h=.060+max(z['joint_speed'],max(map(abs,self.ref_velocity)))/.4
        reserves=[.00005+rates[0]*h,.000005+rates[1]*h];remaining=[limits[j]-vals[j] for j in range(2)]
        warning=any(remaining[j]<=2*reserves[j] for j in range(2))
        self.last_slip.update(measurement=z,headroom=remaining,rate_upper=rates,brake_horizon_s=h,reserve=reserves,warning=warning)
        need(h<=1.,'unsafe_braking_horizon');need(all(remaining[j]>reserves[j] for j in range(2)),'insufficient_braking_headroom')
        rel=np.asarray(z['relative']);self.frames.append(rel.copy());self.frames=self.frames[-16:]
        if self.stage=='normal' and warning:
            self.stage='hold';self.start_hold=self.count;self.brake_count=math.ceil(h/.004)
        elif self.stage=='slow' and warning:need(False,'recurrent_risk_requires_fresh_plan')
        if self.stage=='hold':
            elapsed=self.count-self.start_hold;need(elapsed<250,'hold_timeout_requires_fresh_plan')
            if elapsed>=self.brake_count+15:
                dr=max(angle(rel[:3,:3]@f[:3,:3].T) for f in self.frames)
                dp=max(float(np.linalg.norm(rel[:3,3]-f[:3,3])) for f in self.frames)
                self.last_slip.update(hold_window_rotation_rad=dr,hold_window_position_m=dp)
                need(dr<=.00005 and dp<=.000005,'hold_drift_not_arrested')
                if z['joint_speed']<=.01:self.stage='slow'
        factors=None if self.stage=='normal' else ([0.] if self.stage=='hold' else [.25,.125,0.])
        self.prior=vals;self.last_slip.update(mode=self.stage,hold_start=self.start_hold,hold_brake_ticks=self.brake_count,advance_factors=factors)
        result=super().next(state,advance_factors=factors);result['slip']=dict(self.last_slip);return result

    def accept(self,d):
        super().accept(d);self.ref_velocity=d['reference_v']
