"""Independent controller replay from preceding native states and readbacks.

Does not import Tracking, POLICY, commands or the runtime command generator.
Numeric witnesses are recomputed; hashes alone do not establish semantics.
"""
import numpy as np
from loaded_clearance import ee
from astra_planner import so3_log

CONTRACT=dict(schema='loaded-tracking/006',dt=.004,horizon_s=.060,
    cartesian_speed_m_s=.040,rotation_speed_rad_s=.12,
    forecast_position_m=.004,forecast_rotation_rad=.014,forecast_joint_rad=.010,
    position_uncertainty_m=.0006,rotation_uncertainty_rad=.001,joint_uncertainty_rad=.001,
    forecast_payload_step_m=.0015,feedback_per_s=5.,integral_per_s=2.,
    bias_limit_rad=.015,bias_rate_rad_s=.010,drive_velocity_rad_s=.25,handoff_velocity_fade_ticks=15,
    advance_factors=[1.,.5,.25,.125,0.],max_stall_ticks=1000,max_tick_multiplier=8,
    reference_semantics='fixed_certified_path_fractional_clock_no_rebase',
    drive_semantics='reference_plus_retained_projected_integral_bias',
    antiwindup='project_bias_to_joint_readback_slew_and_bias_rate_intersection',
    velocity_semantics='reference_backward_difference_plus_position_feedback',
    safety_claim='empirical_prediction_only_actual_original_envelope_mandatory')

def require(ok,label):
    if not ok:raise ValueError('tracking_audit_'+label)

def equivalent(a,b):
    if isinstance(a,dict):return isinstance(b,dict) and set(a)==set(b) and all(equivalent(a[k],b[k]) for k in a)
    if isinstance(a,list):return isinstance(b,list) and len(a)==len(b) and all(equivalent(x,y) for x,y in zip(a,b))
    if type(a) in (int,float) and type(b) in (int,float):return np.isfinite(a) and np.isfinite(b) and abs(a-b)<=1e-11
    return type(a)==type(b) and a==b

class Replay:
    def __init__(self,a,commands,state,controls,profile):
        require(profile['tracking']==CONTRACT and profile['handoff']=='retained_drive_bias_with_fixed_measured_reference','contract')
        self.a=a;self.points=np.array([c['q'] for c in commands[:-1100]]);self.at=0.;self.count=0;self.stalls=0
        self.ref=self.points[0].copy();joints=controls[a.arm][:7]
        require(len(joints)==7 and all(j['name']==f'panda_joint{i+1}' and j['stiffness']==1000 and j['damping']==200 and j['mode']=='force' for i,j in enumerate(joints)),'unsupported_actuator')
        self.drive=np.array([j['target'][0] for j in joints]);self.bias=self.drive-self.ref
        require(np.isfinite(self.bias).all() and np.max(np.abs(self.bias))<=.015 and all(np.isfinite(j['velocity'][0]) and abs(j['velocity'][0])<=.25 for j in joints),'handoff')
        self.initial_v=np.array([j['velocity'][0] for j in joints])
        self.limits=np.array(a.world.model.bounds)+[.03,-.03]
        require(np.all(self.drive>=self.limits[:,0]) and np.all(self.drive<=self.limits[:,1]),'drive_limits')
        self.budget=8*(len(self.points)-1)+2101;self.arrival=None;self.previous_q=None;self.last_trials=[]

    def next(self,state,advance_factors=None):
        self.last_trials=[]
        a=self.a;q=np.array(state['robots'][a.arm]['q'][:7]);v=np.array(state['robots'][a.arm]['qdot'][:7])
        require(np.isfinite(q).all() and np.isfinite(v).all() and self.count<self.budget,'state_budget')
        velocities=[v] if self.previous_q is None else [v,(q-self.previous_q)/.004]
        now=ee(a.world.model,a.arm,q)
        forecast_vectors=[q+.060*x for x in velocities]
        # Independent scalar reconstruction of the existing FK margin domain.
        if any(any(not np.isfinite(value) or not lo+.03<=value<=hi-.03
                   for value,(lo,hi) in zip(vector,a.world.model.bounds)) for vector in forecast_vectors):
            self.last_trials=[dict(factor=None,cursor=self.at,
                predicates=dict(measured_forecast_domain=False),
                rejections=['measured_forecast_domain'],admissible=False)]
            raise ValueError('tracking_audit_predictive_guard')
        future=[ee(a.world.model,a.arm,x) for x in forecast_vectors]
        transform=now@a.relative;next_transforms=[ee(a.world.model,a.arm,q+.004*x)@a.relative for x in velocities]
        step=max(np.linalg.norm((verts@t[:3,:3].T+t[:3,3])-(verts@transform[:3,:3].T+transform[:3,3]),axis=1).max() for verts in a.local for t in next_transforms)
        factors=[0.] if self.count==0 or self.at==len(self.points)-1 else ([1.,.5,.25,.125,0.] if advance_factors is None else advance_factors)
        trials=[];self.last_trials=trials
        for advance in factors:
            new_at=min(self.at+advance,len(self.points)-1);i=min(int(new_at),len(self.points)-2)
            ref=self.points[-1].copy() if new_at==len(self.points)-1 else self.points[i]+(new_at-i)*(self.points[i+1]-self.points[i]);rv=(ref-self.ref)/.004
            rf=ee(a.world.model,a.arm,ref);old=ee(a.world.model,a.arm,self.ref)
            vector=ref+.060*rv
            if any(not np.isfinite(value) or not lo+.03<=value<=hi-.03
                   for value,(lo,hi) in zip(vector,a.world.model.bounds)):
                trials.append(dict(factor=advance,cursor=new_at,
                    predicates=dict(reference_forecast_domain=False),
                    rejections=['reference_forecast_domain'],admissible=False))
                continue
            ff=ee(a.world.model,a.arm,vector)
            pe=max([np.linalg.norm(now[:3,3]-rf[:3,3])]+[np.linalg.norm(f[:3,3]-ff[:3,3]) for f in future])+.0006
            re=max([np.linalg.norm(so3_log(now[:3,:3]@rf[:3,:3].T))]+[np.linalg.norm(so3_log(f[:3,:3]@ff[:3,:3].T)) for f in future])+.001
            je=max([np.max(np.abs(q-ref))]+[np.max(np.abs(q+.060*x-ref-.060*rv)) for x in velocities])+.001
            speed=np.linalg.norm(rf[:3,3]-old[:3,3])/.004;spin=np.linalg.norm(so3_log(rf[:3,:3]@old[:3,:3].T))/.004
            bias=self.bias.copy()
            if self.count:
                for k in range(7):bias[k]=min(.015,max(-.015,bias[k]+min(.00004,max(-.00004,.008*(self.ref[k]-q[k])))))
            requested=bias.copy();low=[];high=[]
            # Scalar reconstruction, independent of runtime projection helper.
            for k in range(7):
                left=np.float32(self.limits[k,0]);right=np.float32(self.limits[k,1])
                if float(left)<self.limits[k,0]:left=np.nextafter(left,np.float32(np.inf))
                if float(right)>self.limits[k,1]:right=np.nextafter(right,np.float32(-np.inf))
                low.append(max(-.015,self.bias[k]-.00004,float(left)-ref[k],self.drive[k]-.001-ref[k]))
                high.append(min(.015,self.bias[k]+.00004,float(right)-ref[k],self.drive[k]+.001-ref[k]))
            feasible=all(l<=r for l,r in zip(low,high))
            if self.count and feasible:
                bias=np.array([min(high[k],max(low[k],bias[k])) for k in range(7)])
            drive=ref+bias;velocity=np.array([min(.25,max(-.25,rv[k]+5*(ref[k]-q[k])+self.initial_v[k]*max(0.,1-self.count/15))) for k in range(7)])
            slew=np.abs(drive-self.drive)/.004
            predicates=dict(position=bool(pe<=.004),rotation=bool(re<=.014),joint=bool(je<=.010),
                cartesian_speed=bool(speed<=.040+1e-12),rotation_speed=bool(spin<=.12+1e-12),payload_step=bool(step<=.0015),
                bias_feasible=feasible,drive_slew=bool(max(slew)<=.25+1e-8),drive_lower=bool(np.all(drive>=self.limits[:,0])),
                drive_upper=bool(np.all(drive<=self.limits[:,1])),bias_rate=bool(max(abs(bias-self.bias))<=.00004+1e-12))
            valid=all(predicates.values())
            trials.append(dict(factor=advance,cursor=new_at,position_m=float(pe),rotation_rad=float(re),joint_rad=float(je),cartesian_speed_m_s=float(speed),rotation_speed_rad_s=float(spin),payload_step_m=float(step),
                requested_bias=requested.tolist(),bias=bias.tolist(),bias_lower=list(map(float,low)),bias_upper=list(map(float,high)),drive=drive.tolist(),drive_slew_rad_s=slew.tolist(),
                drive_lower_slack_rad=(drive-self.limits[:,0]).tolist(),drive_upper_slack_rad=(self.limits[:,1]-drive).tolist(),
                predicates=predicates,rejections=[k for k,v in predicates.items() if not v],admissible=valid))
            if valid:
                stalls=self.stalls+1 if new_at==self.at and self.count>0 and new_at<len(self.points)-1 else 0
                require(stalls<=1000,'stall_budget')
                return dict(observed_q=q.tolist(),predictor_velocities=[x.tolist() for x in velocities],index=self.count,cursor=new_at,reference_q=ref.tolist(),reference_v=rv.tolist(),bias=bias.tolist(),q=drive.tolist(),v=velocity.tolist(),stalls=stalls,trials=trials)
        raise ValueError('tracking_audit_predictive_guard')

    def accept(self,d):
        self.previous_q=np.array(d['observed_q'])
        self.at=d['cursor'];self.ref=np.array(d['reference_q']);self.drive=np.array(d['q']);self.bias=np.array(d['bias']);self.stalls=d['stalls'];self.count+=1
        if self.at==len(self.points)-1 and self.arrival is None:self.arrival=self.count
