"""Causal loaded reference governor and bounded PI drive compensation.

The certified reference never becomes the drive target or measured state.
Forecasts are kinematic, empirical early guards, NOT native safety proofs.
No state setters, gain/force changes, geometry changes or new assistance.
"""
import runtime_profile as timing
import numpy as np
from loaded_clearance import ee
from astra_planner import so3_log
from preview_errors import CandidateInfeasible

POLICY=dict(schema='loaded-tracking/006',dt=.004,horizon_s=.060,
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

def check(ok,reason):
    if not ok:raise CandidateInfeasible('loaded_tracking_'+reason)

class Tracking:
    def __init__(self,admission,nominal,state,controls):
        self.a=admission;self.path=np.asarray([x['q'] for x in nominal[:-1100]],float)
        check(self.path.ndim==2 and self.path.shape[1]==7 and len(self.path)>1 and np.isfinite(self.path).all(),'path')
        arm=admission.arm;q=np.asarray(state['robots'][arm]['q'][:7]);rows=controls[arm][:7]
        check(len(rows)==7 and all(j['name']=='panda_joint'+str(i+1) and j['stiffness']==1000. and j['damping']==200. and j['mode']=='force' for i,j in enumerate(rows)),'unsupported_actuator')
        target=np.array([j['target'][0] for j in rows]);v=np.array([j['velocity'][0] for j in rows])
        check(np.isfinite(target).all() and np.isfinite(v).all() and np.max(np.abs(v))<=POLICY['drive_velocity_rad_s'],'unsupported_handoff_velocity')
        check(np.array_equal(q,self.path[0]),'initial_reference')
        self.bias=target-q;self.initial_bias=self.bias.copy();self.reference=q.copy();self.initial_velocity=v.copy()
        self.cursor=0.;self.index=0;self.stalls=0;self.last_drive=target.copy();self.previous_q=None
        self.bounds=np.asarray(admission.world.model.bounds)+[.03,-.03]
        check(np.max(np.abs(self.bias))<=POLICY['bias_limit_rad'],'initial_bias')
        check(np.all(target>=self.bounds[:,0]) and np.all(target<=self.bounds[:,1]),'initial_drive_bounds')
        self.last_trials=[]
        self.budget=(len(self.path)-1)*POLICY['max_tick_multiplier']+POLICY['max_stall_ticks']+1101

    def reference_at(self,cursor):
        if cursor==len(self.path)-1:return self.path[-1].copy()
        i=min(int(cursor),len(self.path)-2);u=cursor-i
        return self.path[i]+u*(self.path[i+1]-self.path[i])

    @timing.measured('tracking')
    def decide(self,state):
        self.last_trials=[]
        p=POLICY;a=self.a;arm=a.arm;dt=p['dt'];h=p['horizon_s']
        q=np.asarray(state['robots'][arm]['q'][:7]);qd=np.asarray(state['robots'][arm]['qdot'][:7])
        check(q.shape==qd.shape==(7,) and np.isfinite(q).all() and np.isfinite(qd).all(),'nonfinite_state')
        check(self.index<self.budget,'time_budget')
        velocities=[qd] if self.previous_q is None else [qd,(q-self.previous_q)/dt]
        now=ee(a.world.model,arm,q)
        # Measured q itself must pass real FK above. Extrapolations are not states.
        forecasts=[q+h*v for v in velocities]
        if any(not np.isfinite(x).all() or np.any(x<self.bounds[:,0]) or np.any(x>self.bounds[:,1]) for x in forecasts):
            self.last_trials=[dict(factor=None,cursor=self.cursor,
                predicates=dict(measured_forecast_domain=False),
                rejections=['measured_forecast_domain'],admissible=False)]
            raise CandidateInfeasible('loaded_tracking_predictive_guard:'+str(self.last_trials))
        futures=[ee(a.world.model,arm,x) for x in forecasts]
        # Held vertices extrapolated at measured qdot; slip remains separately
        # checked against the original envelope after every native step.
        obj=now@a.relative
        next_objects=[ee(a.world.model,arm,q+dt*v)@a.relative for v in velocities]
        payload_step=max(float(np.linalg.norm(v@nobj[:3,:3].T+nobj[:3,3]-(v@obj[:3,:3].T+obj[:3,3]),axis=1).max()) for v in a.local for nobj in next_objects)
        trials=[];self.last_trials=trials
        for factor in ([0.] if self.index==0 or self.cursor==len(self.path)-1 else p['advance_factors']):
            cursor=min(self.cursor+factor,len(self.path)-1);ref=self.reference_at(cursor)
            velocity=(ref-self.reference)/dt
            reference_ee=ee(a.world.model,arm,ref);old_ee=ee(a.world.model,arm,self.reference)
            forecast=ref+h*velocity
            # Reject this trial, never clip a forecast or alter a certified ref.
            if not np.isfinite(forecast).all() or np.any(forecast<self.bounds[:,0]) or np.any(forecast>self.bounds[:,1]):
                trials.append(dict(factor=factor,cursor=cursor,
                    predicates=dict(reference_forecast_domain=False),
                    rejections=['reference_forecast_domain'],admissible=False))
                continue
            future_ref=ee(a.world.model,arm,forecast)
            pe=max([float(np.linalg.norm(now[:3,3]-reference_ee[:3,3]))]+[float(np.linalg.norm(future[:3,3]-future_ref[:3,3])) for future in futures])+p['position_uncertainty_m']
            re=max([float(np.linalg.norm(so3_log(now[:3,:3]@reference_ee[:3,:3].T)))]+[float(np.linalg.norm(so3_log(future[:3,:3]@future_ref[:3,:3].T))) for future in futures])+p['rotation_uncertainty_rad']
            je=max([float(np.max(np.abs(q-ref)))]+[float(np.max(np.abs(q+h*v-ref-h*velocity))) for v in velocities])+p['joint_uncertainty_rad']
            speed=float(np.linalg.norm(reference_ee[:3,3]-old_ee[:3,3]))/dt
            rspeed=float(np.linalg.norm(so3_log(reference_ee[:3,:3]@old_ee[:3,:3].T)))/dt
            bias=self.bias if self.index==0 else np.clip(self.bias+np.clip(p['integral_per_s']*dt*(self.reference-q),-p['bias_rate_rad_s']*dt,p['bias_rate_rad_s']*dt),-p['bias_limit_rad'],p['bias_limit_rad'])
            requested_bias=bias.copy()
            # Project the INTEGRATOR, not the certified geometric reference.
            # Quantize the drive box inward so native float32 storage cannot
            # cross the existing tightened joint bound. No target reset.
            lo=self.bounds[:,0].astype(np.float32);hi=self.bounds[:,1].astype(np.float32)
            lo=np.where(lo.astype(float)<self.bounds[:,0],np.nextafter(lo,np.float32(np.inf)),lo).astype(float)
            hi=np.where(hi.astype(float)>self.bounds[:,1],np.nextafter(hi,np.float32(-np.inf)),hi).astype(float)
            low=np.maximum.reduce([np.full(7,-p['bias_limit_rad']),self.bias-p['bias_rate_rad_s']*dt,lo-ref,self.last_drive-p['drive_velocity_rad_s']*dt-ref])
            high=np.minimum.reduce([np.full(7,p['bias_limit_rad']),self.bias+p['bias_rate_rad_s']*dt,hi-ref,self.last_drive+p['drive_velocity_rad_s']*dt-ref])
            feasible=bool(np.all(low<=high))
            if self.index and feasible:bias=np.minimum(high,np.maximum(low,bias))
            drive=ref+bias
            # Velocity feedforward describes the physical reference, not bias
            # motion. Feedback is explicit and its saturation is fully logged.
            vdrive=np.clip(velocity+p['feedback_per_s']*(ref-q)+self.initial_velocity*max(0.,1.-self.index/p['handoff_velocity_fade_ticks']),-p['drive_velocity_rad_s'],p['drive_velocity_rad_s'])
            slew=np.abs(drive-self.last_drive)/dt
            predicates=dict(position=bool(pe<=p['forecast_position_m']),rotation=bool(re<=p['forecast_rotation_rad']),
                joint=bool(je<=p['forecast_joint_rad']),cartesian_speed=bool(speed<=p['cartesian_speed_m_s']+1e-12),
                rotation_speed=bool(rspeed<=p['rotation_speed_rad_s']+1e-12),payload_step=bool(payload_step<=p['forecast_payload_step_m']),
                bias_feasible=feasible,drive_slew=bool(np.max(slew)<=p['drive_velocity_rad_s']+1e-8),
                drive_lower=bool(np.all(drive>=self.bounds[:,0])),drive_upper=bool(np.all(drive<=self.bounds[:,1])),
                bias_rate=bool(np.max(np.abs(bias-self.bias))<=p['bias_rate_rad_s']*dt+1e-12))
            valid=all(predicates.values())
            trials.append(dict(factor=factor,cursor=cursor,position_m=pe,rotation_rad=re,joint_rad=je,cartesian_speed_m_s=speed,rotation_speed_rad_s=rspeed,payload_step_m=payload_step,
                requested_bias=requested_bias.tolist(),bias=bias.tolist(),bias_lower=low.tolist(),bias_upper=high.tolist(),drive=drive.tolist(),drive_slew_rad_s=slew.tolist(),
                drive_lower_slack_rad=(drive-self.bounds[:,0]).tolist(),drive_upper_slack_rad=(self.bounds[:,1]-drive).tolist(),
                predicates=predicates,rejections=[k for k,v in predicates.items() if not v],admissible=valid))
            if valid:
                stalls=self.stalls+1 if cursor==self.cursor and 0<self.index and cursor<len(self.path)-1 else 0
                check(stalls<=p['max_stall_ticks'],'stall_budget')
                return dict(observed_q=q.tolist(),predictor_velocities=[v.tolist() for v in velocities],index=self.index,cursor=cursor,reference_q=ref.tolist(),reference_v=velocity.tolist(),bias=bias.tolist(),q=drive.tolist(),v=vdrive.tolist(),stalls=stalls,trials=trials)
        raise CandidateInfeasible('loaded_tracking_predictive_guard:'+str(trials))

    def commit(self,decision):
        self.previous_q=np.asarray(decision['observed_q'])
        self.cursor=decision['cursor'];self.reference=np.asarray(decision['reference_q']);self.bias=np.asarray(decision['bias']);self.last_drive=np.asarray(decision['q']);self.stalls=decision['stalls'];self.index+=1

    @property
    def at_end(self):return self.cursor==len(self.path)-1
