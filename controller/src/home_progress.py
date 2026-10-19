"""Opt-in measured HOME reference governor; no plant/deceleration assumption.

The certified joint-linear polyline is unchanged. Each segment uses a scalar
joint-infinity-distance clock with bounded position-FD velocity/acceleration.
At every completed scene step, choose progress using actual corresponding mesh
vertices, native AND position-difference velocity, and a command-stop envelope.
The latter bounds the entire *reference* braking arc relative to the current
measurement, plus the existing SPEED_BOX one-step reserve. It is not a bound
on a physical stopping trajectory. Old hard guards remain authoritative.
"""
from collections import deque
import hashlib
import math
import numpy as np
from provenance import canonical,digest
from preview_errors import CandidateInfeasible
from return_clearance import DT,LINK_TUBE_M,SPEED_BOX,TRACK_BOX
from vertex_governor import readiness,FiniteDifference

POLICY=dict(schema='home-progress/21',speed_rad_s=.06,acceleration_rad_s2=.4,
    full_speed_tracking_m=.001,stop_progress_tracking_m=.002,numerical_m=1e-7,
    bisections=18,motion_commands=30000,pause_ticks=500,total_hold_ticks=5000,
    terminal_ticks=1000,total_execution_ticks=36000,physical_braking_proved=False)
BOUND_POLICY={k:v for k,v in POLICY.items() if k not in ('full_speed_tracking_m','stop_progress_tracking_m')}
BOUND_POLICY.update(schema='home-progress/22',selection='maximal bounded reference brake; no soft tracking throttle')


def stopping_distance(v,dt):
    """Distance including next command, then maximal bounded reference braking."""
    if not np.isfinite(v) or v<0 or not np.isfinite(dt) or dt<=0:raise ValueError('progress_stop_input')
    dv=POLICY['acceleration_rad_s2']*dt
    n=int(math.ceil(v/dv))
    return dt*(n*v-dv*n*(n-1)/2)


def directional_bounds(c,q,direction):
    """Per-link first derivative and half second-derivative bounds.

    The directional vertex derivative is the sum of revolute-axis cross
    products, retaining cancellation between coupled joints. For the complete
    interval, ||x''|| <= 2*sum(abs(direction))*sum(R_j*abs(direction_j)).
    R_j are existing global all-configuration radii, so integrating gives
    displacement <= linear*s + quadratic*s*s, without a future FK query.
    """
    q=np.asarray(q,float);direction=np.asarray(direction,float)
    robot=c.world.robot_shapes(c.arm,q.tolist(),c.fingers.tolist())
    transforms=c.world.model.link_transforms(c.arm,q.tolist(),c.fingers.tolist())
    axes=[]
    for joint in c.world.model.chain[:7]:
        frame=np.asarray(transforms[joint.find('child').get('link')])
        axis=frame[:3,:3]@np.array(list(map(float,joint.find('axis').get('xyz').split())))
        axes.append((frame[:3,3],axis/np.linalg.norm(axis)))
    result={};angle=float(np.abs(direction).sum())
    for name,shape in robot:
        coeff=c.bounds.coefficients[name];derivative=np.zeros_like(shape[1])
        for j,(center,axis) in enumerate(axes):
            if coeff[j]>0:derivative+=direction[j]*np.cross(axis,shape[1]-center)
        result[name]=(float(np.max(np.linalg.norm(derivative,axis=1)))+1e-9,
                      angle*float(coeff[:7]@np.abs(direction))+1e-9)
    return result


def arc_bounds(c,actual,start,end):
    """Independent test/replay entry for the same full-interval runtime bound."""
    start=np.asarray(start,float);end=np.asarray(end,float);delta=end-start;s=float(np.max(np.abs(delta)))
    measured=readiness(c,start,actual,None)['link_displacement_m']
    if not measured:raise CandidateInfeasible('home_progress_geometry')
    curve=directional_bounds(c,start,delta/s if s else np.zeros(7))
    return {n:measured[n]+linear*s+quadratic*s*s+POLICY['numerical_m']
            for n,(linear,quadratic) in curve.items()}


class ProgressGovernor:
    def __init__(self,api,arm,target,path,certifier,nominal_vertices):
        self.api=api;self.arm=arm;self.target=target;self.path=np.asarray(path,float);self.c=certifier
        self.bound_driven=getattr(api,'home_progress_bound',False)
        if type(self.bound_driven) is not bool:raise ValueError('home_progress_bound_bool')
        self.policy=BOUND_POLICY if self.bound_driven else POLICY
        self.dt=api.telemetry()['scene_timestep_s']
        if self.dt!=certifier.dt or abs(self.dt-DT)>1e-9 or not certifier.contact_envelope:
            raise CandidateInfeasible('home_progress_live_clock_model')
        if self.path.ndim!=2 or self.path.shape[1]!=7 or len(self.path)<1 or not np.isfinite(self.path).all():
            raise CandidateInfeasible('home_progress_path')
        self.nominal_vertices=list(nominal_vertices);self.segment=0;self.position=0.;self.speed=0.
        self.q=self.path[0].copy();self.v=np.zeros(7);self.initial=True;self.done=False
        self.pending_vertex=None;self.commands=0;self.pause=0;self.paused=0;self.fd=FiniteDifference()
        self.last_step=None;self.hash=hashlib.sha256();self.rows=deque(maxlen=64);self.samples=[];self.count=0
        self.peak_velocity=0.;self.peak_acceleration=0.;self.first_regulation=None
        api.event('home_progress_admitted',motion_phase='home',arm=arm,object_id=target,policy=self.policy,
            path_sha256=digest(path),model_sha256=certifier.contact_envelope.sha256,actual_dt_s=self.dt,
            original_vertex_indices=self.nominal_vertices,formal_credit=False)

    def checkpoint(self):
        value=self.pending_vertex;self.pending_vertex=None
        return None if value is None else self.nominal_vertices[value]+1

    def resumed(self):
        # The existing vertex hold ends with actual zero targets. Every progress
        # vertex is already at zero nominal velocity; no mid-edge rebase.
        if np.max(np.abs(self.v))>1e-12:raise CandidateInfeasible('home_progress_nonzero_vertex')
        self.fd=FiniteDifference();self.last_step=None

    def choose(self,state,readback,vertex_hold_ticks=0):
        if not self.done and self.commands>=POLICY['motion_commands']:
            raise CandidateInfeasible('home_progress_motion_cap')
        step=readback['scene_step_count'];dt=readback['scene_timestep_s']
        if dt!=self.dt or type(step) is not int or (self.last_step is not None and step!=self.last_step+1):
            raise CandidateInfeasible('home_progress_scene_step')
        self.last_step=step
        actual=np.asarray(state['robots'][self.arm]['q'],float)
        native=np.asarray(state['robots'][self.arm]['qdot'],float)
        fd=self.fd.observe(actual[:7],step,dt)
        if actual.shape!=(9,) or native.shape!=(9,) or not np.isfinite(actual).all() or not np.isfinite(native).all():
            raise CandidateInfeasible('home_progress_invalid_measurement')
        if fd is not None and np.max(np.abs(fd))>.15:raise CandidateInfeasible('home_progress_measured_fd_speed')
        if np.any(np.abs(native)>SPEED_BOX+1e-12):raise CandidateInfeasible('home_progress_native_speed')
        measured=readiness(self.c,self.q,actual,fd)
        error=measured['whole_link_m']
        if error is None:raise CandidateInfeasible('home_progress_invalid_geometry')
        shapes=self.c.world.robot_shapes(self.arm,actual[:7].tolist(),np.clip(actual[-2:],0,.04).tolist())
        steps=self.c.step_bounds(self.arm,actual[:7].tolist(),np.clip(actual[-2:],0,.04).tolist(),shapes)
        if any(measured['link_displacement_m'][n]+POLICY['numerical_m']+steps[n]>LINK_TUBE_M for n in steps):
            raise CandidateInfeasible('home_progress_no_bounded_reference_brake')
        oldq=self.q.copy();oldv=self.v.copy();brake=None;regulated=False
        if self.initial:
            self.initial=False;self.pending_vertex=0
        elif not self.done:
            if self.segment>=len(self.path)-1:self.done=True
            else:
                delta=self.path[self.segment+1]-self.path[self.segment];length=float(np.max(np.abs(delta)))
                remaining=max(0.,length-self.position)
                if remaining<=1e-13:
                    # Explicit zero nominal velocity at every original vertex;
                    # an extra retained-target command, never a position snap.
                    self.v=np.zeros(7);self.speed=0.;self.position=0.;self.segment+=1
                    self.q=self.path[self.segment].copy();self.pending_vertex=self.segment
                    if self.segment==len(self.path)-1:self.done=True
                else:
                    direction=delta/length;dv=POLICY['acceleration_rad_s2']*dt
                    lower=max(0.,self.speed-dv)
                    # v21's 1--2 mm heuristic stranded the actual clock near
                    # static joint-2 error. v22 selects the largest admissible
                    # speed under the SAME full braking arc, one-step, joint,
                    # corner and slew bounds below. No plant braking claim.
                    requested=POLICY['speed_rad_s'] if self.bound_driven else POLICY['speed_rad_s']*float(np.clip((POLICY['stop_progress_tracking_m']-error)/
                        (POLICY['stop_progress_tracking_m']-POLICY['full_speed_tracking_m']),0.,1.))
                    upper=min(POLICY['speed_rad_s'],self.speed+dv,max(lower,requested),remaining/dt)
                    curve=directional_bounds(self.c,self.q,direction)
                    def check(w):
                        distance=stopping_distance(w,dt)
                        if distance>remaining+1e-13:return None
                        distance=min(distance,remaining)
                        endpoint=self.q+direction*distance
                        bounds={n:measured['link_displacement_m'][n]+linear*distance+quadratic*distance*distance+POLICY['numerical_m']
                                for n,(linear,quadratic) in curve.items()}
                        joint=np.maximum(np.abs(np.r_[self.q,self.c.fingers]-actual),
                                         np.abs(np.r_[endpoint,self.c.fingers]-actual))+SPEED_BOX*dt
                        if np.any(joint>TRACK_BOX) or any(bounds[n]+steps[n]>LINK_TUBE_M for n in bounds):return None
                        return dict(reference_stop_distance_rad=distance,reference_stop_q=endpoint.tolist(),
                            link_arc_bound_m=bounds,one_step_reserve_m=steps,
                            tracking_plus_reference_brake_plus_step_m=max(bounds[n]+steps[n] for n in bounds),
                            joint_stop_plus_step_bound=joint.tolist())
                    brake=check(lower)
                    if brake is None:raise CandidateInfeasible('home_progress_no_bounded_reference_brake')
                    top=check(upper)
                    if top is not None:w=upper;brake=top
                    else:
                        lo,hi=lower,upper
                        for _ in range(POLICY['bisections']):
                            mid=(lo+hi)/2;candidate=check(mid)
                            if candidate is None:hi=mid
                            else:lo=mid;brake=candidate
                        w=lo
                    distance=w*dt
                    # Round-off-only endpoint landing within 1e-13 rad. This is
                    # reference arithmetic, never a robot/object state setter.
                    if remaining-distance<=1e-13:distance=remaining
                    self.position+=distance;self.q=self.path[self.segment]+direction*self.position
                    self.v=(self.q-oldq)/dt;self.speed=float(np.max(np.abs(self.v)))
                    regulated=w<upper-1e-10 or requested<POLICY['speed_rad_s']
                    pause=self.pause+1 if distance<=1e-14 else 0
                    paused=self.paused+int(distance<=1e-14)
                    if pause>POLICY['pause_ticks'] or paused+vertex_hold_ticks>POLICY['total_hold_ticks']:
                        raise CandidateInfeasible('home_progress_shared_hold_cap')
                    self.pause=pause;self.paused=paused
        peak=float(np.max(np.abs(self.v)));acc=float(np.max(np.abs(self.v-oldv))/dt)
        if peak>.06+1e-12 or acc>.4+1e-8:raise CandidateInfeasible('home_progress_command_derivative')
        if self.pause>POLICY['pause_ticks'] or self.paused+vertex_hold_ticks>POLICY['total_hold_ticks']:
            raise CandidateInfeasible('home_progress_shared_hold_cap')
        self.peak_velocity=max(self.peak_velocity,peak);self.peak_acceleration=max(self.peak_acceleration,acc)
        row=dict(scene_step=step,actual_dt_s=dt,command_index=self.commands,segment=self.segment,
            segment_position_rad=self.position,reference_q=self.q.tolist(),nominal_qdot=self.v.tolist(),
            measured_q=actual.tolist(),native_qdot=native.tolist(),measured_fd_qdot=fd,
            measured_tracking_m=error,regulated=regulated,reference_brake=brake,
            paused_ticks=self.paused,vertex_hold_ticks=vertex_hold_ticks,done=self.done)
        if regulated and self.first_regulation is None:self.first_regulation=row.copy()
        self.hash.update(canonical(row));self.count+=1;self.rows.append(row)
        if self.count%100==1 and len(self.samples)<300:self.samples.append(row)
        self.commands+=1
        return self.q.copy(),self.v.copy(),row

    def finish(self,failed):
        self.api.event('home_progress_receipt',motion_phase='home',arm=self.arm,object_id=self.target,
            policy=self.policy,path_sha256=digest(self.path.tolist()),model_sha256=self.c.contact_envelope.sha256,
            actual_dt_s=self.dt,failed=failed,reference_complete=self.done,decisions=self.commands,
            packet_count=self.count,all_packets_sha256=self.hash.hexdigest(),samples=self.samples,
            tail=list(self.rows),first_regulation=self.first_regulation,paused_ticks=self.paused,
            max_nominal_velocity_rad_s=self.peak_velocity,max_nominal_acceleration_rad_s2=self.peak_acceleration,
            formal_credit=False,physical_braking_proved=False)
