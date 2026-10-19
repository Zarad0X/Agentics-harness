"""v16 fixed checkpoint calibration. Exactly 500 guarded ticks, no resume.

Old v11 probe is unchanged. OFF sends the same 500 zero-qd controls; all
feedback starts after the frozen checkpoint, not in reconstruction/prefix.
"""
import hashlib
import numpy as np
from provenance import canonical, digest
from preview_errors import CandidateInfeasible
from fixed_setpoint_diagnostic import DiagnosticHoldCompleted, DiagnosticHoldGuardFailed
from return_clearance import ReturnClearance
from vertex_governor import FiniteDifference,readiness,finite_json
from velocity_feedback import FeedbackAPI,targets,retained


def run_hold(api,arm,target,path,certifier,checkpoint):
    q=np.asarray(path[2],float);v=np.zeros(7);fd=FiniteDifference()
    initial=checkpoint['packet'];fd.observe(initial['measured_q'][:7],initial['readback']['scene_step_count'],.004)
    feedback=FeedbackAPI(api,arm,target,'retreat',getattr(api,'velocity_feedback','off'),False)
    if hasattr(api,'feedback_metadata'):api.event('feedback_metadata',metadata=api.feedback_metadata())
    feedback.begin_recovery();packets=[];hashes=hashlib.sha256();count=0;last=None;first=None
    outcome='failed';raw=None;completed=0;forced_stop_tick=None;initial_step=api.telemetry()['scene_step_count']
    def capture(stage):
        s=api.state();monitor=certifier.monitor(s,q)
        clearance=ReturnClearance(api.world(s),arm,target,True).point(s['robots'][arm]['q'][:7],envelopes=False)
        rb=api.telemetry();actual=s['robots'][arm]['q'];velocity=fd.observe(actual[:7],rb['scene_step_count'],rb['scene_timestep_s'])
        return dict(stage=stage,state=s,readback=rb,drive_readback=api.controls(),measured_q=actual,
            monitor=monitor,clearance=clearance,contacts=api.contacts(),
            readiness=readiness(certifier,q,actual,velocity),finite_difference_velocity=velocity)
    def guard(row):
        if not row['monitor']['safe']:raise CandidateInfeasible('postrelease_precommand_envelope:'+','.join(row['monitor']['reasons']))
        if row['clearance'].get('safe') is not True:raise CandidateInfeasible('postrelease_measured_clearance')
        api.governor_guard(target,True)
        if retained(api.controls(),arm)!=feedback.retained:raise CandidateInfeasible('probe_retained_controls')
    try:
        first=capture('initial');last=first;guard(first)
        for n in range(500):
            pre=capture('precommand');last=pre;guard(pre)
            feedback.observe_ready(pre['readiness']['ready'])
            # Reserve the last 64 of the SAME 500 ticks for a bounded stop if
            # READY never arrived. No gain retry, budget extension or resume.
            if n>=436 and not feedback.recovery.withdrawn:
                feedback.recovery.withdrawn=True;forced_stop_tick=n
            feedback.command(arm,q,v)
            try:feedback.tick(target,False,True)
            finally:
                completed=api.telemetry()['scene_step_count']-initial_step
                last=capture('post_attempt')
            guard(last)
            feedback.observe_ready(last['readiness']['ready'],completed=True)
            numeric=dict(hold_tick=n+1,readiness=last['readiness'],streak=feedback.recovery.streak,
                withdrawn=feedback.recovery.withdrawn,**feedback.window[-1])
            packets.append(numeric);hashes.update(canonical(numeric));count+=1
        outcome='diagnostic_completed'
    except Exception as exc:
        raw=dict(type=type(exc).__name__,message=str(exc))
    finally:
        # No cleanup command on any failure. If the 500 ticks end before zero
        # qualification, this probe is explicitly nonqualified and terminal.
        feedback.finish(include_packets=False)
        api.event('feedback_probe',schema='fixed-setpoint-feedback/16',outcome=outcome,raw_failure=raw,
            setting=feedback.setting,reference_q=q.tolist(),original_checkpoint_index=200,progress=2.,
            initial_packet=finite_json(first),final_or_failure_packet=finite_json(last),
            initial_packet_sha256=digest(finite_json(first)),final_packet_sha256=digest(finite_json(last)),
            completed_hold_ticks=completed,moving_prefix=getattr(api,'feedback_prefix',[]),numeric_packets=packets,numeric_capacity=512,packet_count=count,
            all_numeric_sha256=hashes.hexdigest(),first_ready_tick=feedback.recovery.first_ready,
            first_25_zero_qualified_tick=feedback.recovery.first_qualified,withdrawal_observed=feedback.recovery.withdrawn,forced_terminal_withdrawal_tick=forced_stop_tick,
            uncompensated_final_ticks=(completed-feedback.recovery.first_qualified) if feedback.recovery.first_qualified is not None else None,
            saturation_count=feedback.saturation_count,slew_count=feedback.slew_count,
            actual_final_qdot=targets(api.controls(),arm)[1].tolist(),accepted=False,formal_credit=False,
            whole_retreat_success=False,home_success=False)
    if raw:raise DiagnosticHoldGuardFailed(raw['type']+':'+raw['message'])
    raise DiagnosticHoldCompleted('feedback_calibration_500_ticks_no_return_credit')
