"""One bounded independent N10/c5 probe. Never a production configuration."""
import numpy as np
from provenance import digest
from preview_errors import CandidateInfeasible

BUNDLE_SHA256='20873569d05eb966b0b03133d9c318929cf8e2485cd48a71eb3ab750c7c6b720'
MAX_TICKS=500


class DiagnosticHoldStop(CandidateInfeasible):
    """Terminal diagnostic outcome; cannot complete a return or resume motion."""


class DiagnosticHoldCompleted(DiagnosticHoldStop):
    pass


class DiagnosticHoldGuardFailed(DiagnosticHoldStop):
    pass


def validate_request(mode,diagnostic,cfg):
    if cfg.feedback_scope=='calibration' and mode!='hold':raise ValueError('calibration_requires_fixed_hold')
    if mode is not None and cfg.velocity_feedback!='off' and cfg.feedback_scope!='calibration':raise ValueError('probe_feedback_scope')
    if mode is None:return
    if cfg.vertex_governor:raise ValueError('fixed_setpoint_incompatible_vertex_governor')
    if mode not in ('baseline','hold') or not diagnostic:
        raise ValueError('fixed_setpoint_requires_independent_diagnostic')
    b=diagnostic['bundle']
    if (b.get('target')!=10 or diagnostic['candidate_index']!=5 or diagnostic['max_previews']!=1
        or diagnostic['legacy_insertion'] or cfg.retreat_time_scale!=1 or cfg.high_transfer_branch_seed or cfg.defer_intermediate_home
        or digest(b['hashes'])!=BUNDLE_SHA256):
        raise ValueError('fixed_setpoint_requires_verified_N10_c5_scale1_seedoff_first_preview')


def validate_vertex(api,arm,target,phase,path,dense,certificate):
    if (getattr(api,'fixed_setpoint_diagnostic',None) not in ('baseline','hold')
        or not getattr(api,'independent_diagnostic',False) or arm!='right' or target!=10
        or phase!='retreat' or api.retreat_time_scale!=1 or not certificate.get('certified')
        or len(path)<3 or len(dense)<=201 or not np.array_equal(dense[200],path[2])):
        raise ValueError('fixed_setpoint_incorrect_certified_vertex')
    from return_motion import path_progress
    if path_progress(path,dense[200])!=2.:raise ValueError('fixed_setpoint_incorrect_curve_progress')


def plateau(packets,outcome):
    """Final 50 DISTINCT completed ticks; no short/failed-run equilibrium claim."""
    rows=[p['post_attempt'] for p in packets if p.get('physics_tick_issued') and 'measured_q' in p.get('post_attempt',{})]
    tail=rows[-50:]
    distinct=len(tail)==50 and len({r['readback']['scene_step_count'] for r in tail})==50
    fd_ok=distinct and all(r['finite_difference_velocity'] is not None and
        np.isfinite(r['finite_difference_velocity'][:7]).all() and
        max(abs(np.asarray(r['finite_difference_velocity'][:7])))<.001 for r in tail)
    settled=outcome=='diagnostic_completed' and len(rows)==MAX_TICKS and bool(fd_ok)
    error=lambda r:(np.asarray(r['measured_q'][:7])-np.asarray(r['monitor_reference_q']))
    return dict(plateau_eligible=settled,final50_distinct_ticks=distinct,final50_all_arm_fd_below_001=bool(fd_ok),
        completed_distinct_ticks=len({r['readback']['scene_step_count'] for r in rows}),
        final50_error_change_rad=(error(tail[-1])-error(tail[0])).tolist() if distinct else None,
        full_hold_error_change_rad=(error(rows[-1])-error(packets[0]['precommand'])).tolist() if rows else None,
        final_error_rad=error(rows[-1]).tolist() if rows else None,
        interpretation='Measured plateau eligible for review, not force-balance proof.' if settled else
                       'No equilibrium inference: failed, short, or still drifting.')


def run_hold(api,arm,target,path,certifier,checkpoint):
    from return_clearance import ReturnClearance
    q=np.asarray(path[2],float);v=np.zeros(7);packets=[];outcome='fatal_error';raw=None
    issued_index=200;attempted=issued=tick_attempts=tick_returns=0
    previous=checkpoint['packet'];initial_step=previous['readback']['scene_step_count']
    def capture(index,stage):
        nonlocal previous
        s=api.state();monitor=certifier.monitor(s,q)
        clearance=ReturnClearance(api.world(s),arm,target,True).point(s['robots'][arm]['q'][:7],envelopes=False)
        # Getter order remains state/monitor/clearance, native qf, then controls.
        rb=api.telemetry();now=rb['scene_step_count'];before=previous['readback']['scene_step_count']
        measured=np.asarray(s['robots'][arm]['q']);fd=None
        if now>before:fd=((measured-np.asarray(previous['measured_q']))/((now-before)*rb['scene_timestep_s'])).tolist()
        row=dict(command_index=index,upcoming_command_index=index,command_q=q.tolist(),command_qdot=v.tolist(),
            monitor_reference_q=q.tolist(),monitor_reference_index=issued_index,last_issued_index=issued_index,
            last_issued_q=q.tolist(),original_reference_command_index=200,monitor_path_progress=2.,
            packet_stage=stage,measured_q=measured.tolist(),measured_qdot=s['robots'][arm]['qdot'],
            finite_difference_velocity=fd,readback=rb,drive_readback=api.controls(),state=s,
            monitor=monitor,clearance=clearance,contacts=api.contacts(),
            completed_hold_scene_steps=now-initial_step)
        if now!=before:previous=row
        return row
    def guard(row):
        if not row['monitor']['safe']:
            raise CandidateInfeasible('postrelease_precommand_envelope:'+','.join(row['monitor']['reasons']))
        if row['clearance'].get('safe') is False:raise CandidateInfeasible('postrelease_measured_clearance')
    try:
        for n in range(MAX_TICKS):
            index=201+n;pre=capture(index,'precommand');row=dict(hold_tick=n+1,precommand=pre,
                command_attempted=False,command_issued=False,tick_attempted=False,tick_completed=False)
            packets.append(row)
            try:
                guard(pre)
                row['command_attempted']=True;attempted+=1
                api.command(arm,q,v);row['command_issued']=True;issued+=1;issued_index=index
                row['tick_attempted']=True;tick_attempts+=1
                api.tick(target,False,True);row['tick_completed']=True;tick_returns+=1
            finally:
                row['command_completion_uncertain']=row['command_attempted'] and not row['command_issued']
                try:row['post_attempt']=capture(index,'post_attempt')
                except Exception as measurement_error:
                    row['post_attempt']=dict(measurement_error=type(measurement_error).__name__+':'+str(measurement_error),readback=api.telemetry())
                    row['physics_tick_issued']=row['post_attempt']['readback']['scene_step_count']>pre['readback']['scene_step_count']
                    raise
                row['physics_tick_issued']=row['post_attempt']['readback']['scene_step_count']>pre['readback']['scene_step_count']
            guard(row['post_attempt'])
        outcome='diagnostic_completed'
    except Exception as exc:
        import traceback
        raw=dict(type=type(exc).__name__,message=str(exc),traceback=''.join(traceback.format_exception(type(exc),exc,exc.__traceback__)))
        if isinstance(exc,CandidateInfeasible):outcome='diagnostic_guard_failed'
        else:raise
    finally:
        api.event('fixed_setpoint_hold',schema='fixed-setpoint-hold/11',outcome=outcome,raw_outcome=raw,
            accepted=False,formal_credit=False,checkpoint_eligible=False,trajectory_resume_allowed=False,
            reference_q=q.tolist(),reference_command_index=200,path_progress=2.,max_ticks=MAX_TICKS,
            command_attempts=attempted,commands_issued=issued,tick_attempts=tick_attempts,tick_returns=tick_returns,
            completed_scene_ticks=api.telemetry()['scene_step_count']-initial_step,
            packet_capacity=MAX_TICKS,packets=packets,analysis=plateau(packets,outcome))
    if outcome=='diagnostic_completed':raise DiagnosticHoldCompleted('fixed_setpoint_500_ticks_completed_no_return_credit')
    raise DiagnosticHoldGuardFailed(raw['type']+':'+raw['message'])
