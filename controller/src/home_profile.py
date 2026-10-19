"""Opt-in original HOME polyline with v18 deterministic headroom clocks; unchanged refined RETREAT/handoff.

Certification remains conditional on the existing tracking/speed/world bounds.
Longer uninterrupted edges are not a proof of actuator tracking or readiness.
"""
from provenance import digest
from preview_errors import CandidateInfeasible
from return_clearance import ReturnClearance
from motion_refinement import CAPS
from home_admission import checked_vertices, profiles
from vertex_governor import finite_json


def prepare_coarse(api, arm, target, phase, path, old_certificate):
    certificate = None
    try:
        if (phase != 'home' or not getattr(api, 'joint_step_refinement', False)
                or not getattr(api, 'vertex_governor', False)
                or getattr(api, 'retreat_time_scale', 1) != 1
                or getattr(api, 'fixed_setpoint_diagnostic', None) is not None):
            raise CandidateInfeasible('home_coarse_incompatible')
        original = checked_vertices(path)
        # Bounded profile validation before allocation/commands, exact existing
        # quintic schedule, duplicate endpoints and safe governor splices.
        speed=getattr(api,'home_nominal_speed_rad_s',.1)
        original,dense,velocity,original_clock,clock=profiles(original,speed)
        if old_certificate.get('certified') is not True:
            raise CandidateInfeasible('home_coarse_original_certificate_failed')
        state = api.state()
        certifier = ReturnClearance(api.world(state), arm, target, False)
        certificate = certifier.certify(original, max_nodes=CAPS['certificate_nodes'])
        if certificate.get('certified') is not True:
            raise CandidateInfeasible('home_coarse_fresh_certificate_failed')
        api.event('home_coarse_prepared', schema='home-coarse/18', arm=arm,
                  object_id=target, motion_phase=phase, original_vertices=original,
                  original_path_sha256=digest(original), executed_path_sha256=digest(original),
                  original_certificate=old_certificate, fresh_certificate=certificate,
                  original_certificate_sha256=digest(old_certificate),
                  fresh_certificate_sha256=digest(certificate), original_profile=original_clock,
                  executed_profile=None if getattr(api,'home_progress',False) else clock,
                  **(dict(nominal_comparison_profile=clock,dynamic_progress=True,executed_clock='home-progress/22' if getattr(api,'home_progress_bound',False) else 'home-progress/21') if getattr(api,'home_progress',False) else {}), caps=CAPS, fresh_state_sha256=digest(state),
                  controls_sha256=digest(api.controls()), rng_sha256=digest(api.rng.bit_generator.state),
                  nominal_speed_limit_rad_s=speed, correction_reserve_rad_s=.04 if speed==.06 else 0.,
                  clocks_changed=speed!=.1,
                  timing=('HOME-only measured progress; quintic below is comparison only' if getattr(api,'home_progress',False) else ('HOME-only quintic reparameterization; original clocks CHANGED' if speed!=.1 else
                          'original coarse quintic clocks'))+'; governor dwell remains additional',
                  conditional_tracking_only=True, formal_credit=False)
        return original, certifier, certificate, dense, velocity
    except Exception as exc:
        api.event('home_coarse_rejected', arm=arm, object_id=target, motion_phase=phase,
                  reason=type(exc).__name__+':'+str(exc), original_vertices=finite_json(path),
                  original_certificate=finite_json(old_certificate),
                  fresh_certificate=finite_json(certificate), no_fallback=True, formal_credit=False)
        raise


class PhaseReceipt:
    """Opt-in compact lifecycle evidence, including preflight and hard failures.

    Observes existing checkpoint events without retaining per-tick world state.
    Never issues a command, cleanup or tick.
    """
    def __init__(self, api, arm, target, phase):
        self.api, self.arm, self.target, self.phase = api, arm, target, phase
        self.start = api.telemetry()['scene_step_count']
        self.checkpoints = []
        self.plan = None
        self.entry_controls = digest(api.controls())
        self.api.event('motion_phase_entry', schema='motion-phase/18', arm=arm,
                       object_id=target, motion_phase=phase, scene_step=self.start,
                       state_sha256=digest(api.state()), controls_sha256=self.entry_controls,
                       selection=('coarse_headroom06' if getattr(api,'home_nominal_speed_rad_s',.1)==.06 else 'original_coarse') if phase == 'home' else 'existing_refined',
                       nominal_speed_limit_rad_s=getattr(api,'home_nominal_speed_rad_s',.1) if phase=='home' else .1)

    def __getattr__(self, name):
        return getattr(self.api, name)

    def event(self, kind, **row):
        if kind == 'return_motion_plan':
            self.plan = {k:row[k] for k in ('path_sha256','certificate_sha256',
                                           'dense_command_sha256','dense_commands')}
        if kind == 'vertex_governor_vertex':
            self.checkpoints.append({k:row[k] for k in ('vertex','reference_index',
                'resume_index','status','reason','completed_hold_ticks','motion_added_ticks','streak')})
        return self.api.event(kind, **row)

    def finish(self, reason):
        from velocity_feedback import targets
        end = self.api.telemetry()['scene_step_count']
        try:final_targets=finite_json(targets(self.api.controls(),self.arm)[1])
        except Exception as exc:final_targets=dict(unavailable=type(exc).__name__+':'+str(exc))
        self.api.event('motion_phase_exit', schema='motion-phase/18', arm=self.arm,
                       object_id=self.target, motion_phase=self.phase, failed=reason is not None,
                       reason=reason, initial_scene_step=self.start, final_scene_step=end,
                       completed_ticks=end-self.start, plan=self.plan,
                       checkpoints=self.checkpoints, checkpoint_count=len(self.checkpoints),
                       completed_hold_ticks=sum(r['completed_hold_ticks'] for r in self.checkpoints),
                       state_sha256=digest(finite_json(self.api.state())),
                       controls_sha256=digest(finite_json(self.api.controls())),
                       actual_velocity_targets=final_targets,
                       formal_credit=False)
