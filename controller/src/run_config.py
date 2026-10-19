"""Explicit experiment contract. SI units; immutable configuration per run."""
from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class RunConfig:
    mode: str = 'roundtrip'
    count: int = 10
    seed: int = 2
    layout: str = 'original'
    split: str = 'development'
    smoke_id: int = 3
    position_tolerance_m: float = .010
    rotation_tolerance_deg: float = 10.
    tolerance_reason: str = 'Default 10mm/10deg: stricter than tray acceptance; allows measured PD/release settling. Raw errors always retained.'
    speed_tolerance_m_s: float = .03
    spin_tolerance_rad_s: float = .15
    arm_joint_tolerance_rad: float = .025
    arm_speed_tolerance_rad_s: float = .03
    layout_penetration_m: float = .001
    layout_drift_m: float = .002
    layout_rotation_drift_deg: float = 2.
    experience: bool = False
    optimize_geometry: bool = False
    planner_seed: int = 20261007
    retreat_time_scale: int = 1
    high_transfer_branch_seed: bool = False
    vertex_governor: bool = False
    joint_step_refinement: bool = False
    home_coarse: bool = False
    home_contact_envelope: bool = False
    home_progress: bool = False
    home_progress_bound: bool = False
    reserve_home_workspace: bool = False
    home_cpu_profile: bool = False
    home_cpu_profile_max_calls: int = 1
    home_entry_hold: bool = False
    home_nominal_speed_rad_s: float = .1
    velocity_feedback: str = 'off'
    feedback_scope: str = 'retreat'
    feedback_telemetry: bool = False
    defer_intermediate_home: bool = False
    sleep_copy_trace_object_id: int = None
    placement_trace_object_id: int = None
    return_preview_cap: int = 80
    return_time_cap_s: int = 1800

    def __post_init__(self):
        if self.sleep_copy_trace_object_id is not None and (type(self.sleep_copy_trace_object_id) is not int or self.sleep_copy_trace_object_id!=17):raise ValueError('sleep_copy_trace_object_id')
        if self.placement_trace_object_id is not None:
            if (type(self.placement_trace_object_id) is not int or self.placement_trace_object_id<=0
                    or not self.reserve_home_workspace):raise ValueError('placement_trace_object_id')
        from home_cpu_profile import validate
        validate(self.home_cpu_profile,self.home_cpu_profile_max_calls)
        if self.home_cpu_profile and not self.reserve_home_workspace:raise ValueError('home_cpu_profile_reserved_diagnostic_only')
        from velocity_feedback import SETTINGS, SCOPES
        if self.velocity_feedback not in SETTINGS or self.feedback_scope not in SCOPES:raise ValueError('feedback configuration')
        if type(self.feedback_telemetry) is not bool:raise ValueError('feedback_telemetry')
        if self.velocity_feedback!='off' and (self.retreat_time_scale!=1 or (self.feedback_scope!='calibration' and not self.vertex_governor)):
            raise ValueError('feedback requires governor and scale1 outside calibration')
        if self.feedback_scope=='calibration' and (self.vertex_governor or self.joint_step_refinement):raise ValueError('calibration preserves coarse prefix')
        if type(self.joint_step_refinement) is not bool:raise ValueError('joint_step_refinement must be bool')
        if type(self.home_coarse) is not bool or (self.home_coarse and not self.joint_step_refinement):
            raise ValueError('home_coarse requires joint_step_refinement and bool')
        if type(self.home_contact_envelope) is not bool or (self.home_contact_envelope and not self.home_coarse):
            raise ValueError('home_contact_envelope requires home_coarse and bool')
        if type(self.home_nominal_speed_rad_s) not in (int,float) or self.home_nominal_speed_rad_s not in (.1,.06):
            raise ValueError('home_nominal_speed_rad_s must be .1 or predeclared .06')
        if type(self.home_progress) is not bool or (self.home_progress and (not self.home_contact_envelope or self.home_nominal_speed_rad_s!=.06 or self.velocity_feedback!='k6' or self.feedback_scope!='all_unloaded')):
            raise ValueError('home_progress requires contact envelope headroom06 k6 all_unloaded and bool')
        if type(self.home_progress_bound) is not bool or (self.home_progress_bound and not self.home_progress):
            raise ValueError('home_progress_bound requires home_progress and bool')
        if type(self.reserve_home_workspace) is not bool or (self.reserve_home_workspace and
                (not self.home_progress_bound or self.mode!='roundtrip' or self.split!='development' or self.defer_intermediate_home)):
            raise ValueError('reserve_home_workspace requires bounded progress continuous development roundtrip and explicit every HOME')
        if type(self.home_entry_hold) is not bool or (self.home_entry_hold and not self.reserve_home_workspace):
            raise ValueError('home_entry_hold requires reserved continuous diagnostic and bool')
        if self.home_nominal_speed_rad_s!=.1 and not self.home_coarse:
            raise ValueError('HOME headroom requires home_coarse')
        if self.joint_step_refinement and (not self.vertex_governor or self.retreat_time_scale!=1):
            raise ValueError('joint_step_refinement requires vertex_governor and retreat_time_scale=1')
        if type(self.vertex_governor) is not bool:raise ValueError("vertex_governor must be bool")
        if type(self.defer_intermediate_home) is not bool:raise ValueError('defer_intermediate_home must be bool')
        for name,maximum in (('return_preview_cap',400),('return_time_cap_s',14400)):
            if type(getattr(self,name)) is not int or not 1<=getattr(self,name)<=maximum:raise ValueError(name)
        if not self.defer_intermediate_home and (self.return_preview_cap!=80 or self.return_time_cap_s!=1800):
            raise ValueError('return caps require defer_intermediate_home')
        if type(self.high_transfer_branch_seed) is not bool:raise ValueError("high_transfer_branch_seed must be bool")
        if self.high_transfer_branch_seed and self.split != "development":raise ValueError("branch seeds are development only")
        if type(self.retreat_time_scale) is not int or self.retreat_time_scale not in (1,2):
            raise ValueError('retreat_time_scale must be integer 1 or 2')
        if self.mode not in ('roundtrip', 'smoke', 'extraction', 'layout'): raise ValueError('invalid mode')
        if self.mode == 'layout' and (self.experience or self.optimize_geometry): raise ValueError('layout has no candidate ranking or collision cache')
        if self.count not in (10,15,20): raise ValueError('count must be 10/15/20')
        if self.layout not in ('original','generated'): raise ValueError('invalid layout')
        if self.layout == 'original' and (self.count != 10 or self.seed != 2):
            raise ValueError('original means the frozen ten-object seed2 scene')
        if self.mode == 'smoke' and self.layout != 'original': raise ValueError('smoke requires original clutter')
        if self.split not in ('development','heldout'): raise ValueError('invalid split')
        if not isinstance(self.seed,int) or not isinstance(self.planner_seed,int): raise ValueError('integer seeds required')
        for name,value in asdict(self).items():
            if name.endswith(('_m','_deg','_m_s','_rad','_rad_s')):
                if not math.isfinite(value) or value <= 0: raise ValueError('invalid threshold: '+name)
        if (self.position_tolerance_m != .01 or self.rotation_tolerance_deg != 10.) and self.tolerance_reason.startswith('Default'):
            raise ValueError('changed pose tolerances require an explicit justification')

    def to_dict(self):
        data=asdict(self)
        if self.sleep_copy_trace_object_id is None:data.pop('sleep_copy_trace_object_id')
        if self.placement_trace_object_id is None:data.pop('placement_trace_object_id')
        if not self.home_cpu_profile:
            data.pop('home_cpu_profile');data.pop('home_cpu_profile_max_calls')
        if self.velocity_feedback=='off':data.pop('velocity_feedback')
        if self.feedback_scope=='retreat':data.pop('feedback_scope')
        if not self.feedback_telemetry:data.pop('feedback_telemetry')
        if not self.joint_step_refinement:data.pop('joint_step_refinement')
        if not self.home_coarse:data.pop('home_coarse')
        if not self.home_contact_envelope:data.pop('home_contact_envelope')
        if not self.home_progress:data.pop('home_progress')
        if not self.home_progress_bound:data.pop('home_progress_bound')
        if not self.reserve_home_workspace:data.pop('reserve_home_workspace')
        if not self.home_entry_hold:data.pop('home_entry_hold')
        if self.home_nominal_speed_rad_s==.1:data.pop('home_nominal_speed_rad_s')
        if not self.vertex_governor:data.pop("vertex_governor")
        # Preserve every historical OFF configuration byte after canonicalization.
        if not self.defer_intermediate_home:
            for name in ('defer_intermediate_home','return_preview_cap','return_time_cap_s'):data.pop(name)
        return data


def load_config(path):
    import json
    return RunConfig(**json.loads(path.read_text()))


def validate_diagnostic_config(config, original):
    """Only declared retreat timing and transfer IK hint may differ from the frozen prefix."""
    if config.reserve_home_workspace:raise ValueError('workspace_reservation_requires_fresh_continuous_episode')
    a=config.to_dict();b=RunConfig(**original).to_dict()
    for name in ('retreat_time_scale','high_transfer_branch_seed'):
        a.pop(name);b.pop(name)
    for name in ('velocity_feedback','feedback_scope','feedback_telemetry','vertex_governor','joint_step_refinement','home_coarse','home_contact_envelope','home_progress','home_progress_bound','home_nominal_speed_rad_s','defer_intermediate_home','return_preview_cap','return_time_cap_s'):
        a.pop(name,None);b.pop(name,None)
    if a!=b:raise ValueError('diagnostic_config_mismatch')
