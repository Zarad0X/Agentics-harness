"""Diagnostic-only, read-only HOME observer. No simulator import or setters.

The paired intervention is measurement OFF/ON, NOT a physics intervention.
Public APIs were inspected in SAPIEN's physx.pyi; unsupported reads are explicit.
All observations precede preview teardown. No tick is added on any exit path.
"""
from collections import deque
import hashlib
import numpy as np
from feedback_telemetry import get, numeric
from provenance import canonical, digest

NAME = '216_inner0926_s1c0n17'
CAPS = dict(tail=256, pre=64, post=16, windows=6, contacts=128, points=256)


def validate_request(mode, diagnostic, cfg):
    if mode is None:
        return
    if (type(cfg.home_progress_bound) is not bool or (cfg.home_progress_bound and not cfg.home_progress)
            or cfg.reserve_home_workspace):
        raise ValueError('n17_observer_requires_single_shadow_progress_contract')
    # v20 deliberately admits the HOME planning intervention. Observation is
    # still identical and read-only; v19 identical-motion audit is not its gate.
    if type(cfg.home_progress) is not bool or (cfg.home_progress and not cfg.home_contact_envelope):
        raise ValueError('n17_observer_invalid_home_progress_config')
    if type(cfg.home_contact_envelope) is not bool:
        raise ValueError('n17_observer_invalid_home_contact_config')
    if (mode not in ('baseline', 'observe') or not diagnostic
            or diagnostic['bundle'].get('target') != 10
            or diagnostic.get('candidate_index') != 5
            or diagnostic.get('max_previews') != 1
            or diagnostic.get('legacy_insertion')
            or diagnostic.get('fixed_setpoint_diagnostic')
            or diagnostic.get('continuation_diagnostic')
            or not cfg.home_coarse or cfg.home_nominal_speed_rad_s != .06
            or cfg.velocity_feedback != 'k6' or cfg.feedback_scope != 'all_unloaded'
            or cfg.high_transfer_branch_seed or not cfg.experience
            or cfg.defer_intermediate_home):
        raise ValueError('n17_observer_requires_n10_c5_headroom06_single_shadow')


def prop(obj, name):
    try:
        return dict(available=True, value=numeric(getattr(obj, name)))
    except Exception as exc:
        return dict(available=False, reason=name + ':' + type(exc).__name__ + ':' + str(exc))


def shape_metadata(shape):
    row = dict(type=type(shape).__name__, **{n: get(shape, 'get_' + n) for n in
        ('contact_offset', 'rest_offset', 'collision_groups', 'local_pose', 'density')})
    try:
        mat = shape.get_physical_material()
        row['material'] = {n: get(mat, 'get_' + n) for n in
                           ('static_friction', 'dynamic_friction', 'restitution')}
    except Exception as exc:
        row['material'] = dict(available=False, reason=type(exc).__name__ + ':' + str(exc))
    # Actual cooked geometry and transforms, not nominal table heights.
    if hasattr(shape, 'get_half_size'):
        row['half_size'] = get(shape, 'get_half_size')
    elif hasattr(shape, 'get_vertices'):
        row['vertices'] = get(shape, 'get_vertices')
        row['scale'] = get(shape, 'get_scale')
    else:
        row['geometry'] = dict(available=False, reason='No inspected box/convex geometry getter')
    return row


def body_metadata(body, label):
    return dict(label=label, type=type(body).__name__, pose=get(body.entity, 'get_pose'),
        properties={n: get(body, 'get_' + n) for n in
            ('mass', 'inertia', 'cmass_local_pose', 'linear_damping', 'angular_damping',
             'sleep_threshold', 'solver_position_iterations', 'solver_velocity_iterations',
             'kinematic', 'disable_gravity', 'max_contact_impulse')},
        is_sleeping=prop(body, 'is_sleeping'),
        shapes=[shape_metadata(s) for s in body.get_collision_shapes()])


class N17Observer:
    def __init__(self, env, clock):
        self.env, self.clock = env, clock
        bodies = (list(env.scene.physx_system.get_rigid_dynamic_components())
                  + list(env.scene.physx_system.get_rigid_static_components()))
        exact = [b for b in bodies if b.entity.get_name() == NAME]
        if len(exact) != 1 or env.objects[17].actor != exact[0].entity:
            raise ValueError('n17_observer_exact_identity')
        self.body = exact[0]
        self.labels = [(b, dict(name=b.entity.get_name(), arm=None)) for b in bodies]
        for arm in ('left', 'right'):
            self.labels.extend((b, dict(name=b.entity.get_name(), arm=arm))
                               for b in getattr(env.robot, arm + '_entity').get_links())
        selected = [(b, label) for b, label in self.labels if label['name'] in
                    (NAME, '211_inner0926_s1c0n12', 'table', 'real10_catch_right',
                     'panda_leftfinger', 'panda_rightfinger')]
        self.metadata = dict(bodies=[body_metadata(b, label) for b, label in selected],
            scene_timestep_s=float(env.scene.get_timestep()))
        try:
            cfg = env.scene.physx_system.get_config()
            self.metadata['scene_config'] = {n: prop(cfg, n) for n in
                ('gravity', 'bounce_threshold', 'enable_ccd', 'enable_enhanced_determinism',
                 'enable_friction_every_iteration', 'enable_pcm', 'enable_tgs')}
        except Exception as exc:
            self.metadata['scene_config'] = dict(available=False, reason=str(exc))
        self.tail = deque(maxlen=CAPS['tail'])
        self.windows, self.seen, self.errors = [], set(), []
        self.hash = hashlib.sha256()
        self.count = 0
        self.truncated_samples = 0
        self.last_step = None
        self.previous = None
        self.max_speed = self.max_spin = 0.
        self.capture('home_entry')

    def label(self, body):
        for known, label in self.labels:
            if known == body:
                return label
        return dict(name=body.entity.get_name(), arm=None, unregistered=True)

    def contacts(self):
        rows = []
        truncated = False
        for contact in self.env.scene.get_contacts():
            if not any(b == self.body for b in contact.bodies) or not contact.points:
                continue
            if len(rows) >= CAPS['contacts']:
                truncated = True
                continue
            points = list(contact.points)
            truncated |= len(points) > CAPS['points']
            try:
                shape_pair = [dict(contact_offset=get(s, 'get_contact_offset'),
                                   rest_offset=get(s, 'get_rest_offset'),
                                   local_pose=get(s, 'get_local_pose')) for s in contact.shapes]
            except Exception as exc:
                shape_pair = dict(available=False, reason=type(exc).__name__ + ':' + str(exc))
            rows.append(dict(bodies=[self.label(b) for b in contact.bodies], shapes=shape_pair,
                separation_min=min(float(p.separation) for p in points),
                impulse_max=max(float(np.linalg.norm(p.impulse)) for p in points),
                point_count=len(points), points=[dict(separation=float(p.separation),
                    impulse=numeric(p.impulse), normal=numeric(p.normal), position=numeric(p.position))
                    for p in points[:CAPS['points']]]))
        return rows, truncated

    def capture(self, stage):
        step = self.clock()
        if step == self.last_step:
            return  # No fabricated post-step sample after a pre-tick throw.
        pose = self.body.entity.get_pose()
        linear = numeric(self.body.get_linear_velocity())
        angular = numeric(self.body.get_angular_velocity())
        contacts, truncated = self.contacts()
        row = dict(scene_step=step, scene_timestep_s=float(self.env.scene.get_timestep()),
            stage=stage, pose=numeric(pose.p) + numeric(pose.q), linear_velocity=linear,
            angular_velocity=angular, speed_m_s=float(np.linalg.norm(linear)),
            spin_rad_s=float(np.linalg.norm(angular)), is_sleeping=prop(self.body, 'is_sleeping'),
            contacts=contacts, contacts_truncated=truncated,
            contact_step=step, contact_scope='last completed scene step; entry follows preceding motion',
            raw_dt_used=True)
        if self.last_step is not None and step != self.last_step + 1:
            self.errors.append(dict(reason='nonconsecutive_scene_step', before=self.last_step, after=step))
        triggers = []
        if self.previous:
            if row['is_sleeping'] != self.previous['is_sleeping']:
                triggers.append('sleep_transition')
            if row['pose'] != self.previous['pose']:
                triggers.append('first_pose_change')
        if any(any(b['arm'] is not None for b in c['bodies']) for c in contacts):
            triggers.append('first_robot_contact_record')
        if row['spin_rad_s'] > .05:
            triggers.append('spin_above_005')
        if row['spin_rad_s'] > .15:
            triggers.append('spin_above_guard')
        if row['speed_m_s'] > .03:
            triggers.append('speed_above_guard')
        for window in self.windows:
            if window['remaining']:
                window['rows'].append(row)
                window['remaining'] -= 1
        for trigger in triggers:
            if trigger not in self.seen and len(self.windows) < CAPS['windows']:
                self.seen.add(trigger)
                self.windows.append(dict(trigger=trigger, scene_step=step,
                    rows=list(self.tail)[-CAPS['pre']:] + [row], remaining=CAPS['post']))
        self.tail.append(row)
        self.hash.update(canonical(row))
        self.count += 1
        self.truncated_samples += int(truncated)
        self.max_speed = max(self.max_speed, row['speed_m_s'])
        self.max_spin = max(self.max_spin, row['spin_rad_s'])
        self.previous, self.last_step = row, step

    def safe_capture(self, stage):
        try:
            self.capture(stage)
        except Exception as exc:
            # Observation failure must not mask/replace an existing physical guard.
            if len(self.errors) < 8:
                self.errors.append(dict(scene_step=self.clock(), reason=type(exc).__name__ + ':' + str(exc)))

    def finish(self):
        return dict(metadata=self.metadata, metadata_sha256=digest(self.metadata), caps=CAPS,
            samples=self.count, all_samples_sha256=self.hash.hexdigest(),
            max_speed_m_s=self.max_speed, max_spin_rad_s=self.max_spin,
            tail=list(self.tail), windows=self.windows, errors=self.errors,
            truncated_samples=self.truncated_samples,
            complete=not self.errors and self.truncated_samples == 0,
            no_extra_ticks=True, no_physics_mutation=True,
            interpretation='Contact record is not collision proof; impulse and signed gap remain separate. '
                'Sleeping unavailable stays unknown; zero velocity is not sleeping proof.')


def observe_motion(api, env, mode, execute, arm, target, phase, home):
    """Wrap only the HOME API tick; original runner tick and guards are untouched."""
    if phase != 'home' or mode not in ('baseline', 'observe'):
        raise ValueError('n17_observer_home_only')
    observer = None
    error = None
    initial_step = api.telemetry()['scene_step_count']
    original_tick = api.tick
    if mode == 'observe':
        observer = N17Observer(env, lambda: api.telemetry()['scene_step_count'])
        def observed_tick(*args, **kw):
            try:
                return original_tick(*args, **kw)
            finally:
                observer.safe_capture('completed_tick')
        api.tick = observed_tick
    try:
        return execute(api, arm, target, phase, home)
    except Exception as exc:
        error = type(exc).__name__ + ':' + str(exc)
        raise
    finally:
        api.tick = original_tick  # Python wrapper only; no actuator cleanup.
        api.event('n17_observation', schema='n17-observation/19', mode=mode, object_id=17,
            actor_name=NAME, arm=arm, motion_phase=phase, motion_target=target,
            initial_scene_step=initial_step, final_scene_step=api.telemetry()['scene_step_count'],
            reason=error, observation=observer.finish() if observer else None,
            formal_credit=False)
