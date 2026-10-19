"""v15 bounded unloaded joint-linear refinement; no planner/physics fallback.

Same ordered curve within 1e-12 rad, exact original endpoints (including
duplicates). Each subedge receives a NEW quintic clock. Timing and target qd
change. Geometry certification is conditional, never a tracking guarantee.
"""
import math
import numpy as np
import astra_planner as planner
from provenance import digest
from preview_errors import CandidateInfeasible
from return_clearance import ReturnClearance, DT, MAX_VELOCITY, MAX_ACCELERATION
from vertex_governor import splice_bounds, finite_json

STEP_RAD = .01
CURVE_ATOL_RAD = 1e-12
MAX_VERTICES = 512
MAX_DENSE_COMMANDS = 30000  # Includes command zero; excludes governor/terminal.
MAX_EXECUTION_TICKS = 36000  # 30000 + 5000 holds + 1000 separate terminal.
CAPS = dict(vertices=MAX_VERTICES, dense_commands_including_initial=MAX_DENSE_COMMANDS,
            certificate_nodes=4096, certificate_depth=18, holds_per_vertex=500,
            holds_per_motion=5000, terminal_ticks_separate=1000,
            total_execution_ticks=MAX_EXECUTION_TICKS, total_nominal_s=144.)


def reject(reason):
    raise CandidateInfeasible('motion_refinement_' + reason)


def vertices(value):
    # Bound before converting/allocating. Nothing outside this helper is mutated.
    if not 1 <= len(value) <= MAX_VERTICES:reject('vertex_budget')
    q = np.asarray(value, dtype=float)
    if q.shape != (len(value), 7) or not np.isfinite(q).all():reject('nonfinite_or_shape')
    return q


def subdivisions(path):
    counts = []
    total = 1
    for a, b in zip(path[:-1], path[1:]):
        with np.errstate(over='ignore', invalid='ignore'):
            ratio = float(np.max(np.abs(b-a))) / STEP_RAD
        if not math.isfinite(ratio) or ratio > MAX_VERTICES:reject('vertex_budget')
        n = max(1, math.ceil(ratio))
        total += n
        if total > MAX_VERTICES:reject('vertex_budget')
        counts.append(n)
    return counts


def verify_mapping(original, refined, counts, original_indices):
    a, r = vertices(original), vertices(refined)
    expected = subdivisions(a)
    indices = [0] + np.cumsum(expected).tolist()
    if counts != expected or original_indices != indices or len(r) != 1+sum(expected):
        reject('curve_order_mapping')
    if not np.array_equal(r[indices], a):reject('original_endpoints')
    cursor = 0
    for x, y, n in zip(a[:-1], a[1:], counts):
        for j in range(1, n+1):
            cursor += 1
            want = y if j == n else x+(y-x)*(j/n)
            if np.max(np.abs(r[cursor]-want)) > CURVE_ATOL_RAD:reject('curve_order_mapping')
            if np.max(np.abs(r[cursor]-r[cursor-1])) > STEP_RAD+CURVE_ATOL_RAD:reject('joint_step')


def refine(path):
    p = vertices(path)
    counts = subdivisions(p)
    out = [p[0].copy()]
    for a, b, n in zip(p[:-1], p[1:], counts):
        out.extend(b.copy() if j == n else a+(b-a)*(j/n) for j in range(1, n+1))
    result = np.asarray(out)
    mapping = dict(subdivisions=counts, original_vertex_indices=[0]+np.cumsum(counts).tolist(),
                   curve_atol_rad=CURVE_ATOL_RAD, fraction_rule='j/n in original edge order',
                   exact_original_endpoints=True, duplicate_edges_preserved=True)
    verify_mapping(p, result, counts, mapping['original_vertex_indices'])
    return result.tolist(), mapping


def command_count(path, max_velocity=MAX_VELOCITY):
    """Integer count identical to existing resample_path, before dense allocation."""
    if max_velocity not in (MAX_VELOCITY,.06):reject('unsupported_nominal_speed')
    p = vertices(path)
    total = 1
    intervals = []
    for a, b in zip(p[:-1], p[1:]):
        with np.errstate(over='ignore', invalid='ignore'):
            distance = float(np.max(np.abs(b-a)))
        if not math.isfinite(distance):reject('nonfinite_edge')
        ratio = max(1.875*distance/max_velocity, math.sqrt(5.78*distance/MAX_ACCELERATION))/DT
        if not math.isfinite(ratio) or ratio > MAX_DENSE_COMMANDS:reject('dense_command_budget')
        n = max(1, math.ceil(ratio))
        total += n
        if total > MAX_DENSE_COMMANDS:reject('dense_command_budget')
        intervals.append(n)
    return total, intervals


def profile(path, max_velocity=MAX_VELOCITY):
    count, intervals = command_count(path,max_velocity)
    p = vertices(path)
    d = np.asarray(planner.resample_path(p, max_velocity=max_velocity, dt=DT,
                                       max_acceleration=MAX_ACCELERATION))
    if d.shape != (count, 7) or not np.isfinite(d).all():reject('dense_profile_shape_or_finite')
    # Use the known segment schedule, avoiding ambiguous projection at repeated
    # vertices, tiny edges or a curve revisiting an earlier configuration.
    if not np.array_equal(d[0], p[0]):reject('dense_endpoint')
    offset = 0
    for a, b, n in zip(p[:-1], p[1:], intervals):
        u = np.arange(1, n+1, dtype=float)/n
        expected = a+(b-a)*(u**3*(10-15*u+6*u*u))[:, None]
        expected[-1] = b
        if not np.array_equal(d[offset+n], b):reject('dense_endpoint')
        if np.max(np.abs(d[offset+1:offset+n+1]-expected)) > CURVE_ATOL_RAD:reject('dense_curve_order')
        offset += n
    v = np.vstack((np.diff(d, axis=0)/DT, np.zeros((1, 7))))
    acceleration = np.diff(np.vstack((np.zeros((1, 7)), v)), axis=0)/DT
    if not np.isfinite(v).all() or not np.isfinite(acceleration).all():reject('nonfinite_profile')
    if np.max(np.abs(v)) > max_velocity+(1e-8 if max_velocity==MAX_VELOCITY else 1e-12) or np.max(np.abs(acceleration)) > MAX_ACCELERATION+1e-5:
        reject('command_profile_bound')
    splices = splice_bounds(path, d, v, max_velocity=max_velocity)
    if not splices['safe']:reject('unsupported_splice_before_motion')
    return d, v, dict(dense_commands_including_initial=count, interpolation_intervals=count-1,
                     interpolation_nominal_s=(count-1)*DT, dense_execution_nominal_s=count*DT,
                     command_sha256=digest(dict(q=d.tolist(), qdot=v.tolist())),
                     max_velocity_rad_s=float(np.max(np.abs(v))),
                     max_acceleration_rad_s2=float(np.max(np.abs(acceleration))), splices=splices)


def prepare(api, arm, target, phase, path, old_certificate):
    """Complete fail-closed preflight; caller has issued no motion command."""
    original = refined = mapping = certificate = None
    try:
        if not getattr(api, 'vertex_governor', False):reject('requires_vertex_governor')
        if phase not in ('retreat', 'home', 'handoff'):reject('unloaded_phase_only')
        if getattr(api, 'retreat_time_scale', 1) != 1:reject('requires_scale_one')
        if getattr(api, 'fixed_setpoint_diagnostic', None) is not None:reject('incompatible_probe')
        original = vertices(path).tolist()
        refined, mapping = refine(original)
        command_count(original)
        command_count(refined)
        # Fresh world, fresh full certificate, never relabel/reuse the old one.
        state = api.state()
        certifier = ReturnClearance(api.world(state), arm, target, phase == 'retreat')
        certificate = certifier.certify(refined, max_nodes=CAPS['certificate_nodes'])
        if certificate.get('certified') is not True:reject('fresh_certificate_failed')
        od, ov, op = profile(original)
        dense, velocity, rp = profile(refined)
        same = np.all(od[:min(len(od), len(dense))] == dense[:min(len(od), len(dense))], axis=1)
        same &= np.all(ov[:len(same)] == velocity[:len(same)], axis=1)
        changed = np.flatnonzero(~same)
        first = int(changed[0]) if len(changed) else (len(same) if len(od) != len(dense) else None)
        api.event('motion_refinement_prepared', schema='motion-refinement/15', arm=arm, object_id=target,
                  motion_phase=phase, original_vertices=original, refined_vertices=refined, mapping=mapping,
                  original_path_sha256=digest(original), refined_path_sha256=digest(refined),
                  original_certificate_sha256=digest(old_certificate), fresh_certificate=certificate,
                  fresh_certificate_sha256=digest(certificate), original_profile=op, refined_profile=rp,
                  caps=CAPS, step_rad=STEP_RAD, fresh_state_sha256=digest(state),
                  controls_sha256=digest(api.controls()), rng_sha256=digest(api.rng.bit_generator.state),
                  first_changed_dense_command_index=first,
                  timing='new quintic clocks and dense q/qd; old timing NOT preserved',
                  observed_comparison_boundary='before motion command zero; no post-change equality',
                  formal_credit=False)
        return refined, certifier, certificate, dense, velocity
    except Exception as exc:
        api.event('motion_refinement_rejected', arm=arm, object_id=target, motion_phase=phase,
                  reason=type(exc).__name__+':'+str(exc), no_coarse_fallback=True, formal_credit=False,
                  original_vertices=original, refined_vertices=refined, mapping=mapping,
                  fresh_certificate=finite_json(certificate), caps=CAPS)
        raise


class ExecutionBudget:
    """ON-only API proxy, sharing the unchanged guards and governor.

    Counts successful commands and actual completed ticks, including a throwing
    post-step guard. 144 seconds is simulated nominal time, not wall time;
    existing continuation/supervisor wall deadlines remain authoritative.
    """
    def __init__(self, api, arm, target, phase, dense_count):
        self.api = api
        self.arm, self.target, self.phase = arm, target, phase
        self.dense_count = dense_count
        self.attempted = self.issued = self.tick_attempts = self.tick_returns = 0
        rb = api.telemetry()
        if type(rb['scene_step_count']) is not int or not math.isfinite(rb['scene_timestep_s']) or abs(rb['scene_timestep_s']-DT)>1e-9:
            reject('execution_clock')
        self.start = self.last = rb['scene_step_count']

    def __getattr__(self, name):
        return getattr(self.api, name)

    def check(self):
        now = self.api.telemetry()['scene_step_count']
        if type(now) is not int or now != self.last:reject('execution_tick_discontinuity')
        if now-self.start >= MAX_EXECUTION_TICKS:reject('total_execution_budget')
        if self.attempted >= MAX_EXECUTION_TICKS:reject('total_command_budget')
        return now

    def command(self, *args):
        self.check()
        self.attempted += 1
        result = self.api.command(*args)
        self.issued += 1
        return result

    def tick(self, *args):
        # The last allowed command must still be allowed its corresponding tick.
        before = self.api.telemetry()['scene_step_count']
        if before != self.last or before-self.start >= MAX_EXECUTION_TICKS:reject('total_execution_budget')
        self.tick_attempts += 1
        try:
            result = self.api.tick(*args)
            self.tick_returns += 1
        finally:
            self.last = self.api.telemetry()['scene_step_count']
        if type(self.last) is not int or self.last-before != 1:reject('execution_tick_discontinuity')
        return result

    def finish(self):
        self.api.event('motion_refinement_execution', arm=self.arm, object_id=self.target,
                       motion_phase=self.phase, dense_commands_including_initial=self.dense_count,
                       command_attempts=self.attempted, commands_issued=self.issued,
                       tick_attempts=self.tick_attempts, tick_returns=self.tick_returns,
                       completed_ticks=self.last-self.start, max_total_ticks=MAX_EXECUTION_TICKS,
                       formal_credit=False)
