"""Independent numerical planning using only Python and NumPy.

Malformed inputs raise ValueError. Search failures return success=False.
IK concerns task EE pose, not collision or grasp success. Joint planning checks
sampled configurations only. No physics, force model, or continuous collision proof.
"""
import math
import numpy as np

POSITION_TOLERANCE_M = 1e-4
ROTATION_TOLERANCE_RAD = 1e-3
JOINT_MARGIN_RAD = 0.03
TCP_OFFSET_M = 0.12
MAX_COLLISION_CHECKS = 200000
MAX_RESAMPLED_POINTS = 1000000


def _array(value, name):
    try:
        a = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(name + ' must be numeric') from exc
    if not np.all(np.isfinite(a)):
        raise ValueError(name + ' must be finite')
    return a.copy()


def _vector(value, name, size=None):
    a = _array(value, name)
    if a.ndim != 1 or not a.size or (size is not None and a.size != size):
        raise ValueError(name + ' has invalid dimensions')
    return a


def _bounds(value):
    b = _array(value, 'bounds')
    if b.ndim != 2 or b.shape[1] != 2 or not len(b):
        raise ValueError('bounds must have shape (n, 2)')
    with np.errstate(over='ignore'):
        widths = b[:, 1] - b[:, 0]
    if np.any(widths <= 0) or not np.all(np.isfinite(widths)):
        raise ValueError('bounds must have finite positive widths')
    return b


def _inside(q, b):
    return bool(np.all(q >= b[:, 0]) and np.all(q <= b[:, 1]))


def _positive(value, name):
    if isinstance(value, (bool, np.bool_)) or not np.isscalar(value):
        raise ValueError(name + ' must be a positive scalar')
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(name + ' must be a positive scalar') from exc
    if not math.isfinite(value) or value <= 0:
        raise ValueError(name + ' must be finite and positive')
    return value


def _integer(value, name, minimum, maximum):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise ValueError(name + ' must be an integer')
    if not minimum <= value <= maximum:
        raise ValueError(name + ' outside supported resource limits')
    return int(value)


def _rotation(value):
    r = _array(value, 'rotation')
    if r.shape != (3, 3):
        raise ValueError('rotation must have shape (3, 3)')
    if (not np.allclose(r.T @ r, np.eye(3), atol=1e-6, rtol=0)
            or abs(np.linalg.det(r) - 1.0) > 1e-6):
        raise ValueError('rotation must be a proper orthonormal matrix')
    return r


def so3_log(rotation):
    """Principal rotation vector; the sign at exactly pi is deterministic.

    atan2 retains precision near zero and pi. Near pi, the symmetric part's
    unit eigenvector with largest eigenvalue supplies the rotation axis.
    """
    r = _rotation(rotation)
    skew = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0],
                     r[1, 0] - r[0, 1]]) * 0.5
    sine = float(np.linalg.norm(skew))
    cosine = float(np.clip((np.trace(r) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.atan2(sine, cosine)
    if angle < 1e-7:
        return skew * (1.0 + angle * angle / 6.0)
    if math.pi - angle < 1e-5:
        _, axes = np.linalg.eigh((r + r.T) * 0.5)
        axis = axes[:, -1]
        if sine > 1e-12:
            if np.dot(axis, skew) < 0:
                axis = -axis
        elif axis[np.argmax(np.abs(axis))] < 0:
            axis = -axis
        return angle * axis
    return (angle / sine) * skew


def solve_ik(model, arm, target_position, target_rotation, q_seed, *, rng,
             max_iters=100, restarts=4):
    """Bounded numerical DLS IK with backtracking and caller-owned RNG.

    restarts counts additional uniform starts after q_seed. Each start gets
    max_iters updates. iterations counts attempted updates across all starts.
    Uses model.bounds with a 0.03 rad inward margin, and unmodified task EE FK.
    Failure means no solution found within budget, not proof of unreachability.
    """
    max_iters = _integer(max_iters, 'max_iters', 0, 10000)
    restarts = _integer(restarts, 'restarts', 0, 100)
    if arm not in ('left', 'right'):
        raise ValueError('invalid arm')
    b = _bounds(model.bounds)
    b[:, 0] += JOINT_MARGIN_RAD
    b[:, 1] -= JOINT_MARGIN_RAD
    b = _bounds(b)
    seed = _vector(q_seed, 'q_seed', len(b))
    if not _inside(seed, b):
        raise ValueError('q_seed violates joint limit margin')
    target_p = _vector(target_position, 'target_position', 3)
    target_r = _rotation(target_rotation)
    if not callable(getattr(rng, 'uniform', None)):
        raise ValueError('rng must provide uniform(low, high)')
    weights = np.array([1., 1., 1., .3, .3, .3])
    best = None
    iterations = 0

    def fk(q):
        pose = model.evaluate(arm, q.tolist())
        return (_vector(pose['ee_position_m'], 'FK position', 3),
                _rotation(pose['ee_rotation_world']))

    def residual(p, r):
        return np.concatenate((target_p - p, so3_log(target_r @ r.T)))

    def result(success):
        _, q, pe, re = best
        return dict(success=success, q=q.tolist(), position_error_m=pe,
                    rotation_error_rad=re, iterations=iterations,
                    reason='converged' if success else 'no_convergence_within_budget')

    for attempt in range(restarts + 1):
        q = seed.copy() if attempt == 0 else _vector(
            rng.uniform(b[:, 0], b[:, 1]), 'random start', len(b))
        if not _inside(q, b):
            raise ValueError('rng produced an out-of-bounds start')
        for update in range(max_iters + 1):
            p, r = fk(q)
            e = residual(p, r)
            pe, re = float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]))
            cost = float(np.dot(weights * e, weights * e))
            if best is None or cost < best[0]:
                best = (cost, q.copy(), pe, re)
            if pe <= POSITION_TOLERANCE_M and re <= ROTATION_TOLERANCE_RAD:
                # Native-representable goal BEFORE pose acceptance, geometry,
                # path certification or physical preview. Never clip a route.
                from actuator_targets import goal
                native_q = goal(q, b)
                native_p, native_r = fk(native_q)
                native_e = residual(native_p, native_r)
                native_pe = float(np.linalg.norm(native_e[:3]))
                native_re = float(np.linalg.norm(native_e[3:]))
                if native_pe <= POSITION_TOLERANCE_M and native_re <= ROTATION_TOLERANCE_RAD:
                    best = (float(np.dot(weights * native_e, weights * native_e)), native_q, native_pe, native_re)
                    return result(True)
            if update == max_iters:
                break
            iterations += 1
            jac = np.empty((6, len(q)))
            for j in range(len(q)):
                h = min(1e-5, (b[j, 1] - b[j, 0]) / 4)
                qm, qp = q.copy(), q.copy()
                qm[j] = max(b[j, 0], q[j] - h)
                qp[j] = min(b[j, 1], q[j] + h)
                denominator = qp[j] - qm[j]
                if denominator <= 0:
                    raise ValueError('joint interval below numerical resolution')
                pm, rm = fk(qm)
                pp, rp = fk(qp)
                jac[:3, j] = (pp - pm) / denominator
                jac[3:, j] = so3_log(rp @ rm.T) / denominator
            a = weights[:, None] * jac
            accepted = False
            for damping in (.01, .03, .1):
                inverse = a.T @ np.linalg.solve(a @ a.T + damping**2 * np.eye(6), np.eye(6))
                dq = inverse @ (weights * e)
                # Redundancy should move joints away from limits instead of
                # clipping the wrist until the pose solver stalls. Project a
                # bounded joint-centering gradient into the numerical nullspace.
                center = (b[:,0]+b[:,1])/2
                gradient = .12*(center-q)/(b[:,1]-b[:,0])
                gradient += .001*(1/(q-b[:,0]+.05)**2-1/(b[:,1]-q+.05)**2)
                dq += (np.eye(len(q))-inverse@a) @ np.clip(gradient,-.15,.15)
                largest = float(np.max(np.abs(dq)))
                if largest > .18:
                    dq *= .18 / largest
                for k in range(8):
                    trial = np.clip(q + (0.5**k) * dq, b[:, 0], b[:, 1])
                    tp, tr = fk(trial)
                    te = residual(tp, tr)
                    trial_cost = float(np.dot(weights * te, weights * te))
                    if trial_cost < cost - 1e-15:
                        q = trial
                        accepted = True
                        break
                if accepted:
                    break
            if not accepted:
                break
    return result(False)


def solve_ik_sequence(model, arm, positions, rotation, q_seed, *, rng,
                      max_iters=120):
    """Require one locally continuous IK branch for all ordered waypoints.

    Uniform restarts can find an endpoint on a different elbow/wrist branch;
    that does not certify a lift from a closed grasp. Collision checking is
    still the caller's responsibility, including between returned solutions.
    """
    solutions=[]
    seed=q_seed
    for index,p in enumerate(positions):
        ik=solve_ik(model,arm,p,rotation,seed,rng=rng,
                    max_iters=max_iters,restarts=0)
        if not ik['success']:
            return dict(success=False,solutions=solutions,failed_index=index,
                        reason='waypoint_ik_failed',ik=ik)
        if np.max(np.abs(np.asarray(ik['q'])-seed))>.5:
            return dict(success=False,solutions=solutions,failed_index=index,
                        reason='joint_branch_jump',ik=ik)
        solutions.append(ik['q']);seed=ik['q']
    return dict(success=True,solutions=solutions,failed_index=None,reason='converged')


def grasp_candidates(object_pose, local_center, extents, max_aperture=.08):
    """Deterministic contact hypotheses from a current local-to-world 4x4 pose.

    extents are full positive OBB side lengths; local_center is in object space.
    Sample every upward-facing OBB face at a 3x3 grid and eight horizontal yaws.
    EE +x points world-down, EE +y closes the fingers, TCP=EE+0.12*EE_x.
    Each TCP is placed on a sampled surface point, without penetration allowance.

    whole_obb_fits reports only whole-OBB projection fit along the closing axis.
    All samples remain assisted_contact_hypothesis=True, including wide objects.
    Neither flag certifies graspability, force closure, collision clearance,
    IK reachability, magnetic holding force, or a successful physical contact.
    """
    pose = _array(object_pose, 'object_pose')
    if pose.shape != (4, 4) or not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-9, rtol=0):
        raise ValueError('object_pose must be a homogeneous 4x4 transform')
    rotation = _rotation(pose[:3, :3])
    center = _vector(local_center, 'local_center', 3)
    sizes = _vector(extents, 'extents', 3)
    if np.any(sizes <= 0):
        raise ValueError('extents must be positive full side lengths')
    aperture = _positive(max_aperture, 'max_aperture')
    approach = np.array([0., 0., -1.])
    candidates = []
    for face_axis in range(3):
        tangent = [i for i in range(3) if i != face_axis]
        for sign in (-1., 1.):
            if sign * rotation[2, face_axis] <= 1e-10:
                continue
            for u in (-.25, 0., .25):
                for v in (-.25, 0., .25):
                    contact_local = center.copy()
                    contact_local[face_axis] += sign * sizes[face_axis] / 2
                    contact_local[tangent[0]] += u * sizes[tangent[0]]
                    contact_local[tangent[1]] += v * sizes[tangent[1]]
                    contact = pose[:3, 3] + rotation @ contact_local
                    for k in range(8):
                        yaw = k * math.pi / 8
                        closing = np.array([math.cos(yaw), math.sin(yaw), 0.])
                        ee_r = np.column_stack((approach, closing, np.cross(approach, closing)))
                        width = float(np.dot(sizes, np.abs(rotation.T @ closing)))
                        candidates.append(dict(
                            ee_position_m=(contact - TCP_OFFSET_M * approach).tolist(),
                            ee_rotation_world=ee_r.tolist(), closing_width_m=width,
                            whole_obb_fits=bool(width <= aperture),
                            assisted_contact_hypothesis=True))
    return candidates


class _SearchBudget(Exception):
    pass


def plan_joint_path(start, goal, bounds, is_free, *, rng, max_nodes=1500, resolution=.03):
    """Seeded bidirectional RRT-Connect with bounded work.

    bounds are the caller's effective limits: pass model.bounds narrowed by
    0.03 rad for ForwardModel. Resolution is maximum per-joint sample spacing.
    Every returned edge includes checks at both endpoints. Predicates receive
    fresh Python lists and must return bool. No collision exemptions are applied.
    Resource caps: max_nodes total tree vertices, max_nodes outer attempts, and
    MAX_COLLISION_CHECKS predicate calls. Steering uses at most 0.2 rad per joint.
    """
    b = _bounds(bounds)
    s = _vector(start, 'start', len(b))
    g = _vector(goal, 'goal', len(b))
    if not _inside(s, b) or not _inside(g, b):
        raise ValueError('start or goal violates bounds')
    resolution = _positive(resolution, 'resolution')
    max_nodes = _integer(max_nodes, 'max_nodes', 2, 100000)
    if not callable(is_free):
        raise ValueError('is_free must be callable')
    if not callable(getattr(rng, 'uniform', None)) or not callable(getattr(rng, 'random', None)):
        raise ValueError('rng must provide uniform and random')
    checks, nodes = 0, 2

    def finish(success, path, reason):
        return dict(success=success, path=[q.tolist() for q in path],
                    reason=reason, collision_checks=checks)

    def free(q):
        nonlocal checks
        if checks >= MAX_COLLISION_CHECKS:
            raise _SearchBudget('collision_check_budget')
        checks += 1
        answer = is_free(q.tolist())
        if not isinstance(answer, (bool, np.bool_)):
            raise ValueError('is_free must return bool')
        return bool(answer)

    def edge(a, z):
        distance = float(np.max(np.abs(z - a)))
        ratio = distance / resolution
        if not math.isfinite(ratio) or ratio > MAX_COLLISION_CHECKS:
            raise _SearchBudget('collision_check_budget')
        n = max(1, math.ceil(ratio))
        if n + 1 > MAX_COLLISION_CHECKS - checks:
            raise _SearchBudget('collision_check_budget')
        for i in range(n + 1):
            q = a if i == 0 else z if i == n else a + (z - a) * (i / n)
            if not free(q):
                return False
        return True

    def extend(tree, target):
        nonlocal nodes
        index = min(range(len(tree['q'])),
                    key=lambda j: float(np.max(np.abs(tree['q'][j] - target))))
        q = tree['q'][index]
        if np.array_equal(q, target):
            return index, 'reached'
        if nodes >= max_nodes:
            raise _SearchBudget('node_budget')
        distance = float(np.max(np.abs(target - q)))
        new = target.copy() if distance <= .2 else q + (target - q) * (.2 / distance)
        new = np.clip(new, b[:, 0], b[:, 1])
        if np.array_equal(q, new) or not edge(q, new):
            return None, 'trapped'
        tree['q'].append(new)
        tree['parent'].append(index)
        nodes += 1
        return len(tree['q']) - 1, 'reached' if np.array_equal(new, target) else 'advanced'

    def trace(tree, index):
        path = []
        for _ in range(max_nodes):
            path.append(tree['q'][index])
            index = tree['parent'][index]
            if index == -1:
                return path[::-1]
        raise RuntimeError('invalid tree parent chain')

    try:
        if not free(s):
            return finish(False, [], 'blocked_start')
        if not free(g):
            return finish(False, [], 'blocked_goal')
        if np.array_equal(s, g):
            return finish(True, [s], 'sampled_path_found')
        if edge(s, g):
            return finish(True, [s, g], 'sampled_path_found')
        ta = dict(q=[s], parent=[-1], from_start=True)
        tb = dict(q=[g], parent=[-1], from_start=False)
        for _ in range(max_nodes):
            target = tb['q'][0] if rng.random() < .15 else _vector(
                rng.uniform(b[:, 0], b[:, 1]), 'random sample', len(b))
            if not _inside(target, b):
                raise ValueError('rng produced an out-of-bounds sample')
            ia, status = extend(ta, target)
            if status != 'trapped':
                meeting = ta['q'][ia]
                for _ in range(max_nodes):
                    ib, status = extend(tb, meeting)
                    if status == 'trapped':
                        break
                    if status == 'reached':
                        pa, pb = trace(ta, ia), trace(tb, ib)
                        path = pa + pb[-2::-1] if ta['from_start'] else pb + pa[-2::-1]
                        return finish(True, path, 'sampled_path_found')
            ta, tb = tb, ta
        return finish(False, [], 'iteration_budget')
    except _SearchBudget as exc:
        return finish(False, [], str(exc))


def resample_path(path, max_velocity=.4, dt=.004, max_acceleration=None):
    """Piecewise-linear samples spaced dt apart, including all original vertices.

    max_velocity is a scalar rad/s cap on every joint. Each segment takes an
    integer number of time steps, rounding duration upward. If acceleration is
    supplied, a quintic scalar clock gives zero velocity/acceleration at each
    vertex and bounds commanded joint acceleration without changing geometry.
    This does not guarantee physical tracking or collision freedom.
    """
    velocity = _positive(max_velocity, 'max_velocity')
    dt = _positive(dt, 'dt')
    acceleration=None if max_acceleration is None else _positive(max_acceleration,'max_acceleration')
    step = velocity * dt
    if not math.isfinite(step) or step <= 0:
        raise ValueError('max_velocity * dt is not representable')
    p = _array(path, 'path')
    if p.ndim != 2 or not p.shape[0] or not p.shape[1]:
        raise ValueError('path must have nonempty shape (samples, joints)')
    if len(p) > MAX_RESAMPLED_POINTS:
        raise ValueError('resampling point budget exceeded')
    intervals, total = [], 1
    for a, z in zip(p[:-1], p[1:]):
        with np.errstate(over='ignore'):
            distance = float(np.max(np.abs(z - a)))
        ratio = distance / step if acceleration is None else max(
            1.875*distance/velocity,math.sqrt(5.78*distance/acceleration))/dt
        if not math.isfinite(ratio) or ratio > MAX_RESAMPLED_POINTS:
            raise ValueError('resampling point budget exceeded')
        n = max(1, math.ceil(ratio))
        total += n
        if total > MAX_RESAMPLED_POINTS:
            raise ValueError('resampling point budget exceeded')
        intervals.append(n)
    out = [p[0].tolist()]
    for a, z, n in zip(p[:-1], p[1:], intervals):
        for i in range(1, n + 1):
            u=i/n
            clock=u if acceleration is None else u**3*(10-15*u+6*u*u)
            out.append((z if i == n else a + (z - a) * clock).tolist())
    return out
