"""Direct joint command validation and forward kinematics; no inverse solver."""
import math
import xml.etree.ElementTree as ET

ARMS = ('left', 'right')
COMMANDS = ('probe_q', 'joint_targets', 'gripper', 'magnet_on', 'magnet_off', 'wait', 'stop')


def eye():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def mul(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def axis_rotation(axis, angle):
    length = math.sqrt(sum(v*v for v in axis))
    x, y, z = [v/length for v in axis]
    c, s, d = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    a = eye()
    rows = [[c+x*x*d, x*y*d-z*s, x*z*d+y*s],
            [y*x*d+z*s, c+y*y*d, y*z*d-x*s],
            [z*x*d-y*s, z*y*d+x*s, c+z*z*d]]
    for i in range(3):
        a[i][:3] = rows[i]
    return a


def origin(element):
    a = eye()
    if element is None:
        return a
    r, p, y = map(float, element.get('rpy', '0 0 0').split())
    a = mul(mul(axis_rotation([0, 0, 1], y), axis_rotation([0, 1, 0], p)),
            axis_rotation([1, 0, 0], r))
    for i, v in enumerate(map(float, element.get('xyz', '0 0 0').split())):
        a[i][3] = v
    return a


class ForwardModel:
    def __init__(self, urdf):
        self.root = ET.parse(urdf).getroot()
        self.chain = [self.root.find(f"joint[@name='panda_joint{i}']") for i in range(1, 9)]
        self.chain.append(self.root.find("joint[@name='panda_hand_joint']"))
        if any(j is None for j in self.chain):
            raise ValueError('unsupported URDF')
        self.bounds = [[float(j.find('limit').get(k)) for k in ('lower', 'upper')]
                       for j in self.chain[:7]]

    def check_q(self, q):
        if not isinstance(q, list) or len(q) != 7 or any(type(x) not in (int, float) or not math.isfinite(x) for x in q):
            raise ValueError('seven finite joint angles required')
        if any(not lo+.03 <= v <= hi-.03 for v, (lo, hi) in zip(q, self.bounds)):
            raise ValueError('joint limit margin violated')

    def evaluate(self, arm, q):
        self.check_q(q)
        if arm not in ARMS:
            raise ValueError('invalid arm')
        t = axis_rotation([0, 0, 1], math.pi/2)
        t[0][3], t[1][3], t[2][3] = (-.55 if arm == 'left' else .55), -.55, .75
        links = []
        for i, j in enumerate(self.chain):
            t = mul(t, origin(j.find('origin')))
            if j.get('type') == 'revolute':
                t = mul(t, axis_rotation(list(map(float, j.find('axis').get('xyz').split())), q[i]))
            links.append(dict(name=j.find('child').get('link'), position=[t[k][3] for k in range(3)]))
        # SAPIEN's fixed joint frame contributes diag(1,-1,-1), cancelling
        # config global_trans_matrix. This matches the archived physical EE.
        transform = [[0., 0., 1., 0.], [0., -1., 0., 0.], [1., 0., 0., 0.], [0., 0., 0., 1.]]
        ee = mul(t, transform)
        for i in range(3):
            ee[i][3] -= .04*ee[i][0]
        tcp = [ee[i][3]+.12*ee[i][0] for i in range(3)]
        return dict(ee_position_m=[ee[i][3] for i in range(3)],
                    ee_rotation_world=[row[:3] for row in ee[:3]], tcp_position_m=tcp,
                    links=links, collision_checked=False)

    def link_transforms(self, arm, q, finger_q=.04):
        self.check_q(q)
        fingers=[float(finger_q)]*2 if isinstance(finger_q,(int,float)) else list(map(float,finger_q))
        if arm not in ARMS or len(fingers)!=2 or any(not math.isfinite(f) or not 0<=f<=.04 for f in fingers):
            raise ValueError('invalid arm or finger position')
        finger_positions=dict(zip(('panda_finger_joint1','panda_finger_joint2'),fingers))
        t = axis_rotation([0, 0, 1], math.pi/2)
        t[0][3], t[1][3], t[2][3] = (-.55 if arm == 'left' else .55), -.55, .75
        result = {'panda_link0': t}
        pending = list(self.root.findall('joint'))
        positions = {f'panda_joint{i+1}': v for i, v in enumerate(q)}
        for _ in range(len(pending)+1):
            if not pending:
                return result
            for j in pending[:]:
                parent = j.find('parent').get('link')
                if parent not in result:
                    continue
                t = mul(result[parent], origin(j.find('origin')))
                if j.get('type') == 'revolute':
                    t = mul(t, axis_rotation(list(map(float, j.find('axis').get('xyz').split())), positions[j.get('name')]))
                elif j.get('type') == 'prismatic':
                    translation = eye()
                    for i, v in enumerate(map(float, j.find('axis').get('xyz').split())):
                        translation[i][3] = v*finger_positions[j.get('name')]
                    t = mul(t, translation)
                elif j.get('type') != 'fixed':
                    raise ValueError('unsupported joint')
                result[j.find('child').get('link')] = t
                pending.remove(j)
        raise ValueError('disconnected URDF')


def validate(action, request, model):
    fields = {'session', 'seq', 'command', 'reason', 'arm', 'q', 'gripper', 'object_id', 'duration_s'}
    if not isinstance(action, dict) or set(action) != fields:
        raise ValueError('unexpected action fields')
    if action['session'] != request['session'] or type(action['seq']) is not int or action['seq'] != request['seq']:
        raise ValueError('stale response')
    c = action['command']
    if c not in COMMANDS or not isinstance(action['reason'], str) or not action['reason'].strip():
        raise ValueError('invalid command/reason')
    required = {'probe_q': {'arm', 'q'}, 'joint_targets': {'arm', 'q', 'object_id', 'duration_s'},
                'gripper': {'arm', 'gripper', 'object_id', 'duration_s'},
                'magnet_on': {'arm', 'object_id'}, 'magnet_off': {'arm'},
                'wait': {'duration_s'}, 'stop': set()}[c]
    for k in ('arm', 'q', 'gripper', 'object_id', 'duration_s'):
        if k not in required and action[k] is not None:
            raise ValueError(f'unused field {k} must be null')
    if 'arm' in required and action['arm'] not in ARMS:
        raise ValueError('invalid arm')
    if 'object_id' in required and (type(action['object_id']) is not int or str(action['object_id']) not in request['state']['objects']):
        raise ValueError('invalid target object')
    if 'duration_s' in required:
        d = action['duration_s']
        if type(d) not in (int, float) or not math.isfinite(d) or not .2 <= d <= 2.0:
            raise ValueError('duration outside .2..2 seconds')
    if 'q' in required:
        model.check_q(action['q'])
    if c == 'joint_targets':
        current = request['state']['robots'][action['arm']]['q'][:7]
        delta = max(abs(a-b) for a, b in zip(action['q'], current))
        if delta > .20+1e-9 or delta/action['duration_s'] > .4+1e-9:
            raise ValueError('joint increment/speed exceeds direct control bounds')
    if c == 'gripper' and (type(action['gripper']) not in (int, float) or not math.isfinite(action['gripper']) or not 0 <= action['gripper'] <= 1):
        raise ValueError('invalid gripper opening')
    return action


def schema(session, seq):
    p = {'session': {'type': 'string', 'enum': [session]}, 'seq': {'type': 'integer', 'enum': [seq]},
         'command': {'type': 'string', 'enum': list(COMMANDS)}, 'reason': {'type': 'string'},
         'arm': {'type': ['string', 'null'], 'enum': ['left', 'right', None]},
         'q': {'type': ['array', 'null'], 'items': {'type': 'number'}, 'minItems': 7, 'maxItems': 7},
         'gripper': {'type': ['number', 'null']}, 'object_id': {'type': ['integer', 'null']},
         'duration_s': {'type': ['number', 'null']}}
    return {'type': 'object', 'properties': p, 'required': list(p), 'additionalProperties': False}
