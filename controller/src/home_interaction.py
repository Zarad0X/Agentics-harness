"""HOME-only planning envelope from measured public collision-shape metadata.

No setters or simulation calls. Every protected object/robot link is included.
Old robot vertices are retained; measured cooked hull vertices are added so the
HOME model encloses both representations. No collision verdict is cached.
"""
import runtime_profile as timing
import copy
import math
import struct
import xml.etree.ElementTree as ET
import numpy as np
from direct_core import origin
from provenance import digest
from physics_geometry import convex_parts
from return_clearance import DT


def offset(value, name, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('home_contact_invalid_' + name)
    # Accept only exact float32 values; retain the original rejection path.
    # No metadata is cached: each caller still reads every live shape field.
    if type(value) is float and abs(value) <= 3.4028234663852886e38:
        if struct.unpack('=f', struct.pack('=f', value))[0] == value:
            if not positive or value >= 1.1754943508222875e-38:
                return float(value)
    with np.errstate(over='ignore', under='ignore'):
        f = np.float32(value)
    if not np.isfinite(f) or float(f) != value or (positive and value < np.finfo(np.float32).tiny):
        raise ValueError('home_contact_float32_or_underflow_' + name)
    return float(value)


def shape_signature(shape):
    # The actual immutable cooked shape is bound by its handle and geometry at
    # admission; the publicly mutable offsets, scale and pose are re-read live.
    c = offset(shape.get_contact_offset(), 'contact_offset', True)
    r = offset(shape.get_rest_offset(), 'rest_offset')
    if r >= c:
        raise ValueError('home_contact_rest_not_below_contact')
    pose = np.asarray(shape.get_local_pose().to_transformation_matrix(), float)
    scale = np.asarray(shape.get_scale(), float)
    if pose.shape != (4, 4) or scale.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError('home_contact_shape_transform')
    # Own this conversion for this call only; all live reads/checks remain above.
    values = scale.tolist()
    if (not (math.isfinite(values[0]) and math.isfinite(values[1]) and math.isfinite(values[2]))
            or values[0] <= 0 or values[1] <= 0 or values[2] <= 0):
        raise ValueError('home_contact_shape_transform')
    return dict(contact_offset_m=c, rest_offset_m=r, local_matrix=pose.tolist(), scale=values)


def model_binding(world):
    return digest(dict(urdf=ET.tostring(world.model.root, encoding='unicode'),
        geometry=world.geometry, robot_vertices=world.robot_vertices, meshes=world.meshes))


class ContactEnvelope:
    """Own immutable scalar mapping; validate explicit coverage, never defaults."""
    def __init__(self, record, world):
        self.record = copy.deepcopy(record)
        if record.get('schema') != 'home-contact-model/20' or record.get('model_sha256') != model_binding(world):
            raise ValueError('home_contact_model_binding')
        self.dt = offset(record['scene_timestep_s'], 'timestep', True)
        if type(self.dt) is not float or not math.isfinite(self.dt) or abs(self.dt - DT) > 1e-9:
            raise ValueError('home_contact_timestep')
        links = [r['link'] for r in world.geometry['robot_boxes']]
        if len(set(links)) != len(links) or set(record['arms']) != {'left', 'right'} or set(record['objects']) != set(world.meshes) or set(world.meshes) != set(world.state['objects']):
            raise ValueError('home_contact_coverage')
        self.links, self.objects = {}, {}
        self.vertices = {}
        for arm in ('left', 'right'):
            if set(record['arms'][arm]) != set(links):
                raise ValueError('home_contact_link_mapping:' + arm)
            self.links[arm], self.vertices[arm] = {}, {}
            for name in links:
                row = record['arms'][arm][name]
                self.links[arm][name] = max(self._shapes(row))
                vertices = np.asarray(row['union_vertices'], float)
                old = np.asarray(world.robot_vertices[name], float)
                if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < len(old)
                        or not np.isfinite(vertices).all() or not np.array_equal(vertices[:len(old)], old)):
                    raise ValueError('home_contact_robot_geometry_shrunk:' + arm + ':' + name)
                self.vertices[arm][name] = vertices.tolist()
        for oid, row in record['objects'].items():
            values = self._shapes(row)
            if len(values) != len(world.meshes[oid]) or row['geometry_sha256'] != digest(world.meshes[oid]) or row['actor_name'] != world.state['objects'][oid]['actor_name']:
                raise ValueError('home_contact_object_mapping:' + oid)
            if any(s['geometry_sha256'] != digest(p) for s,p in zip(row['shapes'],world.meshes[oid])):
                raise ValueError('home_contact_object_shape_geometry:' + oid)
            self.objects[oid] = values
        # Use union across both arms to preserve existing single-model geometry
        # interface, while retaining the distinct offset mappings for each arm.
        self.union_vertices = {n: self.vertices['left'][n] + self.vertices['right'][n][len(world.robot_vertices[n]):] for n in links}
        self.sha256 = digest(self.record)

    @staticmethod
    def _shapes(row):
        shapes = row['shapes']
        if type(shapes) is not list or not shapes or row['shape_count'] != len(shapes):
            raise ValueError('home_contact_shape_count')
        values = []
        for index, s in enumerate(shapes):
            if s['index'] != index or not isinstance(s['geometry_sha256'], str) or len(s['geometry_sha256']) != 64:
                raise ValueError('home_contact_shape_index_or_geometry')
            try:
                int(s['geometry_sha256'],16)
                matrix=np.asarray(s['local_matrix'],float);scale=np.asarray(s['scale'],float)
                valid=(matrix.shape==(4,4) and scale.shape==(3,) and np.isfinite(matrix).all()
                    and np.isfinite(scale).all() and np.all(scale>0) and np.array_equal(matrix[3],[0,0,0,1])
                    and np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),rtol=0,atol=2e-6))
            except (ValueError,KeyError,TypeError):valid=False
            if not valid:raise ValueError('home_contact_shape_transform_or_hash')
            c = offset(s['contact_offset_m'], 'contact_offset', True)
            r = offset(s['rest_offset_m'], 'rest_offset')
            if r >= c:
                raise ValueError('home_contact_rest_not_below_contact')
            values.append(c)
        return values

    def margin(self, arm, link, oid, part, old_margin):
        # Round outwards after a correctly rounded sum; tiny inputs cannot
        # disappear into the old margin. Existing reserves remain separate.
        required = math.fsum((old_margin, self.links[arm][link], self.objects[oid][part]))
        result = float(np.nextafter(required, math.inf))
        if not math.isfinite(result) or result <= old_margin:
            raise ValueError('home_contact_margin_overflow_or_underflow')
        return result

    def attach(self, world):
        if model_binding(world) != self.record['model_sha256']:
            raise ValueError('home_contact_model_changed')
        # A previously queried world must never reuse old robot hulls.
        world._shape_cache.clear()
        world.robot_vertices = self.union_vertices
        world.home_contact_envelope = self
        return world


class RuntimeEnvelope:
    def __init__(self, env, world):
        self.env = env
        self.live = []
        self.robot_links = {}
        self.object_actors = {}
        links = {r['link']: r for r in world.geometry['robot_boxes']}
        record = dict(schema='home-contact-model/20', model_sha256=model_binding(world),
            scene_timestep_s=float(env.scene.get_timestep()), arms={}, objects={}, construction='live_public_shape_getters')
        def read_body(body, expected_name):
            if body.entity.get_name() != expected_name:
                raise ValueError('home_contact_actor_name')
            shapes = list(body.get_collision_shapes())
            if not shapes:
                raise ValueError('home_contact_empty_shapes')
            signatures = [shape_signature(s) for s in shapes]
            parts = convex_parts(shapes)  # Unsupported/nonfinite geometry fails.
            self.live.append((body, expected_name, shapes, signatures))
            return dict(actor_name=expected_name, shape_count=len(shapes),
                shapes=[dict(index=i, geometry_sha256=digest(p), **s) for i, (p, s) in enumerate(zip(parts, signatures))],
                geometry_sha256=digest(parts)), parts
        for arm in ('left', 'right'):
            actual = [l for l in getattr(env.robot, arm + '_entity').get_links() if l.get_collision_shapes()]
            names = [l.entity.get_name() for l in actual]
            if len(names) != len(set(names)) or set(names) != set(links):
                raise ValueError('home_contact_runtime_link_coverage:' + arm)
            self.robot_links[arm] = actual
            record['arms'][arm] = {}
            for body in actual:
                name = body.entity.get_name()
                row, parts = read_body(body, name)
                transform = np.asarray(origin(ET.Element('origin', links[name]['origin'])))
                points = np.concatenate([np.asarray(p['vertices']) for p in parts])
                in_collision_frame = (points - transform[:3, 3]) @ transform[:3, :3]
                row['union_vertices'] = np.concatenate((np.asarray(world.robot_vertices[name]), in_collision_frame)).tolist()
                record['arms'][arm][name] = row
        if set(map(str, env.objects)) != set(world.meshes):
            raise ValueError('home_contact_runtime_object_coverage')
        bodies = list(env.scene.physx_system.get_rigid_dynamic_components())
        for oid in world.meshes:
            matches = [b for b in bodies if b.entity == env.objects[int(oid)].actor]
            if len(matches) != 1:
                raise ValueError('home_contact_runtime_object_identity:' + oid)
            row, parts = read_body(matches[0], world.state['objects'][oid]['actor_name'])
            if parts != world.meshes[oid]:
                raise ValueError('home_contact_cooked_object_geometry:' + oid)
            self.object_actors[oid] = matches[0].entity
            record['objects'][oid] = row
        self.envelope = ContactEnvelope(record, world)

    @timing.measured('home_metadata')
    def check(self):
        if float(self.env.scene.get_timestep()) != self.envelope.dt:
            raise ValueError('home_contact_timestep_changed')
        for arm,links in self.robot_links.items():
            current=[l for l in getattr(self.env.robot,arm+'_entity').get_links() if l.get_collision_shapes()]
            if current!=links:raise ValueError('home_contact_robot_identity_changed:'+arm)
        if set(map(str,self.env.objects))!=set(self.object_actors) or any(self.env.objects[int(oid)].actor!=actor for oid,actor in self.object_actors.items()):
            raise ValueError('home_contact_object_identity_changed')
        for body, name, shapes, signatures in self.live:
            current = list(body.get_collision_shapes())
            if body.entity.get_name() != name or len(current) != len(shapes) or any(a != b for a, b in zip(current, shapes)):
                raise ValueError('home_contact_shape_mapping_changed:' + name)
            if [shape_signature(s) for s in current] != signatures:
                raise ValueError('home_contact_metadata_changed:' + name)


class HomeInteractionAPI:
    def __init__(self, api, env, arm, target):
        self.api = api
        state = api.state()
        self.runtime = RuntimeEnvelope(env, api.world(state))
        api.event('home_contact_model', schema='home-contact-model-receipt/20', arm=arm,
            object_id=target, motion_phase='home', scene_step=api.telemetry()['scene_step_count'],
            state=state, state_sha256=digest(state), controls_sha256=digest(api.controls()),
            rng=copy.deepcopy(api.rng.bit_generator.state), rng_sha256=digest(api.rng.bit_generator.state),
            model=self.runtime.envelope.record, model_sha256=self.runtime.envelope.sha256,
            reserves='existing 1mm + actual offset sum; tracking/step/world/interval reserves retained',
            search_caps=dict(rrt_attempts=6,nodes_per_attempt=80,point_checks_per_attempt=1200,certificate_nodes=4096),
            parked_additional_bounds=dict(link_displacement_m=.0005,joint_speed_rad_s=.01,finger_speed_m_s=.001),
            scope='HOME only; both arms versus every protected object; loaded/RETREAT/handoff unchanged', formal_credit=False)

    def __getattr__(self, name):
        return getattr(self.api, name)

    def state(self):
        self.runtime.check()
        return self.api.state()

    def world(self, state):
        self.runtime.check()
        return self.runtime.envelope.attach(self.api.world(state))

    def command(self, *args):
        self.runtime.check()
        return self.api.command(*args)
