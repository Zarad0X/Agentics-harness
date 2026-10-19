"""Bind the reviewed endpoint theorem to unchanged execution/static assumptions.

Checks run before GPU acquisition; receipts do not relax any execution check.
"""
import ast
import hashlib
from pathlib import Path
import numpy as np
from provenance import digest,file_hash

# 006 adds only an opt-in stronger unloaded retreat executor. Removing that
# dispatch and its default-false argument reproduces the exact prior plan_to AST.
# IK, path planning, endpoint thresholds and loaded call semantics are unchanged.
FUNCTION_AST = {('physics_runner.py', 'plan_to'): 'f950cccebcb087106645c29daf644fba0dd3fd898b9236a44b9d1aef97400b89', ('mesh_world.py', 'free'): 'c1e4af48dc7f9d1d2e5edbf58d139aa0bbb3c3d610bc307f19482cdc791a71ff', ('collision_world.py', '__init__'): '9d3d01458388927e52698879c0814b57d102d0ab460159abd0cb454e1ba9b506'}
# 008 additionally pins provenance capture and its complete placement caller.
FUNCTION_AST.update({('physics_runner.py', 'main'): '70b47b0f5ece3aab1fa708769f98f5e6b03f324229739227c6319b8bce35b2e7', ('placement_contract.py', 'endpoint_receipt'): 'c917fda0ac8f0bd5fbf91a0441573535579e3660a5c86bd67b7be116281c3dc5'})
HELPER_SHA256 = '46c29be0b13f3e12cb51fec693cc7e261ead81dd646c9b8353afb7224503c64e'


def validate_contract(root):
    root=Path(root)
    for (filename,name),expected in FUNCTION_AST.items():
        tree=ast.parse((root/'src'/filename).read_text())
        node=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name)
        if hashlib.sha256(ast.dump(node,include_attributes=False).encode()).hexdigest()!=expected:
            raise ValueError('placement_endpoint_assumption_drift:'+filename+':'+name)
    if file_hash(root/'src/placement_endpoint.py')!=HELPER_SHA256:
        raise ValueError('placement_endpoint_reviewed_helper_drift')
    # Bind imported IK literals as well as the reviewed execution 5mm/.04rad
    # and MeshWorld 1mm constants protected by exact function AST above.
    values={}
    for node in ast.parse((root/'src/astra_planner.py').read_text()).body:
        if isinstance(node,ast.Assign):
            for target in node.targets:
                if isinstance(target,ast.Name) and target.id in ('POSITION_TOLERANCE_M','ROTATION_TOLERANCE_RAD'):
                    values[target.id]=ast.literal_eval(node.value)
    if values!={'POSITION_TOLERANCE_M':1e-4,'ROTATION_TOLERANCE_RAD':1e-3}:
        raise ValueError('placement_endpoint_IK_assumption_drift')
    return dict(execution_position_m=.005,execution_rotation_rad=.04,static_margin_m=.001,IK=values,
                helper_sha256=HELPER_SHA256,functions={f+':'+n:h for (f,n),h in FUNCTION_AST.items()})


def geometry_binding(world):
    # Pure existing geometry; no native shape getters or extra collision query.
    return dict(static_shapes=[dict(name=n,vertices_sha256=digest(v[1].tolist())) for n,v in world.static_shapes],
                link7_geometry_sha256=digest(dict(boxes=[r for r in world.geometry['robot_boxes'] if r['link']=='panda_link7'],
                    vertices=world.robot_vertices['panda_link7'])))


def endpoint_receipt(world,arm,q,position,rotation,target,slot,result,binding,above):
    if world.held is None or world.held[:2]!=(arm,str(target)):
        raise ValueError('placement_source_held_identity')
    source=dict(schema='placement-source/008',above=np.asarray(above).tolist(),
        held_relative=np.asarray(world.held[2]).tolist(),
        physical_parts_sha256=digest(world.meshes[str(target)]))
    row=dict(placement_source=source,schema='placement-endpoint-admission/30',object_id=int(target),arm=arm,
             endpoint_q=np.asarray(q).tolist(),desired_position=np.asarray(position).tolist(),
             desired_rotation=np.asarray(rotation).tolist(),slot=list(slot),
             state=world.state,state_sha256=digest(world.state),geometry=geometry_binding(world),
             binding=binding,assessment=result,physical_completion_proved=False)
    return dict(row,sha256=digest(row))
