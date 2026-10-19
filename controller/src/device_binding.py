"""Explicit SAPIEN3 renderer assignment, without changing release files.

The legacy wrapper's SapienRenderer(**args) discards device arguments. Both
the renderer and each independent scene therefore need explicit constructors.
This module imports no GPU package; install is called only after resource gates.
"""
import re


def pci_id(value):
    match=re.fullmatch(r'(?:pci:)?([0-9a-fA-F]+):([0-9a-fA-F]{2}):([0-9a-fA-F]{2})\.([0-7])',str(value))
    if not match:raise RuntimeError('unrecognized_GPU_PCI_identity:'+str(value))
    return tuple(int(v,16) for v in match.groups())


def install(sapien,expected_pci):
    # CUDA_VISIBLE_DEVICES has already been set to the assigned GPU UUID.
    device=sapien.Device('cuda:0')
    if pci_id(device.pci_string)!=pci_id(expected_pci) or not device.can_render():
        raise RuntimeError('assigned_render_device_mismatch')
    legacy_engine=sapien.Engine
    legacy_renderer=sapien.SapienRenderer

    class AssignedRenderer(legacy_renderer):
        def __init__(self,**kwargs):
            # Bypass the wrapper __init__ that silently ignores device args;
            # retain its create_material compatibility method.
            sapien.pysapien.render.SapienRenderer.__init__(self,device)

    class AssignedEngine(legacy_engine):
        def create_scene(self,config=None):
            if config is not None:sapien.physx.set_scene_config(config)
            render=sapien.render.RenderSystem(device)
            if pci_id(render.device.pci_string)!=pci_id(expected_pci):
                raise RuntimeError('scene_renderer_device_mismatch')
            return sapien.Scene([sapien.physx.PhysxCpuSystem(),render])

    sapien.SapienRenderer=AssignedRenderer;sapien.Engine=AssignedEngine
    return dict(cuda_alias='cuda:0',pci_string=device.pci_string,name=device.name,
                physics='independent_PhysxCpuSystem',renderer_binding='explicit_SAPIEN3_device')
