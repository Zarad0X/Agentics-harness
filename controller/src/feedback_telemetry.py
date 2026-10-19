"""Best-effort read-only public SAPIEN metadata; never imports a simulator.

Getter names inspected in installed sapien/pysapien/physx.pyi. Missing runtime
APIs stay unavailable. No setters, force computation, or physics probes.
"""
import hashlib
import importlib.metadata
import sys
from pathlib import Path
import numpy as np


def numeric(value):
    if hasattr(value,'p') and hasattr(value,'q'):return dict(p=numeric(value.p),q=numeric(value.q))
    if isinstance(value,np.ndarray):return numeric(value.tolist())
    if isinstance(value,(list,tuple)):return [numeric(v) for v in value]
    if isinstance(value,(str,int,bool)) or value is None:return value
    if isinstance(value,(float,np.number)):
        if not np.isfinite(value):raise ValueError('nonfinite getter value')
        return float(value)
    raise TypeError('unsupported public getter result')


def get(obj,name):
    try:return dict(available=True,value=numeric(getattr(obj,name)()))
    except Exception as exc:return dict(available=False,reason=name+':'+type(exc).__name__+':'+str(exc))


_METADATA={}

def metadata(env):
    from provenance import digest
    identity=id(env.scene)
    if identity in _METADATA and _METADATA[identity][0] is env.scene:
        return dict(metadata_already_recorded=True,metadata_sha256=_METADATA[identity][1])
    packages={}
    for name in ('sapien','sapien-sim','physx'):
        try:packages[name]=dict(available=True,version=importlib.metadata.version(name))
        except importlib.metadata.PackageNotFoundError:packages[name]=dict(available=False,reason='No distribution metadata')
    # Exact loaded extension identity, no account or environment scans.
    modules={}
    for name,module in tuple(sys.modules.items()):
        if not (name=='sapien' or name.startswith('sapien.')):continue
        file=getattr(module,'__file__',None)
        if file and Path(file).is_file() and (name=='sapien' or str(file).endswith('.so')):
            p=Path(file);h=hashlib.sha256()
            with p.open('rb') as f:
                for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
            modules[name]=dict(file=str(p),sha256=h.hexdigest(),version=str(getattr(module,'__version__','unavailable')))
    physx=sys.modules.get('sapien.physx')
    arms={}
    for arm in ('left','right'):
        entity=getattr(env.robot,arm+'_entity');links=[];joints=[]
        try:
            for link in entity.get_links():
                links.append(dict(name=get(link,'get_name'),mass=get(link,'get_mass'),inertia=get(link,'get_inertia'),
                    center_of_mass_local_pose=get(link,'get_cmass_local_pose'),linear_damping=get(link,'get_linear_damping'),angular_damping=get(link,'get_angular_damping')))
        except Exception as exc:links=dict(available=False,reason=type(exc).__name__+':'+str(exc))
        try:
            for j in entity.get_active_joints():
                joints.append({n:get(j,'get_'+n) for n in ('name','damping','friction','stiffness','force_limit','drive_mode')})
        except Exception as exc:joints=dict(available=False,reason=type(exc).__name__+':'+str(exc))
        arms[arm]=dict(name=get(entity,'get_name'),python_type=type(entity).__module__+'.'+type(entity).__name__,
            solver_position_iterations=get(entity,'get_solver_position_iterations'),solver_velocity_iterations=get(entity,'get_solver_velocity_iterations'),links=links,joints=joints)
    result=dict(schema='read-only-actuator-metadata/16',packages=packages,loaded_modules=modules,
        physx_version=get(physx,'version'),scene_timestep=get(env.scene,'get_timestep'),articulations=arms,
        measured_drive_force=dict(available=False,reason='No identified public measured drive force getter in inspected API'),
        constraint_impulse=dict(available=False,reason='Joint solver impulse not identified; existing contact impulses retain contact semantics'),
        formal_credit=False)
    _METADATA[identity]=(env.scene,digest(result))
    return result


def packet(api,arm):
    from velocity_feedback import targets,vector
    s=api.state();rb=api.telemetry();q,v=targets(api.controls(),arm)
    return dict(frame=rb['scene_step_count'],dt=rb['scene_timestep_s'],q=vector(s['robots'][arm]['q'],9).tolist(),
        native_qvel=vector(s['robots'][arm]['qdot'],9).tolist(),readback_q=q.tolist(),readback_qdot=v.tolist(),
        external_qf=rb.get('applied_passive_qf'))


def pair(pre,post,q,v):
    return dict(pre=pre,post=post,just_issued_q=list(q),just_issued_qdot=list(v),
        single_tick_fd=((np.asarray(post['q'])-pre['q'])/.004).tolist() if post['frame']==pre['frame']+1 else None)
