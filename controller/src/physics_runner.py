"""Experimental custom-planner Real10 runner. Requires an idle GPU for SAPIEN.

Existing task code supplies scene/URDF/actuators/magnet only. All candidate, IK
and joint-path generation is routed to astra_planner, with forbidden old APIs.
This runner is independent of the historical success verifier and records its
own diagnostic result; a zero Python exit is not a ten-object success claim.
"""
import runtime_profile as timing
import argparse
import ast
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import types
import traceback
from preview_errors import CandidateInfeasible,exception_diagnostic


def gpu_gate(index):
    p=subprocess.run(['nvidia-smi',f'--id={index}',
        '--query-gpu=uuid,pci.bus_id,memory.used,memory.total,utilization.gpu','--format=csv,noheader,nounits'],
        capture_output=True,text=True,check=True)
    uuid,pci,used,total,util=[v.strip() for v in p.stdout.strip().split(',')]
    used,total,util=map(int,(used,total,util))
    if used>1024 or total-used<12000 or util>10:
        raise RuntimeError(f'GPU {index} busy: used={used}MiB utilization={util}%')
    return dict(index=index,uuid=uuid,pci_bus_id=pci,used_mib=used,total_mib=total,utilization_percent=util)


@timing.session
def main(diagnostic=None,continuous=False,continuous_version=23,visual=None,loaded_variant='B_clearance'):
    if loaded_variant not in ('A_original','B_clearance'):raise ValueError('loaded_variant')
    ap=argparse.ArgumentParser()
    ap.add_argument('--release',required=True,type=Path)
    ap.add_argument('--output',required=True,type=Path)
    ap.add_argument('--output-root',type=Path,default=Path('/data4/yuantao/robotwin-real10-gpt6-astra-20260928/outputs'))
    ap.add_argument('--gpu',type=int,default=0)
    ap.add_argument('--config',required=True,type=Path)
    ap.add_argument('--experience-snapshot',type=Path)
    ap.add_argument('--expected-layout',type=Path)
    ap.add_argument('--check-only',action='store_true')
    args=ap.parse_args()
    from run_config import load_config
    cfg=load_config(args.config.resolve())
    if continuous_version==36:
        if continuous is not True or diagnostic is not None:raise ValueError('v36_requires_fresh_continuous')
        from continuous_scope_v36 import sealed_config
        sealed_config(cfg)
    elif continuous_version==35:
        if continuous is not True or diagnostic is not None:raise ValueError('v35_requires_fresh_continuous')
        from continuous_scope_v35 import sealed_config
        profile_binding=sealed_config(cfg)
    elif continuous_version==34:
        if continuous is not True or diagnostic is not None:raise ValueError('v34_requires_fresh_continuous')
        from continuous_scope_v34 import sealed_config
        profile_binding=sealed_config(cfg)
    elif continuous_version==33:
        if continuous is not True or diagnostic is not None:raise ValueError('v33_requires_fresh_continuous')
        from continuous_scope_v33 import sealed_config
        profile_binding=sealed_config(cfg)
    elif continuous_version==32:
        if continuous is not True or diagnostic is not None:raise ValueError('v32_requires_fresh_continuous')
        from continuous_scope_v32 import sealed_config
        profile_binding=sealed_config(cfg)
    elif continuous_version==31:
        if continuous is not True or diagnostic is not None:raise ValueError('v31_requires_fresh_continuous')
        from continuous_scope_v31 import sealed_config
        profile_binding=sealed_config(cfg)
    elif continuous_version==30:
        if continuous is not True or diagnostic is not None:raise ValueError('v30_requires_fresh_continuous')
        from continuous_scope_v30 import sealed_config
        profile_binding=sealed_config(cfg)
    elif continuous_version==29:
        if continuous is not True or diagnostic is not None:raise ValueError('v29_requires_fresh_continuous')
        from continuous_scope_v29 import sealed_config
        profile_binding=sealed_config(cfg)
    elif continuous_version==28:
        if continuous is not True or diagnostic is not None:raise ValueError('v28_requires_fresh_continuous')
        from continuous_scope_v28 import sealed_config
        profile_binding=sealed_config(cfg)
    elif continuous_version==27:
        if continuous is not True or diagnostic is not None:raise ValueError('v27_requires_fresh_continuous')
        from continuous_scope_v27 import sealed_config
        profile_binding=sealed_config(cfg)
    elif continuous_version==26:
        if continuous is not True or diagnostic is not None:raise ValueError('v26_requires_fresh_continuous')
        from continuous_scope_v26 import sealed_config
        profile_binding=sealed_config(cfg)
    elif continuous_version==25:
        if continuous is not True or diagnostic is not None:raise ValueError('v25_requires_fresh_continuous')
        from continuous_scope_v25 import sealed_config
        profile_binding=sealed_config(cfg)
    elif continuous_version==24:
        if continuous is not True or diagnostic is not None:raise ValueError('v24_requires_fresh_continuous')
        from continuous_scope import sealed_config
        sealed_config(cfg)
    elif continuous_version!=23:raise ValueError('unknown_continuous_version')
    if type(continuous) is not bool or (continuous and continuous_version==23 and (diagnostic is not None or not cfg.reserve_home_workspace
            or cfg.layout!='original' or cfg.count!=10 or cfg.mode!='roundtrip')):
        raise ValueError('continuous_diagnostic_requires_fresh_original10_reserved_homes')
    if cfg.reserve_home_workspace and not continuous:raise ValueError('workspace_reservation_continuous_diagnostic_only')
    if cfg.home_cpu_profile and continuous_version not in (25,26,27,28,29,30,31,32,33,34,35,36):raise ValueError('home_cpu_profile_version')
    pair_probe=diagnostic.get('continuation_diagnostic') if diagnostic else None
    if pair_probe and (diagnostic['bundle'].get('target')!=12 or diagnostic['candidate_index']!=6
            or diagnostic['max_previews']!=2 or diagnostic['legacy_insertion']
            or diagnostic.get('fixed_setpoint_diagnostic') or not cfg.high_transfer_branch_seed or cfg.retreat_time_scale!=1):
        raise ValueError('continuation_diagnostic_requires_n12_c6_seedon_scale1_two_trials')
    if cfg.home_progress and not (diagnostic or continuous):raise ValueError('home_progress_diagnostic_only_until_physical_gate')
    if cfg.home_contact_envelope and not (diagnostic or continuous):raise ValueError('home_contact_envelope_diagnostic_only_until_physical_gate')
    if cfg.home_coarse and not (diagnostic or continuous):raise ValueError('home_coarse_diagnostic_only_until_physical_gate')
    if cfg.velocity_feedback=='k6' and not (diagnostic or continuous):raise ValueError('k6_diagnostic_only')
    from fixed_setpoint_diagnostic import validate_request
    validate_request(diagnostic.get('fixed_setpoint_diagnostic') if diagnostic else None,diagnostic,cfg)
    from n17_observer import validate_request as validate_n17_observer
    validate_n17_observer(diagnostic.get('n17_observer') if diagnostic else None,diagnostic,cfg)
    args.max_objects=1 if cfg.mode=='smoke' else cfg.count
    expected_layout=json.loads(args.expected_layout.read_text()) if args.expected_layout else None
    if args.experience_snapshot:args.experience_snapshot=args.experience_snapshot.resolve()
    release=args.release.resolve();out=args.output.resolve();root=Path(__file__).resolve().parent
    allowed=args.output_root.resolve(strict=True)
    mount=subprocess.check_output(['findmnt','-n','-T',str(allowed),'-o','TARGET'],text=True).strip()
    if mount not in ('/data4','/data_ft_6','/data_ft_7','/data_ft_9_10'):raise ValueError('output root is not on an approved data disk')
    if out.parent!=allowed or out.exists():raise ValueError('need a new output directly on verified data disk')
    gpu=gpu_gate(args.gpu) if visual is None else visual.gpu_binding()
    if args.check_only:
        print(timing.call('json',json.dumps,dict(ready=True,gpu=gpu,simulation_started=False)));return
    os.environ['CUDA_VISIBLE_DEVICES']=gpu['uuid']
    os.environ['GRIP_DRIVE_LIMIT_RAW']='80'
    os.environ['GRIP_RESET_RELEASE_VELOCITY']='1'
    os.environ['GRIP_FINGER_STATIC_FRICTION']='3.0'
    os.environ['GRIP_FINGER_DYNAMIC_FRICTION']='2.4'
    os.environ.pop('REAL10_AABB_SCALE',None)
    os.environ['PYTHONDONTWRITEBYTECODE']='1'
    out.mkdir()
    timing.start(out)
    if diagnostic:
        # Written before scene setup; even an initialization failure is fenced
        # off from production verification and dataset acceptance.
        from provenance import write_once
        write_once(out/'DIAGNOSTIC_ONLY.json',dict(schema='return-diagnostic/7',accepted=False,
            source_fingerprint=diagnostic['bundle']['source_fingerprint']))
    elif continuous:
        from provenance import write_once
        write_once(out/'DIAGNOSTIC_ONLY.json',dict(schema='continuous-roundtrip/22',accepted=False,
            initialized_episode=True,reconstructed=False,formal_credit=False))
    for name in ('cache','cache/torch','cache/cuda','cache/matplotlib','cache/gl'):(out/name).mkdir(exist_ok=True)
    os.environ.update(TORCH_HOME=str(out/'cache/torch'),CUDA_CACHE_PATH=str(out/'cache/cuda'),
                      MPLCONFIGDIR=str(out/'cache/matplotlib'),__GL_SHADER_DISK_CACHE_PATH=str(out/'cache/gl'))
    import numpy as np
    import sapien
    if visual is None:
        from device_binding import install as bind_assigned_renderer
        render_assignment=bind_assigned_renderer(sapien,gpu['pci_bus_id'])
        import imageio.v2 as imageio
    else:render_assignment=visual.render_binding()
    import yaml
    from direct_core import ForwardModel
    import astra_planner as planner
    from collision_world import CollisionWorld,pose_matrix
    from surface_grasps import mesh_grasps
    from return_world import ReturnWorld
    from return_pipeline import ReturnPipeline
    from provenance import write_once,digest,file_hash,source_files,PhaseLedger
    from pose_goals import dependencies,pose_errors
    from scene_pipeline import load_generated,identity_records,validate_layout,inventory,CONTAINER
    from experience import Experience,descriptor,policy_hash
    from separation import plan_separation
    from physics_geometry import convex_parts,bounds_for_parts
    from candidate_search import CandidateSearch
    from gripper_control import advance_finger_targets,is_finger_contact
    from experience_v4 import load_experience
    experience=load_experience(args.experience_snapshot,cfg)
    base=release/'workspace/base';candidate=release/'workspace/candidate'
    if visual is not None:visual.install(sapien,base,pair_probe,cfg)
    phase_started=time.monotonic();phase_times={};phase='initialization'
    support_map={};goal_poses={};returning=False;returned=[];experience_records=[]
    expected_container=CONTAINER
    write_once(out/'effective-config.json',cfg.to_dict())
    os.chdir(base);sys.path[:0]=[str(candidate),str(base)]
    calls=[]
    def forbidden(*unused,**kw):
        calls.append('prebuilt_planner_or_grasp_API');raise RuntimeError(calls[-1])
    stub=types.ModuleType('envs.robot.planner')
    class ForbiddenPlanner:
        def __init__(self,*a,**kw):forbidden()
    stub.MplibPlanner=stub.CuroboPlanner=ForbiddenPlanner
    sys.modules['envs.robot.planner']=stub
    import envs
    spec=importlib.util.spec_from_file_location('envs.empty_container_inner0926',candidate/'empty_container_inner0926_v11.py')
    task=importlib.util.module_from_spec(spec);sys.modules[spec.name]=task;spec.loader.exec_module(task)
    from envs.robot.robot import Robot
    # Extract only the historical scene-construction functions; never run its
    # driver, grasp pool, planner or policy. Physics assets stay read-only.
    tree=ast.parse((candidate/'run_real10_v11_targeted.py').read_text())
    functions=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('load_selected','build_catch_trays')]
    ns=dict(taskmod=task,np=np,sapien=sapien,json=json,os=os,SELECTED_IDS=(3,4,8,9,10,11,12,17,18,19))
    exec(compile(ast.Module(body=functions,type_ignores=[]),'scene_helpers','exec'),ns)
    class DirectScene(task.empty_container_inner0926):
        def setup_scene(self,**kw):
            super().setup_scene(**kw)
            if visual is None:sapien.render.set_ray_tracing_samples_per_pixel(4)
        def load_robot(self,**kw):
            self.robot=Robot(self.scene,False,**kw);self.robot.init_joints()
            self.robot.communication_flag=False
            for name in ('set_planner','left_plan_path','right_plan_path','left_plan_multi_path','right_plan_multi_path'):
                setattr(self.robot,name,forbidden)
        def together_open_gripper(self,**kw):
            for arm in ('left','right'):self.robot.set_gripper(1.,arm,gripper_eps=0)
            for _ in range(250):
                for entity in (self.robot.left_entity,self.robot.right_entity):self.robot._entity_qf(entity)
                self.scene.step()
        def _register_mesh_library(self):pass
        def _init_online_scene_filter(self):pass
        def load_actors(self):
            ns['build_catch_trays'](self);super().load_actors()
            if cfg.layout=='generated':
                # More candidate centers, same physical tray geometry. Fresh
                # payload/world/home/retreat checks decide actual capacity.
                self.grid_slots_left=[[float(x),float(y),.753,1,0,0,0] for x in (-.72,-.60,-.48,-.36) for y in np.arange(-.60,.241,.10)]
                self.grid_slots_right=[[float(x),float(y),.753,1,0,0,0] for x in (.36,.48,.60) for y in np.arange(-.60,.241,.10)]
        def _load_layout_objects(self):
            if cfg.layout=='original':
                inventory(json.loads(Path(task.LAYOUT_PATH).read_text()),10)
                ns['load_selected'](self)
            else:load_generated(self,task,sapien,cfg)
        def _load_settled_poses(self):
            if cfg.layout=='original':
                if not task.SETTLED_PATH.is_file():raise RuntimeError('missing_original_settled_fixture')
                super()._load_settled_poses()
        def check_stable(self):
            # Preserve baseline settling steps; never write into release/assets.
            for _ in range(2000):self.scene.step()
            self.roundtrip_earlier={str(n):o.get_pose().p.tolist()+o.get_pose().q.tolist() for n,o in self.objects.items()}
            for _ in range(500):self.scene.step()
            # Explicit fail-closed gate below retains invalid-layout evidence.
            return True,[]
        move=_list_grasp_candidates=_list_candidates_robust=_hard_reset_planners=forbidden
        get_grasp_pose=get_place_pose=_update_pile_obstacles=forbidden
    # Resolve robot configuration locally, without the old machine's absolute
    # embodiment registry paths or annotation-test imports.
    config=yaml.safe_load((base/'task_config/demo_franka.yml').read_text())
    robot_config=yaml.safe_load((candidate/'franka-panda/config.yml').read_text())
    # Legacy normalized commands extrapolated to [-.01,.05], outside the
    # URDF's [0,.04] finger travel. Use physical limits for open/release too.
    robot_config['gripper_scale']=[0.,.04]
    config.update(left_embodiment_config=robot_config,right_embodiment_config=robot_config,
                  dual_arm_embodied=False,embodiment_dis=config['embodiment'][2],
                  embodiment_name='franka-panda+franka-panda',need_plan=False)
    config.update(left_robot_file=str(candidate/'franka-panda'),right_robot_file=str(candidate/'franka-panda'),
                  task_name='astra_authored_real10',save_data=False,render_freq=0)
    config['data_type']['rgb']=True
    for camera in robot_config['static_camera_list']:
        if camera['name']=='head_camera':
            direction=np.asarray(camera['forward'],float);direction/=np.linalg.norm(direction)
            camera['position']=(np.asarray(camera['position'])-.5*direction).tolist()
    if task.SCENE_SCALE!=.7 or task.COLLISION_SCALE!=.95 or not np.allclose(task.CONTAINER_INNER,[.532,.427]):
        raise RuntimeError('release_geometry_constants_differ_from_r21')
    env=DirectScene()
    try:
        env.setup_demo(now_ep_num=0,seed=cfg.seed,**config)
    except Exception as exc:
        if cfg.mode=='layout':
            from layout_preflight import initialization_failure
            initialization_failure(out,cfg,exc)
        raise
    if cfg.mode!='layout':
        grasp_scene=DirectScene();grasp_scene.setup_demo(now_ep_num=0,seed=cfg.seed,**config)
        separation_scene=DirectScene();separation_scene.setup_demo(now_ep_num=0,seed=cfg.seed,**config)
        physics_systems=[e.scene.physx_system for e in (env,grasp_scene,separation_scene)]
        assert len({id(system) for system in physics_systems})==3
        if pair_probe:
            continuation_scene=DirectScene();continuation_scene.setup_demo(now_ep_num=0,seed=cfg.seed,**config)
            if any(continuation_scene.scene.physx_system is s for s in physics_systems):
                raise RuntimeError('continuation_shadow_not_independent')
    if visual is not None:
        visual.bind('formal',env)
        if cfg.mode!='layout':
            visual.bind('grasp_preview',grasp_scene);visual.bind('separation_preview',separation_scene)
            if pair_probe:visual.bind('continuation_preview',continuation_scene)
    model=ForwardModel(root/'fixtures/panda.urdf')
    geometry=json.loads((root/'fixtures/collision-boxes.json').read_text())
    def scene_meshes(environment):
        return {str(n):convex_parts(actor.actor.find_component_by_type(
            sapien.pysapien.physx.PhysxRigidDynamicComponent).get_collision_shapes())
            for n,actor in environment.objects.items()}
    physical_meshes=scene_meshes(env)
    if pair_probe and scene_meshes(continuation_scene)!=physical_meshes:
        raise RuntimeError('continuation_shadow_geometry_mismatch')
    if cfg.mode!='layout' and (scene_meshes(grasp_scene)!=physical_meshes or scene_meshes(separation_scene)!=physical_meshes):
        raise RuntimeError('forecast_collision_geometry_mismatch')
    for oid,parts in physical_meshes.items():
        geometry['objects'].setdefault(oid,{})['parts']=bounds_for_parts(parts)
    mesh_bytes=(timing.call('json',json.dumps,physical_meshes,sort_keys=True)+'\n').encode()
    (out/'physics-meshes.json').write_bytes(mesh_bytes)
    mesh_hash=hashlib.sha256(mesh_bytes).hexdigest()
    robot_vertices=json.loads((root/'fixtures/robot-vertices.json').read_text())
    @timing.measured('placement')
    def collision_world(s):
        return ReturnWorld(model,geometry,s,physical_meshes,robot_vertices,
            supports=support_map if returning else {},goals=goal_poses if returning else {},optimize=cfg.optimize_geometry)
    def payload_bottom(target):
        t=pose_matrix(state()['objects'][str(target)]['pose'])
        return min((np.asarray(m['vertices'])@t[:3,:3].T+t[:3,3])[:,2].min() for m in physical_meshes[str(target)])
    rng=np.random.default_rng(cfg.planner_seed);planner.MAX_COLLISION_CHECKS=1200
    bounds=np.array(model.bounds)+[.03,-.03]
    step=0;preview_mode=False;preview_steps=0;preview_attempts=0
    preview_total_steps=0;grasp_preview_attempts=0;preview_trace=[]
    if timing.ACTIVE is not None:timing.ACTIVE.context=lambda:(phase,preview_mode,returning)
    phase_physical_steps={};phase_preview_steps={};phase_step_start=0;phase_preview_start=0
    if cfg.mode!='layout':
        log=(out/'events.jsonl').open('x');writer=imageio.get_writer(str(out/'episode.mp4'),fps=10) if visual is None else visual.writer(out)
        trace=(out/'trajectory.jsonl').open('x')
    trace_hash=hashlib.sha256();trace_samples=0;placement_trace=None;copy_observer=None;live_sleep=None
    formal_objects={n:id(o.actor) for n,o in env.objects.items()}
    identity={}
    @timing.measured('phase')
    def switch_phase(name):
        nonlocal phase,phase_started,phase_step_start,phase_preview_start
        now=time.monotonic();phase_times[phase]=phase_times.get(phase,0.)+now-phase_started
        phase_physical_steps[phase]=step-phase_step_start;phase_preview_steps[phase]=preview_total_steps-phase_preview_start
        phase_step_start=step;phase_preview_start=preview_total_steps
        phase=name;phase_started=now
        event('phase',phase=name)
    @timing.measured('trace')
    def record_trace(force=False):
        nonlocal trace_samples
        if preview_mode:return
        if visual is not None:timing.call('visual',visual.capture,env,step,phase,held_id,drive is not None,force)
        if not force and step%25:return
        packet=dict(physics_step=step,phase=phase,state=state(),held_id=held_id,drive_active=drive is not None)
        row=(timing.call('json',json.dumps,packet,sort_keys=True,allow_nan=False)+'\n').encode()
        trace.write(row.decode());trace.flush();trace_hash.update(row);trace_samples+=1
    @timing.measured('event')
    def event(kind,**values):
        if live_sleep is not None:live_sleep.event(kind,values,env)
        if kind=='live_sleep_boundary':
            timing.call('write',log.write,timing.call('json',json.dumps,dict(kind=kind,time_unix=time.time(),physics_step=step,phase=phase,**values),allow_nan=False)+'\n');timing.call('flush',log.flush);return
        if kind in ('preview_accounting_started','preview_accounting_terminal'):
            timing.call('write',log.write,timing.call('json',json.dumps,dict(kind=kind,time_unix=time.time(),physics_step=step,phase=phase,scene_scope='isolated_preview_lifecycle',**values),allow_nan=False)+'\n');timing.call('flush',log.flush);return
        if preview_mode:
            if kind in ('gripper_capture_preflight','gripper_capture_handoff','capture_rebase','capture_motion_interrupted','capture_begin','capture_latch','capture_loss','capture_reacquire','capture_hold_interrupted','capture_command','capture_readback','capture_tick_started','capture_observation','capture_relief','capture_terminal','supported_handoff_source','supported_handoff_plan','supported_handoff_command','supported_handoff_observation','supported_handoff_hold','supported_handoff_terminal','held_transform_source','actuator_caller_source','closure_control_source','closure_control_update','separation_actuator_source','custom_plan','actuator_route_admission','actuator_route_readback','grasp_actuator_handoff','forecast_initialization','separation_profile_bundle','gripper_hold','after_closure','gripper_settle','gripper_settle_failed','separation_plan','separation_preview','separation_measured',
                        'ik_rejected','branch_seed_attempt','branch_seed_path','physical_tracking','return_payload_transform','insertion_plan',
                        'insertion_tracking','insertion_failure','return_motion_plan',
                        'return_motion_rejected','return_motion_trace','return_motion_settled',
                        'motion_refinement_prepared','motion_refinement_rejected','motion_refinement_execution',
                        'home_coarse_prepared','home_coarse_rejected','motion_phase_entry','motion_phase_exit',
                        'fixed_setpoint_checkpoint','fixed_setpoint_hold','feedback_probe','velocity_feedback_receipt','feedback_metadata',
                        'vertex_governor_policy','vertex_governor_rejected','vertex_governor_checkpoint','vertex_governor_vertex',
                        'return_hold_admitted','return_hold_certificate','return_hold_release_reference','return_hold_trace',
                        'return_handoff_bridge','return_handoff_approach_checked','return_final_home_started','return_final_home_completed',
                        'diagnostic_first_return_completed','diagnostic_next_selected','diagnostic_second_return_completed',
                        'diagnostic_first_grasp_completed','diagnostic_next_grasp_completed',
                        'diagnostic_pair_completed','return_continuation_failed',
                        'return_release','returned','n17_observation','home_contact_model','home_contact_rejected','home_progress_admitted','home_progress_receipt'):
                preview_trace.append(dict(kind=kind,**values))
            return
        if kind in ('extracted','extraction_barrier','return_started','returned','return_release'):
            record_trace(force=True)
        timing.call('write',log.write,timing.call('json',json.dumps,dict(kind=kind,time_unix=time.time(),physics_step=step,**({'phase':phase}|values)),allow_nan=False)+'\n');timing.call('flush',log.flush)
    @timing.measured('state')
    def state(environment=None):
        current=env if environment is None else environment
        return dict(robots={a:dict(q=getattr(current.robot,a+'_entity').get_qpos().tolist(),
                        qdot=getattr(current.robot,a+'_entity').get_qvel().tolist()) for a in ('left','right')},
                    objects={str(n):dict(pose=o.get_pose().p.tolist()+o.get_pose().q.tolist(),
                        linear_velocity=o.actor.find_component_by_type(sapien.pysapien.physx.PhysxRigidDynamicComponent).get_linear_velocity().tolist(),
                        angular_velocity=o.actor.find_component_by_type(sapien.pysapien.physx.PhysxRigidDynamicComponent).get_angular_velocity().tolist(),
                        **(identity.get(str(n),{})|dict(actor_name=o.actor.get_name())),
                        extents=(np.array(o.config['extents'])*np.array(o.config['scale'])).tolist(),top_z=current._top_z(o)) for n,o in current.objects.items()})
    @timing.measured('contacts')
    def contacts():
        return [dict(bodies=[b.entity.get_name() for b in c.bodies],
            separation_min=min(float(p.separation) for p in c.points),
            impulse_max=max(float(np.linalg.norm(p.impulse)) for p in c.points),
            points=[dict(separation=float(p.separation),normal=p.normal.tolist(),position=p.position.tolist()) for p in c.points])
            for c in env.scene.get_contacts() if c.points]
    @timing.measured('contact_guard')
    def contact_guard(target,held,allow_target_contact=True):
        fingers={getattr(env.robot,a+'_entity').find_link_by_name(n) for a in ('left','right') for n in ('panda_leftfinger','panda_rightfinger')}
        robot_links={l for a in ('left','right') for l in getattr(env.robot,a+'_entity').get_links()}
        target_name=env.objects[target].actor.get_name()
        touched=False
        for c in env.scene.get_contacts():
            bodies=(c.bodies[0],c.bodies[1]);names=[b.entity.get_name() for b in bodies]
            significant=any(np.linalg.norm(p.impulse)>.002 or p.separation<-.0003 for p in c.points)
            robot=[b for b in bodies if b in robot_links]
            target_finger=allow_target_contact and len(robot)==1 and robot[0] in fingers and target_name in names
            if target_finger and any(is_finger_contact(float(p.separation),float(np.linalg.norm(p.impulse))) for p in c.points):
                touched=True
            if not significant:continue
            if robot:
                if not target_finger:raise CandidateInfeasible('forbidden_robot_contact:'+str(names))
            if held and target_name in names and not robot:
                other=names[1] if names[0]==target_name else names[0]
                supports={'container_floor':.752,'table':.74,'real10_catch_left':.74,'real10_catch_right':.74}
                if other in supports and payload_bottom(target)>=supports[other]-.0005 and all(p.separation>=-.0005 for p in c.points):continue
                if returning and str(target) in goal_poses:
                    allowed={identity[i]['actor_name'] for i in support_map.get(str(target),[])}
                    pe,re=pose_errors(state()['objects'][str(target)]['pose'],goal_poses[str(target)])
                    if other in allowed and pe<=.003 and re<=np.degrees(.04) and all(p.separation>=-.0005 and abs(p.normal[2])>=.65 for p in c.points):
                        continue
                raise CandidateInfeasible('forbidden_payload_contact:'+str(names))
        return touched
    step=0;continuation_guard=None;return_deadline=None;live=None
    @timing.measured('tick')
    def tick(target,held,allow_target_contact=True):
        nonlocal step,preview_steps,preview_total_steps
        if return_deadline is not None and time.monotonic()>=return_deadline:
            raise CandidateInfeasible('return_continuation_time_cap')
        if continuation_guard is not None:timing.call('continuation',continuation_guard.check,'pretick',held)
        if live is not None:live.before_tick()
        for entity in (env.robot.left_entity,env.robot.right_entity):timing.call('qf',env.robot._entity_qf,entity)
        live_sleep.tick(env,step,preview_total_steps,preview_mode)
        if live is not None and live.tracking is not None:
            live.pre_step({a:getattr(env.robot,a+'_entity').get_qf().tolist() for a in ('left','right')})
        timing.call('scene_step',env.scene.step)
        if preview_mode:preview_steps+=1;preview_total_steps+=1
        else:step+=1
        if placement_trace is not None:timing.call('placement_tick',placement_trace.tick,target,held)
        if (preview_mode and preview_total_steps%10000==0) or (not preview_mode and step%2500==0):
            import shutil
            loaded_science.check_output()
            if shutil.disk_usage(out).free<2*1024**3:raise RuntimeError('data_disk_reserve_reached')
        if not preview_mode and ({n:id(o.actor) for n,o in env.objects.items()}!=formal_objects):
            raise RuntimeError('formal_object_substitution')
        contact_guard(target,held,allow_target_contact)
        if live is not None:live.observe(target,held)
        if continuation_guard is not None:timing.call('continuation',continuation_guard.check,'posttick',held)
        record_trace()
        if not preview_mode and step%25==0:
            if visual is None:timing.call('visual',writer.append_data,env.get_obs()['observation']['head_camera']['rgb'])
            else:timing.call('visual',visual.picture_checkpoint,env,step)
    held_id=None;drive=None;completed=[];failure=None;plans=0;placement_targets={}
    homes={a:state()['robots'][a]['q'][:7] for a in ('left','right')}
    selected=[cfg.smoke_id] if cfg.mode=='smoke' else list(env.objects)
    ledger=PhaseLedger(env.objects,selected)
    identity=identity_records(env,task,physical_meshes)
    asset_counts={}
    for o in identity.values():asset_counts[o['asset_sha256']]=asset_counts.get(o['asset_sha256'],0)+1
    initial=state();initial_contacts=contacts()
    validation=validate_layout(initial,env.roundtrip_earlier,physical_meshes,initial_contacts,cfg)
    deps,support_map=({}, {}) if cfg.mode=='layout' else dependencies(initial['objects'],physical_meshes,initial_contacts)
    goal_poses={oid:o['pose'] for oid,o in initial['objects'].items()}
    manifest=dict(schema='roundtrip-scene/1',config=cfg.to_dict(),objects=initial['objects'],
        robots=initial['robots'],initial_contacts=initial_contacts,dependencies=deps,supports=support_map,
        layout_validation=validation,selected_ids=[] if cfg.mode=='layout' else list(map(str,selected)),container=CONTAINER,
        earlier_poses=env.roundtrip_earlier,
        physics_mesh_sha256=mesh_hash,source_sha256=source_files(root),
        release_helpers_sha256={n:file_hash(candidate/n) for n in ('empty_container_inner0926_v11.py','run_real10_v11_targeted.py')},
        original_settled_sha256=file_hash(task.SETTLED_PATH) if cfg.layout=='original' else None,
        experience_snapshot_sha256=experience.snapshot_sha256,configuration_sha256=policy_hash(cfg),
        asset_inventory=dict(instances=len(identity),distinct_assets=len(asset_counts),duplicate_instances=len(identity)-len(asset_counts),
                             repeated_asset_hashes={h:n for h,n in asset_counts.items() if n>1}),
        render_assignment=render_assignment,
        runtime_llm=dict(instrumented=True,calls=0,input_tokens=0,output_tokens=0,reason='deterministic local Python; no model client'),
        development_llm=dict(instrumented=False,usage=None,source='parent turn.completed pending'))
    if visual is not None:manifest['headless_candidate']=visual.manifest(identity,config)
    manifest_hash=write_once(out/'scene-manifest.json',manifest)
    write_once(out/'layout-validation.json',validation)
    endpoint_binding=dict(implementation_sha256=digest(manifest['source_sha256']),config_sha256=digest(cfg.to_dict()),
        physics_mesh_sha256=mesh_hash,fixtures={name:file_hash(root/'fixtures'/name) for name in ('panda.urdf','collision-boxes.json','robot-vertices.json')})
    if cfg.placement_trace_object_id is not None:
        from placement_trace import PlacementTrace,native_observation
        formal_env=env
        placement_trace=PlacementTrace(out,cfg.placement_trace_object_id,
            dict(endpoint_binding,scene_manifest_sha256=manifest_hash),str(id(physics_systems[0])),str(id(env)),
            lambda arm,target,relative:native_observation(env,formal_env,model,
                sapien.pysapien.physx.PhysxRigidDynamicComponent,arm,target,relative,held_id,drive,phase,step,preview_total_steps,preview_mode))
    if cfg.mode=='layout':
        from layout_preflight import finish_layout
        return finish_layout(out,manifest,env.get_obs()['observation']['head_camera']['rgb'],
                             time.monotonic()-phase_started)
    event('initial_state',state=initial);record_trace(force=True)
    def rank_candidates(candidates,target):
        stage='return' if returning else 'extraction'
        current=state()
        ranked=experience.rank(candidates,identity[str(target)]['geometry_sha256'],policy_hash(cfg),descriptor(current,target),stage,
                               object_id=target,current_state=current)
        event('experience_ranking',object_id=target,**experience.last_trace)
        return ranked
    def experience_record(target,candidate,passed,stage):
        # Evaluation never mutates the loaded frozen policy; heldout records
        # are diagnostics only and the exporter refuses to train on them.
        experience_records.append(dict(object_id=str(target),candidate=candidate,evidence_scope='isolated_preview_only',geometry_sha256=identity[str(target)]['geometry_sha256'],
            configuration_sha256=policy_hash(cfg),scene_descriptor=descriptor(state(),target),
            depth_m=candidate['surface_depth_m'],outcome='success' if passed else 'failure',stage=stage,split=cfg.split))
    @timing.measured('plan')
    def execute(arm,path,target,loaded=False,smooth=False,preserve_bias=False,acceleration=.4,allow_target_contact=True,max_velocity=.25,actuator_world=None,separation_source=None):
        actuator_bridge=None
        if cfg.defer_intermediate_home and returning and not loaded:
            from return_motion import bridge_to_measured
            if not np.array_equal(path[0],state()['robots'][arm]['q'][:7]):
                raise CandidateInfeasible('handoff_stale_measured_start')
            # Freshly planned approach starts at this measured configuration;
            # physically bridge the existing drive to that reference first.
            bridge_to_measured(continuation_api(),arm,target)
            # The bridge performed physics. Recheck this approach against a
            # fresh measured world, including the actual idle arm and fingers;
            # never inherit collision approval from before that transition.
            observed=state();fresh=collision_world(observed)
            fingers=np.clip(observed['robots'][arm]['q'][-2:],0.,.04).tolist()
            samples=[observed['robots'][arm]['q'][:7]]
            for start,end in zip(path[:-1],path[1:]):
                samples.extend(np.linspace(start,end,max(2,int(np.ceil(np.max(np.abs(np.asarray(end)-start))/.025))+1)).tolist())
            if not all(fresh.free(arm,q,target,allow_target_contact,fingers) for q in samples):
                raise CandidateInfeasible('handoff_fresh_approach_collision')
            actuator_bridge=dict(object_id=target,arm=arm,state_sha256=digest(observed),
                path_sha256=digest(path),samples=len(samples),reused_approval=False)
            event('return_handoff_approach_checked',**actuator_bridge)
            actuator_world=fresh
        # Only the armed B extraction carry/lowering receives the bound clock.
        # Keep the certified measured-start path; B separately binds drive compensation.
        if live is not None and live.armed and live.variant=='B_clearance':
            from live_clearance import SMOOTH_PROFILE
            smooth=True;acceleration=SMOOTH_PROFILE['max_acceleration']
        if live is not None:live.bind(arm,path,target,loaded,smooth,preserve_bias,acceleration,max_velocity)
        if live is not None and live.armed and live.variant=='B_clearance':
            live.run_tracking(lambda:tick(target,loaded,allow_target_contact))
            actual=getattr(env.robot,arm+'_entity').get_qpos()[:7]
            if np.max(np.abs(actual-np.asarray(path[-1])))>.025:raise CandidateInfeasible('joint_tracking_not_settled')
            return
        dense=planner.resample_path(path,max_velocity=max_velocity,dt=.004,max_acceleration=acceleration if smooth else None)
        dense=np.asarray(dense)
        bias=np.zeros(7)
        if preserve_bias:
            previous=np.array([float(j.get_drive_target()[0]) for j in getattr(env.robot,arm+'_arm_joints')])
            bias=previous-dense[0]
            if np.max(np.abs(bias))>.025:raise CandidateInfeasible('initial_actuator_bias_too_large')
            if np.any(dense+bias<bounds[:,0]) or np.any(dense+bias>bounds[:,1]):
                raise CandidateInfeasible('actuator_bias_exceeds_joint_margin')
            event('actuator_bias_preserved',object_id=target,bias_rad=bias.tolist(),
                  previous_target=previous.tolist(),first_target=(dense[0]+bias).tolist())
        from actuator_targets import Route
        observed=state()
        if actuator_world is None:
            actuator_world=collision_world(observed)
            if loaded:
                relative_now=np.linalg.inv(pose_matrix(env.get_arm_pose(arm)))@pose_matrix(observed['objects'][str(target)]['pose'])
                actuator_world.held=(arm,str(target),relative_now)
        if actuator_world.state!=observed:raise CandidateInfeasible('actuator_route_stale_world')
        previous=[float(j.get_drive_target()[0]) for j in getattr(env.robot,arm+'_arm_joints')]
        from loaded_common import controls as actual_actuator_controls
        caller_source=dict(schema='actuator-caller/011',arm=arm,object_id=target,state=observed,
            phase=phase,returning=returning,
            controls=actual_actuator_controls(env),path=path,loaded=loaded,
            held_id=held_id,drive_active=drive is not None,
            native_ee=pose_matrix(env.get_arm_pose(arm)).tolist(),
            arguments=dict(smooth=smooth,acceleration=acceleration,max_velocity=max_velocity,
                preserve_bias=preserve_bias,allow_target_contact=allow_target_contact),
            bridge=actuator_bridge)
        caller_source['sha256']=digest(caller_source)
        event('actuator_caller_source',source=caller_source)
        route=Route(actuator_world,arm,target,path,dense,bias,previous,allow_target_contact,event,
                    dict(max_velocity=max_velocity,dt=.004,max_acceleration=acceleration if smooth else None),bridge=actuator_bridge,separation=separation_source,caller=caller_source['sha256'])
        emission_failed=True
        try:
            for i,q in enumerate(dense):
                velocity=route.v[i]
                if np.max(np.abs(velocity))>max_velocity+1e-5:raise RuntimeError('interpolation_velocity_limit')
                route.issue(env.robot,i);tick(target,loaded,allow_target_contact)
            emission_failed=False
        finally:route.finish(emission_failed)
        if placement_trace is not None and placement_trace.active and placement_trace.stage=='lowering':
            placement_trace.mark('lowering_hold')
        for _ in range(100):tick(target,loaded,allow_target_contact)
        if placement_trace is not None and placement_trace.active and placement_trace.stage=='lowering_hold':
            placement_trace.mark('lowering_settle')
        actual=getattr(env.robot,arm+'_entity').get_qpos()[:7]
        if np.max(np.abs(actual-np.asarray(path[-1])))>.025:raise CandidateInfeasible('joint_tracking_not_settled')
    def settle_separation(arm,target,separate):
        actor=env.objects[target]
        for attempt in range(21):
            actual_ee=pose_matrix(env.get_arm_pose(arm))
            measured_relative=np.linalg.inv(actual_ee)@actor.get_pose().to_transformation_matrix()
            measured=collision_world(state());measured.held=(arm,str(target),measured_relative)
            joints=getattr(env.robot,arm+'_entity').get_qpos().tolist()
            pe=float(np.linalg.norm(actual_ee[:3,3]-separate['position']))
            re=float(np.linalg.norm(planner.so3_log(np.asarray(separate['rotation'])@actual_ee[:3,:3].T)))
            clear=measured.free(arm,joints[:7],target,True,np.clip(joints[-2:],0.,.04).tolist())
            if clear and pe<=.005 and re<=.04:break
            if attempt<20:
                for _ in range(50):tick(target,True)
        report=dict(clear=bool(clear),position_error_m=pe,rotation_error_rad=re,
                    clearance=measured.payload_clearances(arm,joints[:7],separate['initial_clearance']))
        if not clear or pe>.005 or re>.04:raise CandidateInfeasible('separation_not_settled:'+timing.call('json',json.dumps,report))
        return measured_relative,report
    from preview_accounting_v35 import PreviewAccounting
    preview_accounting=PreviewAccounting(lambda:preview_total_steps,event)
    from sleep_copy_observer_v35 import observe_boundary,CopyObserver
    if cfg.sleep_copy_trace_object_id is not None:
        copy_observer=CopyObserver(out,cfg.sleep_copy_trace_object_id,
            dict(endpoint_binding,scene_manifest_sha256=manifest_hash),
            [(env,'formal'),(grasp_scene,'grasp_preview'),(separation_scene,'separation_preview')],
            sapien.pysapien.physx.PhysxRigidDynamicComponent,
            lambda:dict(phase=phase,formal_step=step,preview_step=preview_total_steps,
                        held_id=held_id,drive_active=drive is not None))
    from live_sleep_witness import LiveWitness
    live_sleep=LiveWitness(out,env,grasp_scene,sapien.pysapien.physx.PhysxRigidDynamicComponent,state,
        lambda:dict(formal_step=step,preview_step=preview_total_steps,phase=phase,held_id=held_id,drive_active=drive is not None,
                    lifecycle=preview_accounting.context()['lifecycle'],trajectory_sha256=trace_hash.hexdigest(),trajectory_samples=trace_samples),
        dict(endpoint_binding,scene_manifest_sha256=manifest_hash),event)
    from loaded_science import Science
    loaded_science=Science(out,loaded_variant,lambda:dict(formal_step=step,preview_step=preview_total_steps,preview=preview_mode,held_id=held_id,drive_active=drive is not None),dict(endpoint_binding,scene_manifest_sha256=manifest_hash))
    forecast_initialization='exact'
    @observe_boundary(copy_observer,preview_accounting.context)
    def copy_to_shadow(source,destination):
        # Only the independent forecast scene is written. Never unpack or
        # reset the committed scene: pack does not establish exact replay of
        # internal contact history, as the recorded first-step failure shows.
        if source.scene.physx_system is destination.scene.physx_system:
            raise RuntimeError('forecast_scene_not_independent')
        before=source.scene.physx_system.pack()
        def controls(environment):
            return [(j.get_stiffness(),j.get_damping(),j.get_force_limit(),j.get_drive_mode(),
                     j.get_drive_target().tolist(),j.get_drive_velocity_target().tolist())
                    for a in ('left','right') for j in getattr(environment.robot,a+'_entity').get_active_joints()]
        before_controls=controls(source)
        from forecast_sleep_hypotheses import ForecastInitialization
        sleep_copy=ForecastInitialization(source.objects,destination.objects,sapien.pysapien.physx.PhysxRigidDynamicComponent,forecast_initialization)
        def check_source_unchanged():
            sleep_copy.assert_source_unchanged()
            if source.scene.physx_system.pack()!=before:raise RuntimeError('forecast_modified_source_scene')
            if controls(source)!=before_controls:raise RuntimeError('forecast_modified_source_controls')
        try:
            if copy_observer is not None:copy_observer.attach(sleep_copy)
            for arm in ('left','right'):
                src=getattr(source.robot,arm+'_entity');dst=getattr(destination.robot,arm+'_entity')
                dst.set_root_pose(src.get_root_pose());dst.set_qpos(src.get_qpos());dst.set_qvel(src.get_qvel())
                sj=src.get_active_joints();dj=dst.get_active_joints()
                if [j.get_name() for j in sj]!=[j.get_name() for j in dj]:raise RuntimeError('forecast_joint_layout_mismatch')
                for original,copied in zip(sj,dj):
                    copied.set_drive_property(original.get_stiffness(),original.get_damping(),original.get_force_limit(),original.get_drive_mode())
                    copied.set_drive_target(float(original.get_drive_target()[0]))
                    copied.set_drive_velocity_target(float(original.get_drive_velocity_target()[0]))
            for n,actor in source.objects.items():
                twin=destination.objects[n]
                twin.actor.set_pose(actor.get_pose())
                cls=sapien.pysapien.physx.PhysxRigidDynamicComponent
                original=actor.actor.find_component_by_type(cls);copied=twin.actor.find_component_by_type(cls)
                copied.set_linear_velocity(original.get_linear_velocity());copied.set_angular_velocity(original.get_angular_velocity())
                if not np.array_equal(copied.get_linear_velocity(),original.get_linear_velocity()) or not np.array_equal(copied.get_angular_velocity(),original.get_angular_velocity()):
                    raise RuntimeError('forecast_velocity_copy_mismatch')
            sleep_copy.restore()
            sleep_copy.assert_scene_state(state(source),state(destination))
            if controls(destination)!=before_controls:raise RuntimeError('forecast_control_copy_mismatch')
        finally:
            check_source_unchanged()
        check_source_unchanged()
        if forecast_initialization!='exact':event('forecast_initialization',**sleep_copy.receipt())
        live_sleep.begin_prefix(source,destination)
        return check_source_unchanged
    if visual is not None and visual.admission:
        if not validation['valid']:raise RuntimeError('invalid_generated_or_original_layout:'+str(validation['errors']))
        status=visual.admit(copy_to_shadow,state)
        writer.close();log.close();trace.close()
        return status
    def nominal_preview_separation(arm,target,candidate):
        """Forecast on a separate physics system; the source is never rewound."""
        nonlocal env,preview_mode,preview_steps,preview_attempts
        source=env;previous_mode=preview_mode
        profiles=[dict(preserve_bias=bias,acceleration=acc) for acc in (.4,1.2,.1) for bias in (False,True)]
        for profile in profiles:
            with preview_accounting.attempt('separation',arm,target,dict(profile=profile)) as accounting_frame:
                check_unchanged=copy_to_shadow(source,separation_scene)
                env=separation_scene;preview_steps=0;preview_mode=True
                failure=None;report=None;preview_attempts+=1;forecast_drive=None
                try:
                    forecast_drive=env._magnetic_attach(arm,env.objects[target],target)
                    execute(arm,candidate['path'],target,True,smooth=True,separation_source=candidate,**profile)
                    _,report=settle_separation(arm,target,candidate)
                except CandidateInfeasible as exc:
                    failure=str(exc)
                finally:
                    if forecast_drive is not None:env._magnetic_detach(forecast_drive)
                    env=source;preview_mode=previous_mode;check_unchanged()
                event('separation_preview',object_id=target,offset_m=candidate['offset_m'],
                      profile=profile,trial_steps=preview_steps,accepted=failure is None,
                      failure=failure,measurement=report,source_unchanged=True,independent_scene=True)
                accounting_frame['accepted']=failure is None
                if failure is None:
                    candidate['execution_profile']=profile;return True
        return False
    def paired_preview_separation(arm,target,candidate):
        """Require the same candidate/profile in every declared initialization."""
        nonlocal env,preview_mode,preview_steps,preview_attempts,forecast_initialization
        from forecast_sleep_hypotheses import initialization_modes,CONTRACT
        from provenance import digest
        source=env;previous_mode=preview_mode;previous_initialization=forecast_initialization
        cls=sapien.pysapien.physx.PhysxRigidDynamicComponent
        modes=initialization_modes(source.objects,cls)
        profiles=[dict(preserve_bias=bias,acceleration=acc) for acc in (.4,1.2,.1) for bias in (False,True)]
        for profile in profiles:
            outcomes=[]
            for mode in modes:
                with preview_accounting.attempt('separation',arm,target,dict(profile=profile,initialization=mode,forecast_contract=CONTRACT)) as accounting_frame:
                    failure=None;report=None;forecast_drive=None;check_unchanged=None
                    preview_steps=0
                    try:
                        forecast_initialization=mode
                        check_unchanged=copy_to_shadow(source,separation_scene)
                        env=separation_scene;preview_mode=True;preview_attempts+=1
                        forecast_drive=env._magnetic_attach(arm,env.objects[target],target)
                        execute(arm,candidate['path'],target,True,smooth=True,separation_source=candidate,**profile)
                        _,report=settle_separation(arm,target,candidate)
                    except CandidateInfeasible as exc:
                        failure=str(exc)
                    finally:
                        try:
                            if forecast_drive is not None:env._magnetic_detach(forecast_drive)
                        finally:
                            env=source;preview_mode=previous_mode
                            forecast_initialization=previous_initialization
                            if check_unchanged is not None:check_unchanged()
                    event('separation_preview',object_id=target,offset_m=candidate['offset_m'],
                          path_sha256=digest(candidate['path']),
                          preview_id=accounting_frame['identity']['preview_id'],
                          failure_category=None if failure is None else 'candidate_infeasible',
                          profile=profile,initialization=mode,forecast_contract=CONTRACT,
                          trial_steps=preview_steps,accepted=failure is None,failure=failure,
                          measurement=report,source_unchanged=True,independent_scene=True)
                    accounting_frame['accepted']=failure is None
                    outcomes.append(dict(initialization=mode,accepted=failure is None))
                if failure is not None:break
            accepted=len(outcomes)==len(modes) and all(row['accepted'] for row in outcomes)
            event('separation_profile_bundle',object_id=target,profile=profile,required_initializations=list(modes),
                  path_sha256=digest(candidate['path']),outcomes=outcomes,accepted=accepted,forecast_contract=CONTRACT)
            if accepted:
                candidate['execution_profile']=profile
                candidate['forecast_contract']=dict(name=CONTRACT,required_initializations=list(modes),
                    empirical_robustness_only=True,conservative_bound_proved=False)
                return True
        return False
    def strict_control_copy(source,destination):
        # Only the independent forecast scene is written. Never unpack or
        # reset the committed scene: pack does not establish exact replay of
        # internal contact history, as the recorded first-step failure shows.
        if source.scene.physx_system is destination.scene.physx_system:
            raise RuntimeError('forecast_scene_not_independent')
        before=source.scene.physx_system.pack()
        def controls(environment):
            return [(j.get_stiffness(),j.get_damping(),j.get_force_limit(),j.get_drive_mode(),
                     j.get_drive_target().tolist(),j.get_drive_velocity_target().tolist())
                    for a in ('left','right') for j in getattr(environment.robot,a+'_entity').get_active_joints()]
        before_controls=controls(source)
        from sleep_state_copy import ObjectSleepCopy
        sleep_copy=ObjectSleepCopy(source.objects,destination.objects,sapien.pysapien.physx.PhysxRigidDynamicComponent)
        for arm in ('left','right'):
            src=getattr(source.robot,arm+'_entity');dst=getattr(destination.robot,arm+'_entity')
            dst.set_root_pose(src.get_root_pose());dst.set_qpos(src.get_qpos());dst.set_qvel(src.get_qvel())
            sj=src.get_active_joints();dj=dst.get_active_joints()
            if [j.get_name() for j in sj]!=[j.get_name() for j in dj]:raise RuntimeError('forecast_joint_layout_mismatch')
            for original,copied in zip(sj,dj):
                copied.set_drive_property(original.get_stiffness(),original.get_damping(),original.get_force_limit(),original.get_drive_mode())
                copied.set_drive_target(float(original.get_drive_target()[0]))
                copied.set_drive_velocity_target(float(original.get_drive_velocity_target()[0]))
        for n,actor in source.objects.items():
            twin=destination.objects[n]
            twin.actor.set_pose(actor.get_pose())
            cls=sapien.pysapien.physx.PhysxRigidDynamicComponent
            original=actor.actor.find_component_by_type(cls);copied=twin.actor.find_component_by_type(cls)
            copied.set_linear_velocity(original.get_linear_velocity());copied.set_angular_velocity(original.get_angular_velocity())
            if not np.array_equal(copied.get_linear_velocity(),original.get_linear_velocity()) or not np.array_equal(copied.get_angular_velocity(),original.get_angular_velocity()):
                raise RuntimeError('forecast_velocity_copy_mismatch')
        sleep_copy.restore()
        if state(source)!=state(destination):raise RuntimeError('forecast_state_copy_mismatch')
        if controls(destination)!=before_controls:raise RuntimeError('forecast_control_copy_mismatch')
        def check_source_unchanged():
            sleep_copy.assert_source_unchanged()
            if source.scene.physx_system.pack()!=before:raise RuntimeError('forecast_modified_source_scene')
            if controls(source)!=before_controls:raise RuntimeError('forecast_modified_source_controls')
        check_source_unchanged()
        return check_source_unchanged
    @timing.measured('forecast')
    def preview_separation(arm,target,candidate):
        event('separation_actuator_source',arm=arm,object_id=target,caller_phase=phase,returning=returning,candidate=copy.deepcopy(candidate))
        from forecast_sleep_hypotheses import initialization_modes
        modes=initialization_modes(env.objects,sapien.pysapien.physx.PhysxRigidDynamicComponent)
        if modes==('exact',):return nominal_preview_separation(arm,target,candidate)
        if not live_sleep.permitted(env,target):raise RuntimeError('ambiguous_copy_outside_live_grasp_scope')
        with live_sleep.qualify(env,separation_scene,candidate,arm,target,
                                lambda:strict_control_copy(env,separation_scene)) as qualification:
            accepted=paired_preview_separation(arm,target,candidate)
        if accepted is not (qualification['status']=='QUALIFIED_FORECAST_PAIR'):
            raise RuntimeError('live_return_outcome_mismatch')
        # A complete physical rejection returns to the existing bounded offset
        # search. qualify() must finish evidence/nonmutation checks first.
        return accepted
    @timing.measured('plan')
    def plan_to(arm,p,r,target,held_relative=None,allow_plan_failure=False,joint_goal=None,allow_target_fingers=True,careful=False,branch_seed=False,certified_retreat=False):
        nonlocal plans
        current=state();world=collision_world(current)
        if held_relative is not None:world.held=(arm,str(target),held_relative)
        q0=current['robots'][arm]['q'][:7]
        # A preflight IK solution belongs to its exact Cartesian target.
        # Validate it without random restarts; still replan/check the path
        # from the measured state and check physical endpoint tracking.
        ik=planner.solve_ik(model,arm,p,r,q0 if joint_goal is None else joint_goal,
            rng=rng,max_iters=120 if joint_goal is None else 0,restarts=2 if joint_goal is None else 0)
        branch_attempted=(not ik['success'] and branch_seed and cfg.high_transfer_branch_seed
                and held_relative is not None and joint_goal is None)
        if branch_attempted:
            from branch_seeds import fallback_ik
            ik=fallback_ik(ik,model,arm,p,r,rng,event)
        if not ik['success']:
            event('ik_rejected',target=target,position=np.asarray(p).tolist(),rotation=np.asarray(r).tolist(),result=ik)
            if allow_plan_failure:return False
            raise CandidateInfeasible('custom_ik_failed')
        plans+=1
        plan=planner.plan_joint_path(q0,ik['q'],bounds,lambda q:world.free(arm,q,target,allow_target_fingers,
            np.clip(current['robots'][arm]['q'][-2:],0.,.04).tolist()),rng=rng,max_nodes=80,resolution=.025)
        event('custom_plan',ik=ik,path=plan,target=target,arm=arm,
            caller=dict(state=current,phase=phase,returning=returning,desired_position=np.asarray(p).tolist(),desired_rotation=np.asarray(r).tolist(),
                held_relative=None if held_relative is None else np.asarray(held_relative).tolist(),allow_target_contact=allow_target_fingers))
        if branch_attempted:
            event('branch_seed_path',arm=arm,target=target,position=np.asarray(p).tolist(),rotation=np.asarray(r).tolist(),
                measured_start=q0,solved_q=ik['q'],path=plan,acceptance=False)
        if not plan['success']:
            if allow_plan_failure:return False
            raise CandidateInfeasible('custom_path_failed:'+plan['reason'])
        if certified_retreat:
            if held_relative is not None:raise RuntimeError('extraction_retreat_held')
            postrelease_motion(arm,target,'retreat',plan['path'])
        else:
            execute(arm,plan['path'],target,held_relative is not None,allow_target_contact=allow_target_fingers,
                    smooth=careful,max_velocity=.1 if careful else .25,actuator_world=world)
        def tracking():
            actual=pose_matrix(env.get_arm_pose(arm));entity=getattr(env.robot,arm+'_entity')
            fk=model.evaluate(arm,entity.get_qpos()[:7].tolist())
            return dict(position_error_m=float(np.linalg.norm(actual[:3,3]-p)),
                rotation_error_rad=float(np.linalg.norm(planner.so3_log(np.asarray(r)@actual[:3,:3].T))),
                q=entity.get_qpos().tolist(),qdot=entity.get_qvel().tolist(),
                actual_ee=actual.tolist(),target_position=np.asarray(p).tolist(),target_rotation=np.asarray(r).tolist(),
                fk_position_error_m=float(np.linalg.norm(np.asarray(fk['ee_position_m'])-actual[:3,3])),
                object_pose=state()['objects'][str(target)]['pose'],contacts=contacts())
        measure=tracking();event('physical_tracking',target=target,extra_hold_steps=0,**measure)
        # A bounded extra hold uses the same joint target and unchanged pose
        # thresholds. Every physics step retains the contact guard.
        for extra in range(50,1001,50):
            if measure['position_error_m']<=.005 and measure['rotation_error_rad']<=.04:break
            if certified_retreat:raise CandidateInfeasible('certified_retreat_cartesian_endpoint')
            for _ in range(50):tick(target,held_relative is not None,allow_target_fingers)
            measure=tracking();event('physical_tracking',target=target,extra_hold_steps=extra,**measure)
        if measure['position_error_m']>.005 or measure['rotation_error_rad']>.04:
            raise CandidateInfeasible('physical_ee_pose_not_settled')
        return True
    @timing.measured('grasp')
    def execute_grasp(arm,target,candidate):
        nonlocal drive,held_id,continuation_guard
        from capture_handoff import preflight as capture_preflight
        capture_preflight(env,arm,target,candidate,state,event,
            dict(held_id=held_id,drive_active=drive is not None,preview=preview_mode,
                 formal_step=step,preview_step=preview_total_steps,
                 lifecycle=preview_accounting.context()['lifecycle']))
        live_sleep.grasp_begin(env)
        if cfg.defer_intermediate_home and returning:
            from return_motion import RetainedHold
            if continuation_guard is not None:raise RuntimeError('unclosed_return_handoff')
            other='left' if arm=='right' else 'right'
            continuation_guard=RetainedHold(continuation_api(),(other,),target,'continuation')
            timing.call('continuation',continuation_guard.check,'admission')
        actor=env.objects[target]
        p=np.array(candidate['ee_position_m']);r=candidate['ee_rotation_world']
        # Keep the already validated IK branches. plan_to revalidates the exact
        # Cartesian goals and replans from measured joints in every scene.
        plan_to(arm,p+[0,0,.1],r,target,allow_target_fingers=False,joint_goal=candidate['pregrasp_q'])
        plan_to(arm,p,r,target,allow_target_fingers=False,careful=True,joint_goal=candidate['grasp_q'])
        # Experimental 017: adaptive physical capture with unchanged final dwell.
        entity=getattr(env.robot,arm+'_entity');active=entity.get_active_joints()
        fingers=[j[0] for j in getattr(env.robot,arm+'_gripper')]
        from capture_native import run as capture_run,contact_points
        from capture_feedback import raw_touch
        def closure_measure(validate_contact=True):
            actual=pose_matrix(env.get_arm_pose(arm))
            return dict(finger_velocity=[float(entity.get_qvel()[active.index(j)]) for j in fingers],
                finger_position=[float(entity.get_qpos()[active.index(j)]) for j in fingers],
                bilateral_contact=all(raw_touch(contact_points(env.scene,fingers,actor.actor))) if validate_contact else False,
                position_error_m=float(np.linalg.norm(actual[:3,3]-p)),
                rotation_error_rad=float(np.linalg.norm(planner.so3_log(np.asarray(r)@actual[:3,:3].T))))
        capture_result=capture_run(env=env,arm=arm,target=target,candidate=candidate,state=state,
            world=collision_world,measurement=lambda:closure_measure(False),tick=lambda:tick(target,False),
            guard=lambda:contact_guard(target,False),event=event,phase=phase,returning=returning,
            context=lambda:dict(held_id=held_id,drive_active=drive is not None,preview=preview_mode,
                formal_step=step,preview_step=preview_total_steps,lifecycle=preview_accounting.context()['lifecycle']))
        held_fingers={j.child_link.get_name():value for j,value in zip(fingers,capture_result['first_latches'])}
        event('gripper_hold',object_id=target,arm=arm,contact_targets=held_fingers,
              contact_targets_semantics='original_contact_latch',
              retained_targets=[float(j.get_drive_target()[0]) for j in fingers],
              drive_force_limit_raw=4.,closure_target_speed_m_s=.01,contact_preload_m=.00025,
              finger_velocity=[float(entity.get_qvel()[active.index(j)]) for j in fingers])
        event('after_closure',object_id=target,arm=arm,state=state(),contacts=contacts())
        target_name=actor.actor.get_name()
        # Relieve only an unsafe supported retained target BEFORE attachment.
        # Same real actions and checks in preview and formal scenes.
        from supported_handoff import run as supported_handoff
        from loaded_common import controls as supported_controls
        from types import SimpleNamespace
        def supported_observe():
            return dict(state=state(),controls=supported_controls(env),
                native_ee=pose_matrix(env.get_arm_pose(arm)).tolist(),
                measurement=closure_measure(),contacts=contacts(),held_id=held_id,drive_active=drive is not None)
        supported_handoff(SimpleNamespace(observe=supported_observe,world=collision_world,
            model=model,robot=env.robot,phase=phase,returning=returning,dt=env.scene.get_timestep(),
            guard=lambda:contact_guard(target,False),tick=lambda:tick(target,False),emit=event),
            arm,target,digest(candidate))
        # No attachment after a lost supported-handoff contact.
        final_capture=closure_measure()
        if (not np.isfinite([*final_capture['finger_position'],*final_capture['finger_velocity'],final_capture['position_error_m'],final_capture['rotation_error_rad']]).all()
            or not final_capture['bilateral_contact'] or max(map(abs,final_capture['finger_velocity']))>.01
            or final_capture['position_error_m']>.005 or final_capture['rotation_error_rad']>.04):
            raise CandidateInfeasible('capture_pre_magnet_gate')
        if not contact_guard(target,False):raise CandidateInfeasible('no_target_contact_for_magnet')
        drive=env._magnetic_attach(arm,actor,target);held_id=target
        actual=env.get_arm_pose(arm);ee=pose_matrix(actual)
        relative=np.linalg.inv(ee)@actor.get_pose().to_transformation_matrix()
        event('held_transform_source',object_id=target,arm=arm,state=state(),ee_transform=ee.tolist(),
            object_transform=actor.get_pose().to_transformation_matrix().tolist(),relative=relative.tolist(),origin='magnet_attachment')
        event('magnet_attached',object_id=target,raw_drive_limit=80)
        start_world=collision_world(state());start_world.held=(arm,str(target),relative)
        joints=getattr(env.robot,arm+'_entity').get_qpos().tolist()
        initial_contacts={}
        for n,obj in env.objects.items():
            if n==target:continue
            pairs=[c for c in contacts() if target_name in c['bodies'] and obj.actor.get_name() in c['bodies']]
            if pairs:
                initial_contacts[str(n)]=dict(separation_min=min(c['separation_min'] for c in pairs),
                                             impulse_max=max(c['impulse_max'] for c in pairs))
        separate=plan_separation(model,start_world,arm,joints[:7],target,np.clip(joints[-2:],0.,.04).tolist(),rng=rng,initial_contacts=initial_contacts,
            validate_candidate=lambda c:preview_separation(arm,target,c))
        event('separation_plan',object_id=target,arm=arm,caller_phase=phase,returning=returning,**separate)
        if not separate['success']:raise CandidateInfeasible('separation_failed:'+separate['reason'])
        if separate['needed']:
            execute(arm,separate['path'],target,True,smooth=True,separation_source=separate,**separate['execution_profile'])
            relative,measurement=settle_separation(arm,target,separate)
            event('held_transform_source',object_id=target,arm=arm,state=state(),ee_transform=pose_matrix(env.get_arm_pose(arm)).tolist(),
                object_transform=actor.get_pose().to_transformation_matrix().tolist(),relative=relative.tolist(),origin='settled_separation')
            event('separation_measured',object_id=target,**measurement)
        # No replayed grasp/path. Targets are explicit geometric waypoints;
        # all inverse and trajectory calculations use the new algorithms.
        lift_start=np.asarray(env.get_arm_pose(arm)[:3]);lift_amount=max(.10,.94-payload_bottom(target))
        for rise in np.linspace(0,lift_amount,int(np.ceil(lift_amount/.04))+1)[1:]:
            plan_to(arm,lift_start+[0,0,rise],r,target,relative)
        event('lifted',object_id=target,bottom_z=float(payload_bottom(target)))
        if not returning and loaded_variant=='B_clearance':
            from actuator_targets import handoff
            from loaded_common import controls as actuator_controls
            admission=handoff(model,arm,state(),actuator_controls(env))
            event('grasp_actuator_handoff',object_id=target,receipt=admission)
        return relative,lift_start,lift_amount,r
    @preview_accounting.wrap('grasp')
    @timing.measured('preview')
    def preview_grasp_once(arm,target,candidate,index,trial):
        nonlocal env,preview_mode,drive,held_id,plans,grasp_preview_attempts,preview_trace
        if drive is not None or held_id is not None:raise RuntimeError('grasp_preview_requires_empty_gripper')
        source=env;check_unchanged=copy_to_shadow(source,grasp_scene)
        env=grasp_scene;rng_state=copy.deepcopy(rng.bit_generator.state);before_plans=plans
        before_steps=preview_total_steps
        preview_trace=[];preview_mode=True;failure=None;stopped=None;trial_contacts=None;bottom=None
        grasp_preview_attempts+=1
        try:
            execute_grasp(arm,target,candidate)
            bottom=float(payload_bottom(target))
        except CandidateInfeasible as exc:
            failure=str(exc);stopped=state();trial_contacts=contacts()
        finally:
            if drive is not None:env._magnetic_detach(drive)
            drive=None;held_id=None
            env=source;check_unchanged();rng.bit_generator.state=rng_state;plans=before_plans
            preview_mode=False
        event('grasp_preview',object_id=target,index=index,trial=trial,accepted=failure is None,
              failure=failure,trial_steps=preview_total_steps-before_steps,lift_bottom_z=bottom,
              separation_trials=preview_trace,source_unchanged=True,independent_scene=True,failure_state=stopped,
              failure_contacts=trial_contacts)
        return failure is None
    @timing.measured('forecast')
    def preview_grasp(arm,target,candidate,index):
        # A single forecast admitted N18 despite a different closed-contact
        # outcome in the committed scene. Require repeatability from the same
        # copied source state before committing. Each trial restores only the
        # independent shadow scene; the source is checked bitwise unchanged.
        for trial in (1,2):
            if not preview_grasp_once(arm,target,candidate,index,trial):return False
        event('grasp_candidate_verified',object_id=target,index=index,independent_trials=2)
        return True
    @timing.measured('placement')
    def insert_to(arm,p,r,target,relative):
        nonlocal plans
        current=state();world=collision_world(current);world.held=(arm,str(target),relative)
        q0=np.asarray(current['robots'][arm]['q'][:7]);finger=np.clip(current['robots'][arm]['q'][-2:],0.,.04).tolist()
        ik=planner.solve_ik(model,arm,p,r,q0,rng=rng,max_iters=120,restarts=0)
        if not ik['success']:raise CandidateInfeasible('insertion_ik_failed')
        q1=np.asarray(ik['q']);n=max(2,int(np.ceil(np.max(np.abs(q1-q0))/.01))+1)
        # FK requires seven native Python floats. list(ndarray) preserves
        # np.float64 scalars and fails that contract; keep measured state 7+2.
        path=np.linspace(q0,q1,n).tolist()
        if not all(world.free(arm,q,target,True,finger) for q in path):
            raise CandidateInfeasible('insertion_collision:'+str(world.last_collision))
        plans+=1;event('insertion_plan',object_id=target,arm=arm,path=path,
            caller=dict(state=current,phase=phase,returning=returning,desired_position=np.asarray(p).tolist(),desired_rotation=np.asarray(r).tolist(),held_relative=np.asarray(relative).tolist(),allow_target_contact=True))
        from return_diagnostics import insertion_measurement,tracking_ok
        def measure(extra):
            entity=getattr(env.robot,arm+'_entity')
            controls=[dict(name=j.get_name(),target=j.get_drive_target().tolist(),
                           velocity_target=j.get_drive_velocity_target().tolist(),
                           stiffness=j.get_stiffness(),damping=j.get_damping(),force_limit=j.get_force_limit())
                      for j in entity.get_active_joints()]
            report=insertion_measurement(model,arm,p,r,q1,env.get_arm_pose(arm),state(),target,
                relative,goal_poses[str(target)],contacts(),controls)
            event('insertion_tracking',object_id=target,arm=arm,extra_hold_steps=extra,**report)
            return report
        try:
            execute(arm,path,target,True,smooth=True,max_velocity=.08,actuator_world=world)
            report=measure(0)
            # Same target, same tolerances, same per-step contact guard as transit.
            # Diagnostic legacy mode isolates the old immediate rejection.
            hold_limit=0 if diagnostic and diagnostic['legacy_insertion'] else 1000
            for extra in range(50,hold_limit+1,50):
                if tracking_ok(report):break
                for _ in range(50):tick(target,True)
                report=measure(extra)
            if not tracking_ok(report):raise CandidateInfeasible('insertion_tracking_failed')
        except CandidateInfeasible as exc:
            event('insertion_failure',object_id=target,arm=arm,reason=str(exc),measurement=measure(None))
            raise
    def release_return(arm,target):
        nonlocal drive,held_id
        env._release_magnetic(drive,env.objects[target]);drive=None;held_id=None
        if continuation_guard is not None:continuation_guard.released()
        env.robot.set_gripper(1.,arm,gripper_eps=0)
        for _ in range(250):tick(target,False)
    def home_return(arm,target):
        postrelease_motion(arm,target,'home',homes[arm])
    def retreat_return(arm,target):
        postrelease_motion(arm,target,'retreat')
    def governor_guard(target,allow_target_contact):
        if return_deadline is not None and time.monotonic()>=return_deadline:
            raise CandidateInfeasible('return_continuation_time_cap')
        contact_guard(target,False,allow_target_contact)
        if continuation_guard is not None:timing.call('continuation',continuation_guard.check,'governor',False)
    if continuous_version==36:
        from home_cpu_profile_v36 import profile_first_actual_home
    elif continuous_version==35:
        from home_cpu_profile_v35 import profile_first_actual_home
    elif continuous_version==34:
        from home_cpu_profile_v34 import profile_first_actual_home
    elif continuous_version==33:
        from home_cpu_profile_v33 import profile_first_actual_home
    elif continuous_version==32:
        from home_cpu_profile_v32 import profile_first_actual_home
    elif continuous_version==31:
        from home_cpu_profile_v31 import profile_first_actual_home
    elif continuous_version==30:
        from home_cpu_profile_v30 import profile_first_actual_home
    elif continuous_version==29:
        from home_cpu_profile_v29 import profile_first_actual_home
    elif continuous_version==28:
        from home_cpu_profile_v28 import profile_first_actual_home
    elif continuous_version==27:
        from home_cpu_profile_v27 import profile_first_actual_home
    elif continuous_version==26:
        from home_cpu_profile_v26 import profile_first_actual_home
    else:
        from home_cpu_profile import profile_first_actual_home
    @profile_first_actual_home(enabled=cfg.home_cpu_profile,max_calls=cfg.home_cpu_profile_max_calls,output=out,
        context=lambda:dict(preview=preview_mode,phase=phase,step=step),
        binding=lambda:dict(source_sha256=profile_binding['source_sha256'],implementation_sha256=profile_binding['implementation_sha256'],
            config_sha256=file_hash(args.config),bindings_sha256=file_hash(root.parent/f'harness/configs/v{continuous_version}/BINDINGS.json')))
    @timing.measured('unloaded')
    def postrelease_motion(arm,target,motion_phase,home=None):
        # Retain the four-argument profiler protocol. For retreat only, the
        # fourth argument is the already planned route, never a HOME target.
        supplied_path=home if motion_phase=='retreat' else None
        if supplied_path is not None:home=None
        from return_motion import run_motion,motion_readback
        def controls():
            return {a:[dict(name=j.get_name(),stiffness=j.get_stiffness(),damping=j.get_damping(),
                           force_limit=j.get_force_limit(),mode=str(j.get_drive_mode()),
                           target=j.get_drive_target().tolist(),velocity_target=j.get_drive_velocity_target().tolist())
                       for j in getattr(env.robot,a+'_entity').get_active_joints()] for a in ('left','right')}
        def contact_summary():
            return [dict(bodies=[b.entity.get_name() for b in c.bodies],
                separation_min=min(float(p.separation) for p in c.points),
                impulse_max=max(float(np.linalg.norm(p.impulse)) for p in c.points))
                for c in env.scene.get_contacts() if c.points and any('panda' in b.entity.get_name() for b in c.bodies)]
        api=types.SimpleNamespace(state=state,world=collision_world,model=model,rng=rng,event=event,
            controls=controls,contacts=contacts,contact_summary=contact_summary,tick=tick,
            retreat_time_scale=cfg.retreat_time_scale,
            vertex_governor=cfg.vertex_governor,governor_guard=governor_guard,
            joint_step_refinement=cfg.joint_step_refinement,home_coarse=cfg.home_coarse,home_progress=cfg.home_progress,home_progress_bound=cfg.home_progress_bound,home_entry_hold=cfg.home_entry_hold,
            home_nominal_speed_rad_s=cfg.home_nominal_speed_rad_s,
            velocity_feedback=cfg.velocity_feedback,feedback_scope=cfg.feedback_scope,feedback_telemetry=cfg.feedback_telemetry,
            feedback_metadata=lambda:__import__('feedback_telemetry').metadata(env),
            telemetry=lambda:motion_readback(env,preview_total_steps if preview_mode else step),
            prefix_sha256=digest(preview_trace) if preview_mode else None,
            ee_pose=lambda a:env.get_arm_pose(a),
            command=lambda a,q,v:__import__('actuator_targets').issue(env.robot,a,q,v,bounds))
        if diagnostic and diagnostic.get('fixed_setpoint_diagnostic') is not None and motion_phase=='retreat':
            if not preview_mode:raise RuntimeError('fixed_setpoint_requires_shadow')
            api.independent_diagnostic=True
            api.fixed_setpoint_diagnostic=diagnostic['fixed_setpoint_diagnostic']
        if cfg.home_contact_envelope and motion_phase=='home':
            if not continuous and (not diagnostic or not preview_mode):raise RuntimeError('home_contact_requires_shadow')
            from home_interaction import HomeInteractionAPI
            try:api=HomeInteractionAPI(api,env,arm,target)
            except Exception as exc:
                event('home_contact_rejected',arm=arm,object_id=target,motion_phase='home',
                      reason=type(exc).__name__+':'+str(exc),stage='metadata_admission',formal_credit=False)
                raise
        if diagnostic and diagnostic.get('n17_observer') and motion_phase=='home':
            if not preview_mode:raise RuntimeError('n17_observer_requires_shadow')
            from n17_observer import observe_motion
            observe_motion(api,env,diagnostic['n17_observer'],run_motion,arm,target,motion_phase,home)
        elif supplied_path is not None:
            if motion_phase!='retreat':raise RuntimeError('supplied_route_scope')
            from return_motion import run_supplied_retreat
            run_supplied_retreat(api,arm,target,supplied_path)
        else:
            run_motion(api,arm,target,motion_phase,home)
    def settle_return(target,count):
        nonlocal continuation_guard
        if not cfg.defer_intermediate_home:
            for _ in range(count):tick(target,False,False)
            return
        from return_motion import RetainedHold
        if continuation_guard is not None:continuation_guard.finish()
        continuation_guard=RetainedHold(continuation_api(),('left','right'),target,'settle')
        try:
            timing.call('continuation',continuation_guard.check,'admission')
            for _ in range(count):tick(target,False,False)
            continuation_guard.finish()
        finally:continuation_guard=None
    def continuation_api():
        from return_motion import motion_readback
        def controls():
            return {a:[dict(name=j.get_name(),stiffness=j.get_stiffness(),damping=j.get_damping(),
                force_limit=j.get_force_limit(),mode=str(j.get_drive_mode()),target=j.get_drive_target().tolist(),
                velocity_target=j.get_drive_velocity_target().tolist())
                for j in getattr(env.robot,a+'_entity').get_active_joints()] for a in ('left','right')}
        return types.SimpleNamespace(state=state,world=collision_world,model=model,rng=rng,event=event,
            controls=controls,contacts=contacts,contact_summary=contacts,tick=tick,
            vertex_governor=cfg.vertex_governor,governor_guard=governor_guard,
            joint_step_refinement=cfg.joint_step_refinement,home_coarse=cfg.home_coarse,home_progress=cfg.home_progress,home_progress_bound=cfg.home_progress_bound,home_entry_hold=cfg.home_entry_hold,
            home_nominal_speed_rad_s=cfg.home_nominal_speed_rad_s,
            velocity_feedback=cfg.velocity_feedback,feedback_scope=cfg.feedback_scope,feedback_telemetry=cfg.feedback_telemetry,
            feedback_metadata=lambda:__import__('feedback_telemetry').metadata(env),
            telemetry=lambda:motion_readback(env,preview_total_steps if preview_mode else step),
            ee_pose=lambda a:env.get_arm_pose(a),command=lambda a,q,v:__import__('actuator_targets').issue(env.robot,a,q,v,bounds))
    def return_budget(deadline):
        nonlocal return_deadline
        return_deadline=deadline
    @preview_accounting.wrap('return')
    @timing.measured('preview')
    def preview_return(arm,target,candidate,place,index,trial,shadow=None):
        nonlocal env,preview_mode,drive,held_id,plans,grasp_preview_attempts,preview_trace,continuation_guard
        if preview_mode:raise RuntimeError('nested_return_preview_forbidden')
        if drive is not None or held_id is not None:raise RuntimeError('return_preview_requires_empty_gripper')
        destination=grasp_scene if shadow is None else shadow
        source=env;check_unchanged=copy_to_shadow(source,destination)
        previous_guard=continuation_guard;continuation_guard=None
        env=destination;before_rng=copy.deepcopy(rng.bit_generator.state);before_plans=plans
        before_steps=preview_total_steps;preview_mode=True;error=None;fatal=None;diagnostic={};grasp_preview_attempts+=1
        trial_started=time.monotonic()
        preview_trace=[]
        try:
            execute_grasp(arm,target,candidate);place()
        except Exception as exc:
            error=type(exc).__name__+':'+str(exc);diagnostic=exception_diagnostic(exc)
            diagnostic['failure_state']=state()
            diagnostic['failure_contacts']=contacts()
            if not diagnostic['retryable'] and diagnostic['failure_category'] not in ('diagnostic_completed','diagnostic_guard_failed'):fatal=exc
        finally:
            if continuation_guard is not None:continuation_guard.finish(error)
            if drive is not None:env._magnetic_detach(drive)
            drive=None;held_id=None;env=source;preview_mode=False
            continuation_guard=previous_guard
            rng.bit_generator.state=before_rng;plans=before_plans;check_unchanged()
        event('return_preview',object_id=target,index=index,trial=trial,accepted=error is None,
            **(diagnostic|dict(failure=error)),trial_steps=preview_total_steps-before_steps,source_unchanged=True,independent_scene=True,
            trace=preview_trace,arm=arm,trial_elapsed_s=time.monotonic()-trial_started)
        if fatal is not None:raise fatal
        return error is None
    if diagnostic:
        from return_reconstruction import reconstruct,check_bundle_unchanged,diagnostic_result
        from return_candidates import goal_conditioned
        bundle=diagnostic['bundle'];report=None;diagnostic_failure=None;rows=[];source_state=None;source_pack=None
        # This branch exits before the formal episode's extraction loop/ledger.
        # All physical motion below occurs in the existing independent shadows.
        try:
            from run_config import validate_diagnostic_config
            validate_diagnostic_config(cfg,bundle['config'])
            if drive is not None or held_id is not None:raise RuntimeError('diagnostic_active_drive')
            if manifest['release_helpers_sha256']!=bundle['manifest']['release_helpers_sha256']:
                raise RuntimeError('diagnostic_scene_helpers_mismatch')
            report=reconstruct(bundle,env,sapien,state,physical_meshes)
            report['robot_frames']={};report['reconstructed_controls']={}
            for arm in ('left','right'):
                entity=getattr(env.robot,arm+'_entity');fk=model.evaluate(arm,state()['robots'][arm]['q'][:7])
                actual=pose_matrix(env.get_arm_pose(arm));root_pose=entity.get_root_pose()
                frame=dict(fk_position_error_m=float(np.linalg.norm(np.asarray(fk['ee_position_m'])-actual[:3,3])),
                           fk_rotation_max_error=float(np.max(np.abs(np.asarray(fk['ee_rotation_world'])-actual[:3,:3]))),
                           asset_defined_root_pose=root_pose.p.tolist()+root_pose.q.tolist())
                report['robot_frames'][arm]=frame
                report['reconstructed_controls'][arm]=[dict(name=j.get_name(),stiffness=j.get_stiffness(),
                    damping=j.get_damping(),force_limit=j.get_force_limit(),mode=str(j.get_drive_mode()),
                    target=j.get_drive_target().tolist(),velocity_target=j.get_drive_velocity_target().tolist())
                    for j in entity.get_active_joints()]
                if frame['fk_position_error_m']>1e-5 or frame['fk_rotation_max_error']>1e-5:
                    report['errors'].append('live_fk_frame_mismatch:'+arm);report['validated']=False
            write_once(out/'reconstruction.json',report)
            if not report['validated']:raise RuntimeError('diagnostic_reconstruction_failed')
            manifest=bundle['manifest'];homes=bundle['final']['homes']
            support_map=manifest['supports'];goal_poses={i:o['pose'] for i,o in manifest['objects'].items()}
            returning=True;phase='return_diagnostic'
            api=types.SimpleNamespace(state=state,ee_pose=lambda arm:env.get_arm_pose(arm),meshes=physical_meshes,
                model=model,rng=rng,world=collision_world,plan_to=plan_to,insert=insert_to,
                release=release_return,retreat=retreat_return,home=home_return,settle=settle_return,event=event,rank=rank_candidates,
                grasp=execute_grasp,return_budget=return_budget)
            pipeline=ReturnPipeline(api,manifest,cfg)
            if pair_probe:
                from pose_goals import insertion_order
                selected_ids=set(map(str,manifest['selected_ids']))
                already_returned=set(map(str,bundle['final']['returned_ids']))
                pair_order=insertion_order(manifest['dependencies'],selected_ids-already_returned,
                    (set(manifest['objects'])-selected_ids)|already_returned)
                if len(pair_order)<2 or pair_order[0]!='12':raise ValueError('diagnostic_dependency_prefix')
                # Explicit caps also apply to the HOME comparison diagnostic.
                pipeline.start_budget()
                event('diagnostic_pair_request',order=pair_order,preference=pair_probe,
                    defer_intermediate_home=cfg.defer_intermediate_home,independent_systems=2,
                    source_state_sha256=digest(state()),source_controls_sha256=digest(report['reconstructed_controls']),
                    source_rng_sha256=digest(rng.bit_generator.state),
                    formal_credit=False,nested_return_previews=False)
            source_state=copy.deepcopy(state());source_pack=env.scene.physx_system.pack()
            target=bundle.get('target',10)
            def diagnostic_candidates():
                if diagnostic['candidate_index'] is not None:
                    index=diagnostic['candidate_index']
                    from diagnostic_candidates import select_candidate
                    arm,c,provenance=select_candidate(bundle,index,model)
                    event('diagnostic_candidate',object_id=target,index=index,candidate=c,provenance=provenance)
                    p=np.asarray(c['ee_position_m']);r=c['ee_rotation_world']
                    q=state()['robots'][arm]['q'][:7]
                    ik=planner.solve_ik(model,arm,p+[0,0,.1],r,q,rng=rng,max_iters=100,restarts=1)
                    if not ik['success']:raise CandidateInfeasible('diagnostic_recorded_candidate_pregrasp_ik_failed')
                    grasp=planner.solve_ik(model,arm,p,r,ik['q'],rng=rng,max_iters=100,restarts=1)
                    if not ik['success'] or not grasp['success']:raise CandidateInfeasible('diagnostic_recorded_candidate_ik_failed')
                    yield arm,index,dict(c,pregrasp_q=ik['q'],grasp_q=grasp['q'])
                else:
                    for arm in ('right','left'):
                        for index,c in goal_conditioned(api,manifest,arm,target,pipeline.candidates_for(arm,target)):
                            yield arm,index,c
            for arm,index,c in diagnostic_candidates():
                accepted_trials=[]
                for trial in (1,2):
                    if len(rows)>=diagnostic['max_previews']:break
                    if pair_probe:
                        passed=preview_return(arm,target,c,
                            lambda:pipeline.diagnostic_next(arm,target,pair_order,pair_probe),index,trial,
                            shadow=grasp_scene if trial==1 else continuation_scene)
                    else:passed=preview_return(arm,target,c,lambda:pipeline.place(arm,target),index,trial)
                    row=dict(arm=arm,index=index,trial=trial,preview_completed=passed)
                    if pair_probe:
                        row.update(pair_completed=any(e['kind']=='diagnostic_pair_completed' for e in preview_trace),
                            completed_return_ids=[e['object_id'] for e in preview_trace if e['kind'] in
                                ('diagnostic_first_return_completed','diagnostic_second_return_completed')],
                            actual_relations=[e['actual_relation'] for e in preview_trace if e['kind']=='diagnostic_next_selected'],
                            independent_shadow=trial,formal_credit=False)
                    if diagnostic.get('fixed_setpoint_diagnostic') is not None:
                        holds=[e for e in preview_trace if e['kind'] in ('fixed_setpoint_hold','feedback_probe')]
                        row['diagnostic_outcome']=holds[-1]['outcome'] if holds else 'probe_not_reached'
                    rows.append(row)
                    accepted_trials.append(passed)
                    if not passed and not pair_probe:break
                if accepted_trials==[True,True] or len(rows)>=diagnostic['max_previews']:break
            if state()!=source_state or env.scene.physx_system.pack()!=source_pack:
                raise RuntimeError('diagnostic_source_modified')
        except Exception as exc:
            diagnostic_failure=exception_diagnostic(exc);traceback.print_exc()
        finally:
            check_bundle_unchanged(bundle)
            result=diagnostic_result(reconstruction_validated=bool(report and report['validated']),
                failure=diagnostic_failure,previews=rows,source_sha256=source_files(root),
                source_fingerprint=bundle['source_fingerprint'],input_sha256=bundle['hashes'],
                legacy_insertion=diagnostic['legacy_insertion'],physical_preview_steps=preview_total_steps,
                source_unchanged=bool(report and report['validated'] and state()==source_state and env.scene.physx_system.pack()==source_pack),
                runtime_llm_calls=0,development_usage=None)
            write_once(out/'diagnostic-result.json',result)
            writer.close();log.close();trace.close()
        # Zero means a diagnostic finished, never task success. Failed previews
        # remain visible and cannot enter experiment.py's acceptance route.
        return 0 if diagnostic_failure is None else 2
    try:
        # Verify actual live frames before any plan; never calibrate to a target.
        for arm in ('left','right'):
            q=state()['robots'][arm]['q'][:7]
            fk=model.evaluate(arm,q);actual=pose_matrix(env.get_arm_pose(arm))
            if np.linalg.norm(np.array(fk['ee_position_m'])-actual[:3,3])>1e-5 or np.max(np.abs(np.asarray(fk['ee_rotation_world'])-actual[:3,:3]))>1e-5:
                raise RuntimeError('live_fk_frame_mismatch')
        homes={a:state()['robots'][a]['q'][:7] for a in ('left','right')}
        if expected_layout is not None:
            from provenance import verify_selected_scene
            verify_selected_scene(manifest,expected_layout)
        if not validation['valid']:raise RuntimeError('invalid_generated_or_original_layout:'+str(validation['errors']))
        reservation=None
        if cfg.reserve_home_workspace:
            from home_reservation import Reservation
            reservation=Reservation(env,collision_world,state(),homes,event)
            if not reservation.check(state(),None,'initial'):
                raise CandidateInfeasible('initial_HOME_workspace_not_admissible')
        switch_phase('extraction')
        targets=sorted(selected,key=lambda n:-env._top_z(env.objects[n]))
        if hasattr(experience,'rank_objects'):
            targets=experience.rank_objects(targets,initial,{oid:o['geometry_sha256'] for oid,o in identity.items()})
            event('experience_object_ranking',**experience.last_object_trace)
        search=CandidateSearch(targets)
        while len(completed)<args.max_objects:
            target=search.next_target()
            if target is None:break
            actor=env.objects[target];s=state();o=s['objects'][str(target)]
            arm='right' if o['pose'][0]>=0 else 'left'
            # Preserve the established search tier, then offer shallower entries
            # when the open fingers cannot descend without pushing the object.
            candidates=[]
            for depths in ((.014,.025,.04,.06),(.008,),(.004,),(.001,),(.0005,)):
                candidates.extend(mesh_grasps(pose_matrix(o['pose']),physical_meshes[str(target)],depths)[:24])
            candidates=rank_candidates(candidates,target)
            from extraction_orientations import append_halfturns
            candidates=append_halfturns(candidates)
            from wall_grasps import append_inward_tilt
            candidates=append_inward_tilt(candidates,pose_matrix(o['pose']),
                                          physical_meshes[str(target)],manifest['container']['center'])
            chosen=None;visit_forecasts=0
            event('search_visit',object_id=target,start_index=search.cursors[target],total_candidates=len(candidates))
            for index in search.indices(len(candidates)):
                if visit_forecasts>=search.forecast_budget:break
                event('candidate_considered',object_id=target,index=index,candidate=candidates[index])
                c=candidates[index];search.advance(index)
                p=np.array(c['ee_position_m']);r=c['ee_rotation_world'];candidate_state=state();q0=candidate_state['robots'][arm]['q'][:7]
                from candidate_replay import CandidateReplay
                replay=CandidateReplay(event,rng,candidate_state,arm,target,index,c)
                ik=replay.call('pregrasp_ik',lambda:planner.solve_ik(model,arm,p+[0,0,.1],r,q0,rng=rng,max_iters=100,restarts=1),position=p+[0,0,.1],rotation=r,q_seed=q0,max_iters=100,restarts=1)
                if not ik['success']:continue
                geometry_state=state();world=collision_world(geometry_state)
                replay.world_state(geometry_state)
                if not replay.call('pregrasp_geometry',lambda:replay.free(world,arm,ik['q']),q=ik['q']):continue
                grasp=replay.call('grasp_ik',lambda:planner.solve_ik(model,arm,p,r,ik['q'],rng=rng,max_iters=100,restarts=1),position=p,rotation=r,q_seed=ik['q'],max_iters=100,restarts=1)
                if not grasp['success'] or not replay.call('grasp_geometry',lambda:replay.free(world,arm,grasp['q'],target,False),q=grasp['q'],target_id=target,allow_target_fingers=False):continue
                # Endpoint reachability alone admitted a wrist-limit dead end
                # on N4. Require every lift waypoint on the grasp's IK branch,
                # with 1cm extra clearance for measured closure/PD offsets.
                lift_amount=max(.10,.95-payload_bottom(target))
                lift_points=[p+[0,0,h] for h in np.linspace(0,lift_amount,int(np.ceil(lift_amount/.04))+1)[1:]]
                lift_ik=replay.call('lift_continuation',lambda:planner.solve_ik_sequence(model,arm,lift_points,r,grasp['q'],rng=rng),positions=lift_points,rotation=r,q_seed=grasp['q'])
                if not lift_ik['success']:
                    event('candidate_rejected',object_id=target,index=index,stage='lift_continuation',result=lift_ik)
                    continue
                prepath=replay.call('approach_path',lambda:planner.plan_joint_path(q0,ik['q'],bounds,lambda q:replay.free(world,arm,q),rng=rng,max_nodes=80,resolution=.025),start=q0,goal=ik['q'],bounds=bounds,max_nodes=80,resolution=.025)
                if not prepath['success']:continue
                descent=replay.call('descent_path',lambda:planner.plan_joint_path(ik['q'],grasp['q'],bounds,lambda q:replay.free(world,arm,q,target,False),rng=rng,max_nodes=80,resolution=.025),start=ik['q'],goal=grasp['q'],bounds=bounds,target_id=target,allow_target_fingers=False,max_nodes=80,resolution=.025)
                if not descent['success']:continue
                c=dict(c,pregrasp_q=ik['q'],grasp_q=grasp['q'])
                event('candidate_preflight',object_id=target,index=index,candidate=c,pregrasp_ik=ik,grasp_ik=grasp,pregrasp_path=prepath,descent_path=descent,lift_continuation=lift_ik)
                visit_forecasts+=1
                passed=replay.call('physical_forecasts',lambda:preview_grasp(arm,target,c,index),candidate=c)
                experience_record(target,c,passed,'extraction')
                if not passed:continue
                chosen=c;break
            if chosen is None:
                exhausted=search.defer(len(candidates))
                event('no_candidate' if exhausted else 'search_deferred',object_id=target,
                      pending=list(search.pending),next_index=search.cursors[target],
                      total_candidates=len(candidates),visit_forecasts=visit_forecasts)
                continue
            relative,lift_start,lift_amount,r=execute_grasp(arm,target,chosen)
            from live_clearance import Live,scoped_contacts
            from loaded_common import controls as loaded_controls
            live_api=types.SimpleNamespace(state=state,ee_pose=lambda a:env.get_arm_pose(a),
                controls=lambda:loaded_controls(env),contacts=lambda:scoped_contacts(env,target),world=collision_world,model=model,rng=rng,
                execute=execute,tick=tick,event=event,relative=lambda:relative,
                command=lambda a,q,v:__import__('actuator_targets').issue(env.robot,a,q,v,bounds))
            live=Live(live_api,arm,target,loaded_variant,loaded_science)
            slots=env.grid_slots_right if arm=='right' else env.grid_slots_left
            options=[s for s in slots if s not in env.grid_slots]
            base_xy=np.array([-.55 if arm=='left' else .55,-.55])
            options.sort(key=lambda s:np.linalg.norm(np.asarray(s[:2])-base_xy))
            # Preserve the original search tier; append exact-deduplicated alternatives.
            options=live.slots(options,env.grid_slots)
            transported=False
            for slot in options:
                for yaw in (0,np.pi/4,-np.pi/4,np.pi/2,-np.pi/2,np.pi):
                    cs,sn=np.cos(yaw),np.sin(yaw)
                    turn=np.array([[cs,-sn,0],[sn,cs,0],[0,0,1]])
                    place_r=turn@np.asarray(r)
                    payload_offset=place_r@relative[:3,3]
                    above=np.array([slot[0]-payload_offset[0],slot[1]-payload_offset[1],lift_start[2]+lift_amount])
                    # A high transit pose alone does not establish that a bulky
                    # object fits below tray rims or between earlier placements.
                    # Check the entire lowering path before transporting there.
                    current=state();q0=current['robots'][arm]['q'][:7]
                    up_ik=planner.solve_ik(model,arm,above,place_r,q0,rng=rng,max_iters=150,restarts=2)
                    if not up_ik['success']:continue
                    predicted=np.eye(4);predicted[:3,:3]=place_r;predicted[:3,3]=above;predicted=predicted@relative
                    bottom=min((np.asarray(m['vertices'])@predicted[:3,:3].T+predicted[:3,3])[:,2].min() for m in physical_meshes[str(target)])
                    # Reserve both arms' home workspace before releasing an
                    # obstacle there. N19 previously made the return GOAL itself
                    # collide, which no amount of RRT search can repair.
                    settled_pose=predicted.copy();settled_pose[2,3]+=.74-bottom
                    future=dict(current,robots={a:dict(q=list(homes[a])+[.04,.04]) for a in homes})
                    after=collision_world(future);after.set_object_transform(target,settled_pose)
                    home_clear=True
                    for a in homes:
                        if not after.free(a,homes[a]):
                            event('placement_rejected',object_id=target,slot=slot,reason='blocks_home',collision=after.last_collision)
                            home_clear=False;break
                    if not home_clear:continue
                    if reservation is not None and not reservation.check(current,target,'predicted_slot',settled_pose):
                        event('placement_rejected',object_id=target,slot=slot,reason='blocks_contact_HOME_workspace')
                        continue
                    low=above-[0,0,max(0.,bottom-.77)]
                    low_ik=planner.solve_ik(model,arm,low,place_r,up_ik['q'],rng=rng,max_iters=150,restarts=2)
                    if not low_ik['success']:continue
                    probe=collision_world(current);probe.held=(arm,str(target),relative)
                    # Reserve existing endpoint tracking error before transport.
                    # Nominal clearance alone admitted the s103 link7/wall failure.
                    from placement_endpoint import assess_link7_endpoint
                    endpoint=assess_link7_endpoint(probe,arm,low_ik['q'],low,place_r)
                    from placement_contract import endpoint_receipt
                    endpoint_record=endpoint_receipt(probe,arm,low_ik['q'],low,place_r,target,slot,endpoint,endpoint_binding,above)
                    event('placement_endpoint_admission',object_id=target,arm=arm,receipt=endpoint_record)
                    if not endpoint['safe']:
                        event('placement_rejected',object_id=target,slot=slot,
                              reason='placement_link7_tracking_clearance',endpoint=endpoint)
                        continue
                    finger=np.clip(current['robots'][arm]['q'][-2:],0.,.04).tolist()
                    down_path=planner.plan_joint_path(up_ik['q'],low_ik['q'],bounds,
                        lambda q:probe.free(arm,q,target,True,finger),rng=rng,max_nodes=80,resolution=.025)
                    if not down_path['success']:
                        event('placement_rejected',object_id=target,slot=slot,reason=down_path['reason'],collision=probe.last_collision)
                        continue
                    retreat=planner.solve_ik(model,arm,low+[0,0,.12],place_r,low_ik['q'],rng=rng,max_iters=120,restarts=0)
                    if not retreat['success']:continue
                    away=planner.plan_joint_path(low_ik['q'],retreat['q'],bounds,
                        lambda q:after.free(arm,q,target,True),rng=rng,max_nodes=80,resolution=.025)
                    if not away['success']:
                        event('placement_rejected',object_id=target,slot=slot,reason='release_retreat:'+away['reason'],collision=after.last_collision)
                        continue
                    from placement_retreat import assess as assess_retreat
                    retreat_record=assess_retreat(collision_world,current,live_api.controls(),arm,target,
                        low_ik['q'],retreat['q'],settled_pose,away['path'],rng)
                    event('placement_retreat_admission',object_id=target,arm=arm,slot=slot,
                        endpoint_admission_sha256=endpoint_record['sha256'],receipt=retreat_record)
                    if not retreat_record['safe']:
                        event('placement_rejected',object_id=target,slot=slot,
                            reason='unloaded_retreat_contract',retreat_admission_sha256=retreat_record['sha256'])
                        continue
                    if reservation is not None:
                        event('placement_home_entry_context',object_id=target,arm=arm,slot=slot,
                            endpoint_admission_sha256=endpoint_record['sha256'],binding=endpoint_binding,
                            state_sha256=endpoint_record['state_sha256'],retreat_q=retreat['q'],
                            predicted_transform=settled_pose.tolist(),
                            home_contact_model_sha256=reservation.runtime.envelope.sha256)
                    if reservation is not None:
                        from home_entry_preflight import assess as assess_HOME_entry
                        entry=assess_HOME_entry(collision_world,current,reservation.runtime.envelope,
                            arm,target,retreat['q'],settled_pose)
                        event('placement_home_entry_admission',object_id=target,arm=arm,slot=slot,receipt=entry)
                        if not entry['safe']:
                            event('placement_rejected',object_id=target,slot=slot,
                                  reason='retreat_endpoint_blocks_HOME_entry',entry=entry)
                            continue
                    back=planner.plan_joint_path(retreat['q'],homes[arm],bounds,
                        lambda q:after.free(arm,q),rng=rng,max_nodes=80,resolution=.025)
                    if not back['success']:
                        event('placement_rejected',object_id=target,slot=slot,reason='return:'+back['reason'],collision=after.last_collision)
                        continue
                    if cfg.home_coarse and cfg.joint_step_refinement:
                        from home_admission import admit
                        try:
                            resource_admission=admit(back['path'],cfg.home_nominal_speed_rad_s)
                        except CandidateInfeasible as exc:
                            event('placement_rejected',object_id=target,slot=slot,
                                  reason='HOME_computational_admission:'+str(exc))
                            continue
                        event('home_command_admission',object_id=target,arm=arm,
                              stage='predicted_placement_route',**resource_admission)
                    event('placement_preflight',object_id=target,arm=arm,retreat_admission_sha256=retreat_record['sha256'],home_entry_admission_sha256=None if reservation is None else entry['sha256'],endpoint_admission_sha256=endpoint_record['sha256'],slot=slot,rotation=place_r.tolist(),
                          above=above.tolist(),low=low.tolist(),up_q=up_ik['q'],low_q=low_ik['q'],retreat_q=retreat['q'])
                    if not live.prepare(above,place_r,up_ik['q'],down_path['path'],slot,yaw):continue
                    if live.move('transport',above,place_r,up_ik['q'],plan_to):
                        r=place_r;transported=True;break
                if transported:break
            if not transported:raise RuntimeError('no_reachable_clear_placement')
            event('transported',object_id=target,arm=arm,retreat_admission_sha256=retreat_record['sha256'],home_entry_admission_sha256=None if reservation is None else entry['sha256'],endpoint_admission_sha256=endpoint_record['sha256'],slot=slot,yaw_adjustment_rad=float(yaw))
            # Execute the mesh-derived lowering target and IK branch already
            # checked above, rather than changing height using a loose OBB.
            final=low.copy()
            if placement_trace is not None:
                placement_trace.begin(target,arm,relative,dict(endpoint_admission_sha256=endpoint_record['sha256'],
                    slot=slot,low=low.tolist(),rotation=place_r.tolist(),low_q=low_ik['q'],retreat_q=retreat['q'],
                    homes=homes,predicted_transform=settled_pose.tolist(),
                    reservation_model_sha256=None if reservation is None else reservation.runtime.envelope.sha256,
                    HOME_command_admission=resource_admission if cfg.home_coarse and cfg.joint_step_refinement else None))
            live.move('lowering',final,r,low_ik['q'],plan_to)
            def release_observation(stage,attempted_step=None,failure=None):
                observed=state();controls=live_api.controls()
                event('extraction_release_transition',object_id=target,arm=arm,stage=stage,
                    state=observed,controls=controls,state_sha256=digest(observed),controls_sha256=digest(controls),
                    native_ee=pose_matrix(env.get_arm_pose(arm)).tolist(),held_id=held_id,drive_active=drive is not None,attempted_step=attempted_step,failure=failure)
            release_observation('loaded_exit')
            if placement_trace is not None:placement_trace.mark('detach_before')
            env._release_magnetic(drive,actor);drive=None;held_id=None
            if placement_trace is not None:placement_trace.mark('detach_after')
            env.robot.set_gripper(1.,arm,gripper_eps=0)
            if placement_trace is not None:placement_trace.mark('opening_after')
            if placement_trace is not None:placement_trace.mark('postrelease_wait')
            release_observation('released_open')
            for _ in range(250):
                before=step
                try:tick(target,False)
                except Exception as exc:
                    release_observation('wait_tick',before,type(exc).__name__+':'+str(exc));raise
                else:release_observation('wait_tick',before)
            release_observation('retreat_start')
            if placement_trace is not None:placement_trace.mark('first_retreat')
            plan_to(arm,final+[0,0,.12],r,target,joint_goal=retreat['q'],certified_retreat=True)
            release_observation('retreat_end')
            if placement_trace is not None:placement_trace.mark('retreat_complete')
            if reservation is not None:
                # Recheck the ACTUAL released placement before HOME. Predicted
                # rigid geometry is not a settling or executable-prefix proof.
                if not reservation.check(state(),target,'measured_after_retreat'):
                    raise CandidateInfeasible('measured_placement_blocks_HOME_workspace')
                home_return(arm,target)
                if not reservation.check(state(),target,'measured_after_home'):
                    raise CandidateInfeasible('measured_HOME_workspace_lost')
            else:
                world=collision_world(state());q0=state()['robots'][arm]['q'][:7]
                back=planner.plan_joint_path(q0,homes[arm],bounds,lambda q:world.free(arm,q),rng=rng,max_nodes=80)
                event('return_plan',object_id=target,path=back,collision=world.last_collision)
                if not back['success']:raise RuntimeError('custom_return_failed')
                execute(arm,back['path'],target)
            body=actor.actor.find_component_by_type(sapien.pysapien.physx.PhysxRigidDynamicComponent)
            pos=actor.get_pose().p
            settled=(np.linalg.norm(pos[:2]-np.array(slot[:2]))<.07 and pos[2]>.72
                     and np.linalg.norm(body.get_linear_velocity())<.03)
            event('released',object_id=target,settled=bool(settled),position=pos.tolist())
            if not settled:raise RuntimeError('released_object_not_settled')
            env.grid_slots.append(slot);completed.append(target)
            search.complete()
            placement_targets[str(target)]=list(slot[:3])
            ledger.extract(target)
            event('extracted',object_id=target,state=state())
        if set(completed)!=set(selected):raise RuntimeError('incomplete_extraction')
        if reservation is not None and not reservation.check(state(),None,'all_extracted'):
            raise CandidateInfeasible('barrier_HOME_workspace_lost')
        # Freeze evidence of the all-extracted barrier before any physical regrasp.
        event('extraction_barrier',selected_ids=selected,state=state())
        if cfg.mode!='extraction':
            ledger.begin_return();returning=True;switch_phase('return')
            api=types.SimpleNamespace(state=state,ee_pose=lambda arm:env.get_arm_pose(arm),meshes=physical_meshes,
                model=model,rng=rng,world=collision_world,plan_to=plan_to,insert=insert_to,
                release=release_return,retreat=retreat_return,home=home_return,settle=settle_return,event=event,
                rank=rank_candidates,grasp=execute_grasp,preview_return=preview_return,ledger=ledger,
                experience_record=experience_record,return_budget=return_budget)
            pipeline=ReturnPipeline(api,manifest,cfg)
            returned=pipeline.run(selected)
        switch_phase('final_settle')
        # Check the final state of every completed object after the last return,
        # rather than retaining only each object's earlier release result.
        if completed:
            if cfg.defer_intermediate_home and returning:settle_return(completed[-1],500)
            else:
                for _ in range(500):tick(completed[-1],False)
    except Exception as exc:
        failure=type(exc).__name__+':'+str(exc)
        if placement_trace is not None:placement_trace.exception(failure)
        diagnostic=exception_diagnostic(exc)
        if isinstance(exc,CandidateInfeasible):
            # This handler ends actual execution; only isolated previews may
            # reject a candidate and continue from their unchanged source.
            diagnostic.update(retryable=False,recovery_requires_fresh_planning=True,
                              candidate_retry_scope='isolated_preview_only')
        # Persist a strings-only diagnostic before potentially broken state or
        # event serialization. The harness also captures the full stderr stack.
        traceback.print_exc()
        write_once(out/'failure-diagnostic.json',dict(phase=phase,physics_step=step,**diagnostic))
        event('stopped',reason=failure,**diagnostic,state=state(),contacts=contacts(),held_id=held_id)
    finally:
        if live_sleep is not None:live_sleep.finish(failure)
        loaded_science.finish(failure)
        if copy_observer is not None:copy_observer.finish(failure)
        if placement_trace is not None:placement_trace.finish(phase,step,failure)
        phase_times[phase]=phase_times.get(phase,0.)+time.monotonic()-phase_started
        phase_physical_steps[phase]=step-phase_step_start;phase_preview_steps[phase]=preview_total_steps-phase_preview_start
        record_trace(force=True)
        terminal=state()
        final_meshes=scene_meshes(env)
        final_identity=identity_records(env,task,final_meshes)
        for oid,o in terminal['objects'].items():o.update(final_identity[oid])
        write_once(out/'final-state.json',dict(state=terminal,homes=homes,placement_targets=placement_targets,
            held_id=held_id,drive_active=drive is not None,active_magnetic_ids=[held_id] if drive is not None else [],
            completed_ids=completed,returned_ids=sorted(map(int,ledger.returned)),contacts=contacts(),physics_step=step,
            scene_manifest_sha256=manifest_hash,trajectory_sha256=trace_hash.hexdigest(),trajectory_samples=trace_samples,
            formal_mutations=[],identity_preserved={n:id(o.actor) for n,o in env.objects.items()}==formal_objects))
        write_once(out/'experience-records.json',dict(schema='roundtrip-experience/1',records=experience_records,
            split=cfg.split,policy_frozen=True,snapshot_sha256=experience.snapshot_sha256,matches=experience.matches,rank_calls=experience.rank_calls,
            used_hints=experience.used_hints,reordered_calls=experience.reordered_calls))
        writer.close();log.close();trace.close()
        result=dict(completed_ids=completed,returned_ids=sorted(map(int,ledger.returned)),total_objects=cfg.count,requested_objects=args.max_objects,
            mode=cfg.mode,partial=cfg.mode=='smoke',phase_elapsed_s=phase_times,scene_manifest_sha256=manifest_hash,
            phase_physical_steps=phase_physical_steps,phase_preview_steps=phase_preview_steps,
            physics_step_scope='formal trajectory and isolated forecasts after frozen initial scene; initialization steps are not instrumented',
            initialization_physics_steps=None if visual is None else visual.initialization_steps(),
            render_assignment=render_assignment,
            accepted=failure is None and len(completed)==args.max_objects and (cfg.mode=='extraction' or ledger.returned==ledger.selected),
            full_ten_success=False,whole_episode_success=False,verification_required=True,
            failure=failure,magnetic_drive_limit_raw=80,forbidden_legacy_calls=calls,
            custom_planning_calls=plans,physics_steps=step,physics_preview_attempts=preview_attempts,grasp_preview_attempts=grasp_preview_attempts,physics_preview_steps=preview_total_steps,forecast_isolation='separate_PhysxCpuSystem',runtime_variant='v36-live-combined-021',loaded_variant=loaded_variant,forecast_contract='separation-sleep-hypotheses/1',gpu_at_start=gpu,
            retained=['SAPIEN physics','PD actuators','scene construction','magnetic drive'],
            release_velocity_reset=True,finger_friction=[3.0,2.4],
            geometry_source='live_PhysX_cooked_convex_shapes',physics_mesh_sha256=mesh_hash,
            grasp_forecast_repetitions=2,approach_target_finger_contact_allowed=False,final_approach_velocity_rad_s=.1,
            reuse_preflight_grasp_ik=True,
            grasp_search_depth_tiers_m=[[.014,.025,.04,.06],[.008],[.004],[.001],[.0005]],
            search_schedule=dict(candidate_budget_per_visit=24,forecast_budget_per_visit=6,
                                 reset_after_committed_success=True),
            gripper_control=dict(travel_m=[0.,.04],contact_preload_m=.00025,drive_force_limit_raw=4.,max_settled_speed_m_s=.01,closure_target_speed_m_s=.01,
                                 touch_separation_range_m=[-.0003,.0001],touch_min_point_impulse=1e-6),
            source_sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in ('astra_planner.py','direct_core.py','collision_world.py','physics_runner.py','surface_grasps.py','convex_geometry.py','mesh_world.py','separation.py','physics_geometry.py','candidate_search.py','gripper_control.py')},
            preview=dict(fps=10,capture_every_steps=25,ray_samples=4) if visual is None else visual.capture_contract(),
            claim='diagnostic prototype; not the frozen historical verifier')
        if visual is not None:result['headless_candidate']=visual.final_accounting(step,preview_total_steps)
        episode_completed=result['accepted']
        if continuous:
            result.update(schema='continuous-roundtrip-result/22',diagnostic_completed=episode_completed,
                accepted=False,dataset_success=False,formal_credit=False,initialized_episode=True,reconstructed=False)
        if continuous_version in (35,36):
            result.update(preview_accounting_schema='preview-lifecycle/35',preview_lifecycle_attempts=preview_accounting.sequence)
        write_once(out/'result.json',result)
        (out/'terminal_status').write_text('0\n' if episode_completed else '2\n')
        print(timing.call('json',json.dumps,result,indent=2))
    return 0 if episode_completed else 2


if __name__=='__main__':sys.exit(main())
