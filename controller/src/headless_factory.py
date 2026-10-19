"""Opt-in, process-local visual suppression. No native imports at module import.

Use in a detached process only. All physics comes from bound original sources.
Default frozen runner never imports this module and is byte unchanged.
"""
import ast,copy,hashlib,importlib.abc,importlib.util,pathlib,sys,types

def dump(n):return ast.dump(n,include_attributes=False)
def module(nodes):return ast.fix_missing_locations(ast.Module(body=nodes,type_ignores=[]))
def named(tree,name):return next(n for n in ast.walk(tree) if isinstance(n,(ast.ClassDef,ast.FunctionDef)) and n.name==name)
def strip_visual_source(source,kind):
    """Closed, inspectable AST removals; unchanged statements retain exact AST."""
    t=ast.parse(source);removed=[]
    def drop(body,predicate):
        keep=[]
        for n in body:
            if predicate(n):removed.append(ast.unparse(n))
            else:keep.append(n)
        return keep
    if kind=='base':
        f=named(t,'setup_scene')
        # Only these standalone visual declarations; light/RNG loops remain.
        starts=('from sapien.render import set_global_config','set_global_config(',
            'self.renderer = sapien.SapienRenderer(', 'self.engine.set_renderer(',
            'sapien.render.set_camera_shader_dir(', 'sapien.render.set_ray_tracing_')
        f.body=drop(f.body,lambda n:ast.unparse(n).startswith(starts))
        if len(removed)!=8:raise ValueError('base_visual_source_shape')
    elif kind=='box':
        f=named(t,'create_entity_box')
        f.body=drop(f.body,lambda n:ast.unparse(n).startswith((
            'if texture_id is not None:', 'render_component = ',
            'render_component.attach(', 'entity.add_component(render_component)')))
        if len(removed)!=4:raise ValueError('box_visual_source_shape')
        # Record description, not a native renderer or collision replacement.
        f.body.insert(-1,ast.parse('_headless_box(locals())').body[0])
    elif kind=='urdf':
        f=named(t,'_build_link')
        f.body=drop(f.body,lambda n:isinstance(n,ast.For) and ast.unparse(n.iter)=='link.visuals')
        if len(removed)!=1:raise ValueError('urdf_visual_source_shape')
    else:raise ValueError(kind)
    return module(t.body),removed

def factory_nodes(source):
    main=named(ast.parse(source),'main');body=main.body
    cls=copy.deepcopy(next(n for n in body if isinstance(n,ast.ClassDef) and n.name=='DirectScene'))
    f=named(cls,'setup_scene');assert len(f.body)==2 and ast.unparse(f.body[1])=='sapien.render.set_ray_tracing_samples_per_pixel(4)'
    f.body=f.body[:1]
    start=next(i for i,n in enumerate(body) if ast.unparse(n).startswith("config = yaml.safe_load("))
    end=next(i for i,n in enumerate(body) if ast.unparse(n)=='env = DirectScene()')
    return module([cls]+copy.deepcopy(body[start:end+1]))

def camera_nodes(source):
    cls=named(ast.parse(source),'Camera');cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('__init__','load_camera','update_wrist_camera')]
    assert len(cls.body)==3
    # Keep original constructor and load_camera including random draws.
    return module(ast.parse('import os\nimport numpy as np\nimport yaml\nimport sapien.core as sapien\nfrom envs._GLOBAL_CONFIGS import CONFIGS_PATH').body+[cls])

def plain(v):
    if v is None or isinstance(v,(str,bool,int,float)):return v
    if isinstance(v,dict):return {str(k):plain(w) for k,w in v.items()}
    if isinstance(v,(list,tuple)):return [plain(w) for w in v]
    if hasattr(v,'tolist'):return v.tolist()
    if hasattr(v,'p') and hasattr(v,'q'):return dict(p=plain(v.p),q=plain(v.q))
    if isinstance(v,Description):return plain(v.__dict__)
    raise TypeError('unknown_visual_descriptor:'+type(v).__name__)

class Description:
    def __init__(self,kind,args=(),kwargs=None):self.kind=kind;self.args=plain(args);self.kwargs=plain(kwargs or {})
    def set_base_color_texture(self,x):self.base_color_texture=plain(x)
    def set_pose(self,p):self.pose=plain(p)
    def set_color(self,c):self.color=plain(c)
    def take_picture(self):raise RuntimeError('headless_picture_unavailable')

class TransformLoader(importlib.abc.Loader):
    def __init__(self,path,kind,records):self.path=path;self.kind=kind;self.records=records
    def create_module(self,spec):return None
    def exec_module(self,m):
        m.__file__=str(self.path)
        source=self.path.read_text()
        if self.kind=='camera':tree=camera_nodes(source)
        else:tree,_=strip_visual_source(source,self.kind)
        m.__dict__['_headless_box']=lambda values:self.records.append(dict(kind='box',definition=plain({k:values[k] for k in ('pose','half_size','color','name','texture_id')})))
        exec(compile(tree,str(self.path),'exec'),m.__dict__)

class TransformFinder(importlib.abc.MetaPathFinder):
    def __init__(self,base,records):self.base=base;self.records=records
    def find_spec(self,fullname,path=None,target=None):
        spec={'envs._base_task':('envs/_base_task.py','base'),
            'envs.utils.create_actor':('envs/utils/create_actor.py','box'),
            'envs.camera.camera':('envs/camera/camera.py','camera')}.get(fullname)
        if spec:return importlib.util.spec_from_loader(fullname,TransformLoader(self.base/spec[0],spec[1],self.records))

def install(sapien,base,max_steps,scene_roles=("formal",)):
    """Parent native only. Never instantiate any rendering or GPU PhysX system."""
    import sapien.wrapper.scene as scene_module
    import sapien.wrapper.actor_builder as actors
    import sapien.wrapper.urdf_loader as urdfs
    stats=dict(physics_steps=0,scenes=0,scene_roles=list(scene_roles),steps_by_role={},render_updates_skipped=0,forbidden_calls=[])
    class RoleRecords(list):
        def append(self,item):
            role=scene_roles[stats['scenes']-1] if stats['scenes'] else 'pre_scene'
            if isinstance(item,dict):item.setdefault('scene_role',role)
            elif isinstance(item,Description):item.scene_role=role
            super().append(item)
    records=RoleRecords()
    if not scene_roles or len(scene_roles)!=len(set(scene_roles)):raise ValueError('declared_scene_roles')
    def forbidden(*a,**kw):
        stats['forbidden_calls'].append('native_render_or_gpu_call');raise RuntimeError('headless_native_render_or_gpu_forbidden')
    # Disallow every native render constructor, including aliases already imported.
    native_render=sapien.pysapien.render
    constructors={getattr(native_render,n) for n in dir(native_render) if n.startswith('Render') or n=='SapienRenderer'}
    for m in tuple(m for n,m in sys.modules.items() if n=='sapien' or n.startswith('sapien.')):
        if m is None:continue
        for n in tuple(vars(m)):
            try:hit=getattr(m,n) in constructors
            except TypeError:hit=False
            if hit:setattr(m,n,forbidden)
    sapien.SapienRenderer=forbidden
    if hasattr(sapien.physx,'PhysxGpuSystem'):sapien.physx.PhysxGpuSystem=forbidden
    def describe(kind):
        def f(*a,**kw):
            d=Description(kind,a,kw);records.append(d);return d
        return f
    # Data-only materials/textures for legacy create_table; no decoding/native call.
    sapien.render.RenderMaterial=describe('material');sapien.render.RenderTexture2D=describe('texture')
    for name in tuple(vars(actors.ActorBuilder)):
        if name.startswith('add_') and ('visual' in name):
            def omit(self,*a,_name=name,**kw):records.append(dict(kind=_name,args=plain(a),kwargs=plain(kw)));return self
            setattr(actors.ActorBuilder,name,omit)
    original_link=urdfs.URDFLoader._build_link
    tree,_=strip_visual_source(pathlib.Path(urdfs.__file__).read_text(),'urdf')
    fn=named(tree,'_build_link');ns=dict(original_link.__globals__)
    exec(compile(module([fn]),urdfs.__file__,'exec'),ns);urdfs.URDFLoader._build_link=ns['_build_link']
    original_scene_init=sapien.Scene.__init__
    def guarded_init(self,systems=None):
        if systems is None or len(systems)!=1 or not isinstance(systems[0],sapien.physx.PhysxCpuSystem):forbidden()
        original_scene_init(self,systems)
    sapien.Scene.__init__=guarded_init
    class HeadlessScene(sapien.Scene):
        def __init__(self):
            index=stats['scenes']
            if index>=len(scene_roles):raise RuntimeError('declared_scene_limit')
            self.headless_role=scene_roles[index];stats['scenes']+=1
            stats['steps_by_role'][self.headless_role]=0
            super().__init__([sapien.physx.PhysxCpuSystem()])
        def step(self):
            if stats['physics_steps']>=max_steps:raise RuntimeError('headless_physics_step_budget')
            super().step();stats['physics_steps']+=1
            stats['steps_by_role'][self.headless_role]+=1
        def update_render(self):stats['render_updates_skipped']+=1
        def set_ambient_light(self,color):
            self._headless_ambient=plain(color);records.append(dict(kind='ambient_light',color=plain(color),scene_role=self.headless_role))
        @property
        def ambient_light(self):return self._headless_ambient
        def add_directional_light(self,*a,**kw):return describe('directional_light')(*a,**kw)
        def add_point_light(self,*a,**kw):return describe('point_light')(*a,**kw)
        def add_camera(self,*a,**kw):
            d=describe('camera')(*a,**kw);d.entity=Description('camera_mount');return d
    def create_scene(engine,config=None):
        if config is None:config=sapien.SceneConfig()
        sapien.physx.set_scene_config(config)
        return HeadlessScene()
    sapien.Engine.create_scene=create_scene
    sys.meta_path.insert(0,TransformFinder(base,records))
    return records,stats

def build(frozen,release,cfg,max_steps):
    """Fresh original10 initialization extracted from exact frozen v34, no main()."""
    import os,json,numpy as np,yaml,sapien
    base=release/'workspace/base';candidate=release/'workspace/candidate'
    records,stats=install(sapien,base,max_steps)
    sys.path[:0]=[str(frozen/'src'),str(candidate),str(base)];os.chdir(base)
    calls=[]
    def forbidden(*a,**kw):calls.append('prebuilt_planner_or_grasp_API');raise RuntimeError(calls[-1])
    stub=types.ModuleType('envs.robot.planner')
    class ForbiddenPlanner:
        def __init__(self,*a,**kw):forbidden()
    stub.MplibPlanner=stub.CuroboPlanner=ForbiddenPlanner;sys.modules[stub.__name__]=stub
    import envs
    spec=importlib.util.spec_from_file_location('envs.empty_container_inner0926',candidate/'empty_container_inner0926_v11.py')
    task=importlib.util.module_from_spec(spec);sys.modules[spec.name]=task;spec.loader.exec_module(task)
    from envs.robot.robot import Robot
    from scene_pipeline import inventory,load_generated
    tree=ast.parse((candidate/'run_real10_v11_targeted.py').read_text())
    functions=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('load_selected','build_catch_trays')]
    ns=dict(taskmod=task,np=np,sapien=sapien,json=json,os=os,SELECTED_IDS=(3,4,8,9,10,11,12,17,18,19))
    exec(compile(module(functions),'bound_scene_helpers','exec'),ns)
    context=dict(task=task,np=np,sapien=sapien,yaml=yaml,Robot=Robot,forbidden=forbidden,ns=ns,cfg=cfg,base=base,candidate=candidate,Path=pathlib.Path,json=json,inventory=inventory,load_generated=load_generated)
    exec(compile(factory_nodes((frozen/'src/physics_runner.py').read_text()),'bound_v34_factory','exec'),context)
    env=context['env'];config=context['config'];env.setup_demo(now_ep_num=0,seed=cfg.seed,**config)
    if set(env.objects)!={3,4,8,9,10,11,12,17,18,19} or stats['physics_steps']!=4251:raise RuntimeError('fresh_original_initialization_inventory_or_steps')
    return env,task,records,stats,config,calls
