"""Live-only forecast qualification. Read-only source capture; no scene stepping."""
import runtime_profile as timing
import base64,contextlib,gzip,hashlib,json,traceback
from pathlib import Path
from provenance import canonical,digest,write_once
from sleep_copy_observation_v35 import snapshot
from forecast_report_io_v36 import pack,SerializationFailure
from action_tape import Writer,TapeFailure
from summary_stream import Summaries
from action_tape import PROFILES
from live_sleep_readback import force_limit,SystemReferences,validate_identity

CONTRACT='live-sleep-qualification/2'
from live_sleep_outcome import classify

@timing.measured('controls')
def controls(env):
    return {a:[dict(name=j.get_name(),stiffness=j.get_stiffness(),damping=j.get_damping(),
        force_limit=force_limit(j.get_force_limit()),mode=str(j.get_drive_mode()),target=j.get_drive_target().tolist(),
        velocity_target=j.get_drive_velocity_target().tolist()) for j in getattr(env.robot,a+'_entity').get_active_joints()]
        for a in ('left','right')}

@timing.measured('copy')
def capture(env,cls,state):
    raw=env.scene.physx_system.pack()
    contacts=sorted([dict(bodies=[b.entity.get_name() for b in c.bodies],
        points=[dict(separation=float(p.separation),impulse=p.impulse.tolist(),position=p.position.tolist(),normal=p.normal.tolist()) for p in c.points])
        for c in env.scene.get_contacts()],key=lambda r:json.dumps(r,sort_keys=True))
    return dict(objects=snapshot(env.objects,cls),state=state(env),controls=controls(env),contacts=contacts,
        pack_base64=base64.b64encode(raw).decode(),pack_sha256=hashlib.sha256(raw).hexdigest())

def ambiguous_ids(raw):
    return [k for k,v in raw.items() if v['sleeping'] and any(v['linear']+v['angular'])]

def strict_outcome(before,after,failure):
    expected=json.loads(json.dumps(before))
    ids=ambiguous_ids(before)
    for k in ids:expected[k]['linear']=[0.,0.,0.];expected[k]['angular']=[0.,0.,0.]
    return bool(ids and failure=='RuntimeError:forecast_sleep_copy_readback_mismatch' and after==expected)

class LiveWitness:
    def __init__(self,out,formal,grasp,cls,state,context,binding,emit):
        self.out=Path(out);self.formal=formal;self.grasp=grasp;self.cls=cls;self.state=state
        self.systems=SystemReferences()
        self.context=context;self.binding=binding;self.emit=emit;self.active=None;self.rows=Summaries(self.out)
        self.prefix=None;self.prefix_owner=None;self.global_hash=hashlib.sha256();self.global_count=0;self.tape_serial=0
    def new_prefix(self,source,owner):
        if self.prefix is not None:self.prefix['writer'].close()
        self.tape_serial+=1
        self.prefix_owner=owner
        initial=capture(source,self.cls,self.state);context=self.context()
        self.prefix=dict(writer=Writer(self.out,self.tape_serial),initial=initial,context=context)
    def begin_prefix(self,source,destination):
        if destination is self.grasp:self.new_prefix(source,destination)
    def grasp_begin(self,env):
        if env is self.formal:self.new_prefix(env,env)
    def permitted(self,source,target):
        return source is self.grasp or (source is self.formal and target in self.rows.qualified)
    @timing.measured('witness')
    def tick(self,env,formal_step,preview_step,preview):
        commands=dict(kind='physics_command',controls=controls(env),qf={a:getattr(env.robot,a+'_entity').get_qf().tolist() for a in ('left','right')},
            formal_step=formal_step,preview_step=preview_step,preview=preview)
        raw=canonical(commands);self.global_hash.update(raw);self.global_count+=1
        if env is self.prefix_owner and self.prefix is not None:
            self.prefix['writer'].append(raw)
    def event(self,kind,values,env=None):
        if kind=='magnet_attached' and self.prefix is not None and env is self.prefix_owner and self.active is None:
            raw=canonical(dict(kind=kind,**values));self.global_hash.update(raw);self.global_count+=1
            self.prefix['writer'].append(raw)
        if self.active is not None and kind in ('forecast_initialization','separation_preview','separation_profile_bundle'):
            self.active['events'].append(dict(kind=kind,**values))
    @contextlib.contextmanager
    def qualify(self,source,destination,candidate,arm,target,baseline):
        if self.active is not None or not self.permitted(source,target):raise RuntimeError('live_sleep_scope')
        if len(self.rows)>=PROFILES['summaries'][2]:raise TapeFailure('live_sleep_episode_boundary_budget')
        index=len(self.rows)+1;folder=self.out/'live-sleep'/('%04d'%index);folder.mkdir(parents=True)
        native_identity=self.systems.pair(source,destination)
        row=dict(schema=CONTRACT,binding=self.binding,context=self.context(),arm=arm,object_id=target,
            source_role='formal' if source is self.formal else 'grasp_preview',
            path=candidate['path'],path_sha256=digest(candidate['path']),source_before=capture(source,self.cls,self.state),
            formal_before=capture(self.formal,self.cls,self.state),events=[],status='FAILED',
            destination_reuse=True,destination_reset='all original copy setters before each hypothesis',
            contact_history_equivalent=False,conservative_bound_proved=False,
            global_action_prefix=dict(count=self.global_count,sha256=self.global_hash.hexdigest()),
            native_system_identity=native_identity,source_system=native_identity['source_system'],destination_system=native_identity['destination_system'])
        self.active=row;error=None
        try:
            validate_identity(native_identity,row['source_system'],row['destination_system'])
            p=self.prefix
            if p is None:raise TapeFailure('live_grasp_action_prefix_unavailable')
            row['grasp_prefix']=dict(initial=p['initial'],context=p['context'],**p['writer'].checkpoint())
            failure=None;stack=None
            try:baseline()
            except Exception as exc:failure=type(exc).__name__+':'+str(exc);stack=traceback.format_exc()
            row['baseline']=dict(failure=failure,traceback=stack,destination_after=capture(destination,self.cls,self.state))
            row['source_after_baseline']=capture(source,self.cls,self.state)
            if row['source_before']!=row['source_after_baseline']:raise RuntimeError('live_baseline_modified_source')
            if row['formal_before']!=capture(self.formal,self.cls,self.state):raise RuntimeError('live_baseline_modified_formal')
            if not strict_outcome(row['source_before']['objects'],row['baseline']['destination_after']['objects'],failure):
                raise RuntimeError('live_baseline_mismatch_not_reproduced')
            yield row
            outcome=classify(row['events'],row['path_sha256'],target)
            row['status']='QUALIFIED_FORECAST_PAIR' if outcome['qualified'] else 'REJECTED_FORECAST_PAIR'
            row['rejection_reason']=None if outcome['qualified'] else 'all_separation_profiles_physically_rejected'
        except BaseException as exc:
            error=exc;row['failure_category']='infrastructure_serialization' if isinstance(exc,(TapeFailure,SerializationFailure,OSError)) else 'qualification_or_source_failure';row['failure']=type(exc).__name__+':'+str(exc);row['traceback']=traceback.format_exc();raise
        finally:
            try:
                row['source_after']=capture(source,self.cls,self.state);row['formal_after']=capture(self.formal,self.cls,self.state)
                row['source_unchanged']=row['source_before']==row['source_after']
                row['formal_unchanged']=row['formal_before']==row['formal_after']
                if not row['source_unchanged'] or not row['formal_unchanged']:
                    row['status']='FAILED';row['source_guard_failure']='live_qualification_mutated_source'
                compact=dict(schema=CONTRACT,index=index,status=row['status'],context=row['context'],
                    source_role=row['source_role'],object_id=target,arm=arm,
                    path_sha256=row['path_sha256'],failure=row.get('failure'),failure_category=row.get('failure_category'),rejection_reason=row.get('rejection_reason'),baseline_failure=row.get('baseline',{}).get('failure'),
                    source_unchanged=row['source_unchanged'],formal_unchanged=row['formal_unchanged'],formal_credit=False,
                    contact_history_equivalent=False,conservative_bound_proved=False,full_report=None)
                try:
                    data,bound=pack(row);bound['path']='report.json.gz'
                    self.rows.writer._space(len(data)+65536)
                    with (folder/'report.json.gz').open('xb') as f:f.write(data)
                    compact['full_report']=bound
                except BaseException as exc:
                    compact.update(status='SERIALIZATION_FAILED',failure_category='infrastructure_serialization',serialization_error=type(exc).__name__+':'+str(exc))
                    write_once(folder/'summary.json',compact);self.rows.append(compact)
                    if not row['source_unchanged'] or not row['formal_unchanged']:raise RuntimeError('live_qualification_mutated_source') from exc
                    raise
                write_once(folder/'summary.json',compact);self.rows.append(compact)
                self.emit('live_sleep_boundary',summary_path=str((folder/'summary.json').relative_to(self.out)),summary_sha256=digest(compact),status=compact['status'])
                # Never let an ordinary rejection hide source corruption.
                if not row['source_unchanged'] or not row['formal_unchanged']:
                    raise RuntimeError('live_qualification_mutated_source')
                if compact['status'] not in ('QUALIFIED_FORECAST_PAIR','REJECTED_FORECAST_PAIR') and error is None:raise RuntimeError('live_sleep_source_or_qualification_failed')
            finally:self.active=None
    def finish(self,failure):
        if self.prefix is not None:self.prefix['writer'].close()
        tape_hash=hashlib.sha256();tape_records=0
        for i in range(1,self.tape_serial+1):
            raw=(self.out/'live-action-tapes'/('%08d'%i)/'terminal.json').read_bytes()
            tape_hash.update(raw);tape_records+=json.loads(raw)['count']
        write_once(self.out/'live-sleep-receipt.json',dict(schema=CONTRACT,binding=self.binding,summary_stream=self.rows.close(),
            action_tapes=dict(count=self.tape_serial,records=tape_records,terminal_sha256=tape_hash.hexdigest()),
            episode_failure=failure,global_action_prefix=dict(count=self.global_count,sha256=self.global_hash.hexdigest()),
            qualification_only=True,formal_credit=False,goal_complete=False))
