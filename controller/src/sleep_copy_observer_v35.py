"""Opt-in bounded observations, never a sleep policy or a retry.

One exact helper construction on real native objects validates API/alias/raw
state first. Its destination body handles alone are wrapped with transparent
getter/API proxies. The helper's algorithms, raw values and setters are intact.
Explicit lifecycle context replaces stack-frame inference. Logging failure is
recorded as incomplete evidence, never allowed to suppress a native guard.
"""
import atexit,functools
from provenance import write_once
from live_sleep_readback import SystemReferences,validate_identity
from sleep_copy_observation_v35 import JsonTrace,BodyProxy,snapshot,differences

class CopyObserver:
    def __init__(self,out,target,binding,scenes,cls,state):
        self.out=out;self.target=target;self.binding=binding;self.cls=cls;self.state=state
        self.systems=SystemReferences()
        for scene,_ in scenes:self.systems.retain(scene)
        self.roles={id(s):r for s,r in scenes};self.stack=[];self.sequence=0
        self.errors=[];self.closed=False;self.completed=0;self.trace=None
        try:self.trace=JsonTrace(out/'sleep-copy-native.jsonl')
        except Exception as exc:self.fail(exc)
        atexit.register(self.finish,'process_exit_without_terminal')
    def fail(self,exc):
        if len(self.errors)<8:self.errors.append(type(exc).__name__+':'+str(exc))
    def emit(self,kind,**values):
        if self.errors or self.closed:return
        try:self.trace(kind,**values)
        except Exception as exc:self.fail(exc)
    def read(self,source,destination):
        try:return dict(source=snapshot(source.objects,self.cls),destination=snapshot(destination.objects,self.cls))
        except Exception as exc:
            self.fail(exc);return dict(source={},destination={})
    def attach(self,guard):
        if not self.stack:return
        frame=self.stack[-1]
        frame['initialization']=getattr(guard,'mode','exact')
        def observe(stage,oid,method,before,error=None):
            try:
                now=self.read(frame['source'],frame['destination'])
                self.emit('sleep_api_'+stage,context=frame['context'],object_id=str(oid),method=method,**now,
                    source_differences=differences(frame['before']['source'],now['source']),
                    destination_differences=[] if before is None else differences(before['destination'],now['destination']),
                    native_error=None if error is None else type(error).__name__+':'+str(error))
                return now
            except Exception as exc:
                self.fail(exc);return dict(source={},destination={})
        # No replacement of source actors, destination actors, raw captured
        # values, or native getter results; all helper alias checks ran first.
        guard.rows={key:(actor,body,twin,BodyProxy(copied,key,observe),before)
                    for key,(actor,body,twin,copied,before) in guard.rows.items()}
        old_restore=guard.restore
        def restore():
            try:self.emit('after_old_setters',context=frame['context'],**self.read(frame['source'],frame['destination']))
            except Exception as exc:self.fail(exc)
            return old_restore()
        guard.restore=restore
    def finish(self,error=None):
        if self.closed:return
        self.emit('observer_terminal',binding=self.binding,copies=self.sequence,completed_boundaries=self.completed,
                  error=error,active_boundaries=len(self.stack))
        self.closed=True;receipt=None
        try:
            if self.trace is not None:receipt=self.trace.close()
        except Exception as exc:self.fail(exc)
        try:write_once(self.out/'sleep-copy-native-receipt.json',dict(schema='sleep-copy-observer/36-chunked',
            binding=self.binding,target=self.target,trace=receipt,errors=self.errors,copies=self.sequence,
            completed_boundaries=self.completed,active_boundaries=len(self.stack),terminal_error=error,
            record_cap=450_000_001,record_raw_cap=512*1024,chunk_raw_cap=1024*1024,complete=not self.errors and not self.stack,
            formal_credit=False,historical_cause_proved=False))
        except Exception as exc:self.fail(exc)


def observe_boundary(observer,context):
    if observer is None:return lambda fn:fn  # OFF: original callable, no getters.
    def decorate(fn):
        @functools.wraps(fn)
        def observed(source,destination):
            lifecycle=context()['lifecycle']
            if not lifecycle or lifecycle[-1]['object_id']!=observer.target:return fn(source,destination)
            observer.sequence+=1
            identity=observer.systems.pair(source,destination)
            ctx=dict(native_system_identity=identity,copy_id=observer.sequence,lifecycle=lifecycle,binding=observer.binding,
                source_role=observer.roles.get(id(source),'unknown'),destination_role=observer.roles.get(id(destination),'unknown'),
                source_system=identity['source_system'],destination_system=identity['destination_system'],
                **observer.state())
            frame=dict(source=source,destination=destination,context=ctx,before=observer.read(source,destination))
            observer.stack.append(frame);error=None
            observer.emit('copy_before',context=ctx,**frame['before'])
            try:
                validate_identity(identity,ctx['source_system'],ctx['destination_system'])
                check=fn(source,destination)
                def checked():
                    error=None
                    try:return check()
                    except BaseException as exc:error=exc;raise
                    finally:
                        try:observer.emit('source_recheck',context=ctx,source=snapshot(source.objects,observer.cls),
                            source_differences=differences(frame['before']['source'],snapshot(source.objects,observer.cls)),
                            error=None if error is None else type(error).__name__+':'+str(error))
                        except Exception as exc:observer.fail(exc)
                return checked
            except BaseException as exc:error=exc;raise
            finally:
                observer.stack.pop();observer.completed+=1
                try:
                    now=observer.read(source,destination)
                    observer.emit('copy_terminal',context=ctx,status='completed' if error is None else 'failed',
                        error=None if error is None else type(error).__name__+':'+str(error),**now,
                        initialization=frame.get('initialization','exact'),
                        raw_differences=differences(frame['before']['source'],now['destination']),
                        source_differences=differences(frame['before']['source'],now['source']))
                except Exception as exc:observer.fail(exc)
        return observed
    return decorate
