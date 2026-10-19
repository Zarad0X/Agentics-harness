"""Opt-in, one actual outer HOME call. Diagnostic failure never controls physics.

No scene getters, commands, timers in the control body, or per-tick sampling.
The disabled decorator returns the original function itself.
"""
import functools


def validate(enabled, max_calls):
    if type(enabled) is not bool:raise ValueError('home_cpu_profile_bool')
    if type(max_calls) is not int or max_calls != 1:raise ValueError('home_cpu_profile_max_calls_must_be_1')


def _exception(exc):
    # Even a user exception's __str__ can throw; reporting must not mask it.
    try:return dict(type=type(exc).__name__,message=str(exc))
    except BaseException:return dict(type='unprintable_exception',message=None)


def _write(path, value):
    """Publish complete JSON atomically and without replacing any pathname."""
    import json
    import os
    data=(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()
    pending=path.with_name(path.name+'.pending')
    with pending.open('xb') as stream:
        stream.write(data);stream.flush();os.fsync(stream.fileno())
    pending.chmod(0o444)
    os.link(pending,path)  # fails if destination exists; complete file visible now
    try:pending.unlink()
    except OSError:pass  # complete published file still valid; no overwrite


def _rows(profiler):
    import pstats
    stats=pstats.Stats(profiler)
    rows=[]
    for (file,line,name), value in sorted(stats.stats.items()):
        callers=[dict(file=f,line=l,function=n,values=list(v) if isinstance(v,tuple) else [v])
                 for (f,l,n),v in sorted(value[4].items())]
        rows.append(dict(file=file,line=line,function=name,primitive_calls=value[0],calls=value[1],
                         exclusive_s=value[2],inclusive_s=value[3],callers=callers))
    return stats.total_tt,rows


def profile_first_actual_home(*, enabled, max_calls, output, context, binding):
    validate(enabled,max_calls)
    if not enabled:return lambda fn:fn
    consumed=False
    def decorate(fn):
        @functools.wraps(fn)
        def call(arm,target,motion_phase,home=None):
            nonlocal consumed
            if consumed or motion_phase!='home':return fn(arm,target,motion_phase,home)
            # The context contains only runner-owned Python counters. On a
            # context failure, consume the one attempt and run unprofiled.
            try:
                start_context=context()
                if type(start_context['preview']) is not bool:raise ValueError('profile_preview_bool')
            except BaseException as exc:
                # Do not catch exceptions from fn on the preview route!
                consumed=True
                _failure(output,'context',exc,None,None)
                return fn(arm,target,motion_phase,home)
            if start_context['preview']:return fn(arm,target,motion_phase,home)
            consumed=True
            profiler=None;previous=None;hook_read=False;active=False
            setup_error=None;physical_error=None;identity=None;wall=None;cpu=None
            stage='setup'
            try:
                import cProfile
                import sys
                import time
                identity=binding()
                previous=sys.getprofile();hook_read=True
                if previous is not None:raise RuntimeError('existing_profile_hook')
                profiler=cProfile.Profile()
                wall=time.perf_counter();cpu=time.process_time()
                profiler.enable();active=True
            except BaseException as exc:
                setup_error=exc
                try:
                    if profiler is not None:profiler.disable()
                    if hook_read:
                        import sys
                        sys.setprofile(previous)
                except BaseException:pass
            try:
                return fn(arm,target,motion_phase,home)
            except BaseException as exc:
                physical_error=exc
                raise
            finally:
                # ALL finalization (including disable, clocks, context, pstats,
                # encoding, filesystem and fallback reporting) is protected.
                try:
                    stage='disable'
                    if profiler is not None:profiler.disable()
                    if hook_read:
                        import sys
                        if sys.getprofile() is not previous:sys.setprofile(previous)
                    if setup_error is not None:raise setup_error
                    if not active:raise RuntimeError('profile_not_active')
                    stage='clocks'
                    import time
                    wall=time.perf_counter()-wall;cpu=time.process_time()-cpu
                    stage='end_context';end_context=context()
                    if end_context['preview'] is not False:raise ValueError('profile_end_preview')
                    stage='stats';total,rows=_rows(profiler)
                    stage='write'
                    _write(output/'home-cpu-001.json',dict(schema='home-cpu/32',capture_success=True,
                        actual=True,preview=False,invocation=1,max_calls=max_calls,
                        arm=arm,object_id=target,motion_phase=motion_phase,
                        start=start_context,end=end_context,binding=identity,
                        physical_exception=None if physical_error is None else _exception(physical_error),
                        wall_s=wall,process_cpu_s=cpu,profile_total_exclusive_s=total,functions=rows,
                        scope='outer postrelease_motion: metadata, initial plan/certificate, coarse recertification, execution, guards, physics and existing video/IO',
                        accounting='Sum exclusive_s only; inclusive_s and caller edge times overlap. Native calls may include waits; async worker CPU is not fully attributed.',
                        profiler_overhead_can_affect_deadlines=True,terminal=False,
                        physical_success_inferred=False,formal_credit=False))
                except BaseException as exc:
                    # Disable can itself fail (tested). Restore the previous
                    # hook without letting profiler errors escape into motion.
                    try:
                        if hook_read:
                            import sys
                            sys.setprofile(previous)
                    except BaseException:pass
                    _failure(output,'setup' if setup_error is not None else stage,exc,start_context,identity)
        return call
    return decorate


def _failure(output,stage,exc,start,identity):
    try:
        _write(output/'home-cpu-001.failure.json',dict(schema='home-cpu-failure/32',capture_success=False,
            stage=stage,error=_exception(exc),start=start,binding=identity,
            terminal=False,physical_success_inferred=False,formal_credit=False))
    except BaseException:
        try:print('HOME CPU diagnostic reporting failed; capture not qualified',flush=True)
        except BaseException:pass
