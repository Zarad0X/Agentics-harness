"""Proposed lifecycle receipts only; no trial acceptance/control changes.

Totals count exclusive steps, so nested separation is never double-counted.
The old physical-success gates must remain mandatory in any future version.
"""
import functools,contextlib

class PreviewAccounting:
    def __init__(self,steps,emit):self.steps=steps;self.emit=emit;self.sequence=0;self.stack=[]
    @contextlib.contextmanager
    def attempt(self,kind,arm,target,metadata=None):
        if kind not in ('grasp','separation','return'):raise ValueError('preview_kind')
        self.sequence+=1;seq=self.sequence;parent=self.stack[-1] if self.stack else None
        begin=self.steps();frame=dict(id=seq,child_steps=0,accepted=False);self.stack.append(frame)
        error=None;started=False
        identity=dict(accounting_schema='preview-lifecycle/35',preview_id=seq,parent_preview_id=None if parent is None else parent['id'],preview_kind=kind,arm=arm,object_id=target,metadata=metadata)
        frame['identity']=identity
        try:
            self.emit('preview_accounting_started',**identity,preview_steps_begin=begin);started=True
            yield frame
        except BaseException as exc:error=exc;raise
        finally:
            end=self.steps();self.stack.pop();inclusive=end-begin
            if parent is not None:parent['child_steps']+=inclusive
            values=dict(**identity,preview_steps_begin=begin,preview_steps_end=end,inclusive_steps=inclusive,exclusive_steps=inclusive-frame['child_steps'],status='failed' if error is not None else 'completed',accepted=error is None and frame['accepted'] is True,error=None if error is None else type(error).__name__+':'+str(error),started_record_emitted=started)
            try:self.emit('preview_accounting_terminal',**values)
            except BaseException as logging_error:
                if error is None:raise
                error.__context__=logging_error  # retain guard error; missing receipt fails audit
    def context(self):
        identities=[f['identity'] for f in self.stack]
        return dict(lifecycle=[dict(x) for x in identities])
    def wrap(self,kind):
        def decorate(fn):
            @functools.wraps(fn)
            def counted(*args,**kwargs):
                meta=dict(index=args[3],trial=args[4]) if kind=='grasp' else dict(index=args[4],trial=args[5])
                with self.attempt(kind,args[0],args[1],meta) as frame:
                    answer=fn(*args,**kwargs);frame['accepted']=answer is True;return answer
            return counted
        return decorate
