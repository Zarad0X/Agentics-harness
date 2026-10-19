"""Read-only observation proxies around the byte-exact deployed helper.

No source setters, pose normalization, inferred flags, retry, or repair.
"""
import runtime_profile as timing
import hashlib,json
import numpy as np
from sleep_state_copy import ObjectSleepCopy

@timing.measured('copy')
def snapshot(objects,cls):
    rows={}
    for oid,obj in objects.items():
        actor=obj.actor;body=actor.find_component_by_type(cls);row={}
        for field,get in [('p',lambda:actor.get_pose().p),('q',lambda:actor.get_pose().q),('linear',lambda:body.get_linear_velocity()),('angular',lambda:body.get_angular_velocity()),('sleeping',lambda:body.is_sleeping)]:
            try:
                value=get()
                if field=='sleeping':
                    if not isinstance(value,(bool,np.bool_)):raise TypeError('sleep_flag_not_bool')
                    row[field]=bool(value)
                else:
                    a=np.array(value,copy=True);row[field]=a.tolist()
                row[field+'_available']=True
            except Exception as exc:row[field+'_available']=False;row[field+'_error']=type(exc).__name__+':'+str(exc)
        rows[str(oid)]=row
    return rows

def differences(a,b):
    result=[]
    for oid in sorted(set(a)|set(b)):
        for field in ('p','q','linear','angular','sleeping'):
            x=a.get(oid,{});y=b.get(oid,{})
            if not x.get(field+'_available') or not y.get(field+'_available'):
                result.append(dict(object_id=oid,field=field,available=False,before=x,after=y));continue
            aa,bb=x[field],y[field]
            if not np.array_equal(aa,bb):
                delta=None if field=='sleeping' else (np.asarray(bb)-np.asarray(aa)).tolist()
                result.append(dict(object_id=oid,field=field,before=aa,after=bb,delta=delta))
    return result

class JsonTrace:
    def __init__(self,path):
        from action_tape import Writer
        self.writer=Writer(path.parent/'copy-observations',1,profile='copy');self.rows=0;self.bytes=0;self.sha=hashlib.sha256()
    @timing.measured('copy')
    def __call__(self,kind,**values):
        record=dict(kind=kind,sequence=self.rows,**values)
        # Nonfinite native values remain explicit diagnostic text, never valid helper state.
        def clean(v):
            if isinstance(v,float) and not np.isfinite(v):return {'nonfinite':repr(v)}
            if isinstance(v,dict):return {k:clean(x) for k,x in v.items()}
            if isinstance(v,(list,tuple)):return [clean(x) for x in v]
            return v
        data=__import__('provenance').canonical(clean(record))
        self.writer.append(data);self.sha.update(data);self.rows+=1;self.bytes+=len(data)
    def close(self):
        self.writer.close()
        return json.loads((self.writer.folder/'terminal.json').read_bytes())

class BodyProxy:
    def __init__(self,body,oid,observe):self.body=body;self.oid=oid;self.observe=observe
    @property
    def is_sleeping(self):return self.body.is_sleeping
    def get_linear_velocity(self):return self.body.get_linear_velocity()
    def get_angular_velocity(self):return self.body.get_angular_velocity()
    def _call(self,method):
        before=self.observe('before',self.oid,method,None);error=None
        try:return getattr(self.body,method)()
        except BaseException as exc:error=exc;raise
        finally:
            try:self.observe('after',self.oid,method,before,error)
            except BaseException:
                if error is None:raise
                # Keep the native failure authoritative; probe result is incomplete.
    def put_to_sleep(self):return self._call('put_to_sleep')
    def wake_up(self):return self._call('wake_up')

class ActorProxy:
    def __init__(self,actor,body,cls):self.actor=actor;self.body=body;self.cls=cls
    def get_pose(self):return self.actor.get_pose()
    def find_component_by_type(self,cls):assert cls is self.cls;return self.body

class Wrapper:
    def __init__(self,actor):self.actor=actor
