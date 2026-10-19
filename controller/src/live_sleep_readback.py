"""Read-only identity retention and explicit nonfinite force-limit wire values."""
import math


def force_limit(value):
    if math.isnan(value):raise ValueError('nan_native_force_limit')
    if math.isinf(value):return {'native_force_limit':'+inf' if value>0 else '-inf'}
    return value


def decode_force_limit(value):
    if isinstance(value,dict):
        if value=={'native_force_limit':'+inf'}:return float('inf')
        if value=={'native_force_limit':'-inf'}:return float('-inf')
        raise ValueError('invalid_force_limit_tag')
    if isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value):
        raise ValueError('untagged_or_invalid_force_limit')
    return value


def validate_controls(controls):
    for joints in controls.values():
        for joint in joints:
            decode_force_limit(joint['force_limit'])


class SystemReferences:
    """Retain env and its pybind wrapper together through owner lifetime."""
    def __init__(self):self.held={}
    def retain(self,env):
        key=id(env)
        if key not in self.held:self.held[key]=(env,env.scene.physx_system)
        return self.held[key][1]
    def pair(self,source,destination):
        left=self.retain(source);right=self.retain(destination)
        repeated=[[source.scene.physx_system is left,destination.scene.physx_system is right] for _ in range(3)]
        same=left is right;equal=bool(left==right)
        return dict(schema='retained-native-system-pair/1',source_system=str(id(left)),destination_system=str(id(right)),
            repeated_accessor_same=repeated,same_wrapper=same,same_system_equal=equal,
            independent=all(all(r) for r in repeated) and not same and not equal,
            basis='simultaneously held pybind wrappers; stable accessor identity and pairwise identity/equality')


def validate_identity(evidence,source,destination):
    if (evidence.get('schema')!='retained-native-system-pair/1' or
        evidence.get('source_system')!=source or evidence.get('destination_system')!=destination or
        not isinstance(source,str) or not source or not isinstance(destination,str) or not destination or source==destination or
        evidence.get('repeated_accessor_same')!=[[True,True]]*3 or
        evidence.get('same_wrapper') is not False or evidence.get('same_system_equal') is not False or
        evidence.get('independent') is not True):raise ValueError('native_system_alias_or_unstable_accessor')
