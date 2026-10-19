"""Task-owned CPU runner. Fail closed unless kernel Landlock write policy installs."""
import ctypes, hashlib, json, math, os, pathlib, resource, signal, subprocess, sys, time

ROOT = pathlib.Path(__file__).resolve().parent
BASE = ROOT.parent / ROOT.name
METER = ROOT / 'cpu-budget.json'

def boundary():
    libc = ctypes.CDLL(None, use_errno=True)
    abi = libc.syscall(444, 0, 0, 1)
    if abi < 1:
        raise RuntimeError(f'Landlock required, got {abi}')
    writes = sum(1 << b for b in (1, 4, 5, 6, 7, 8, 9, 10, 11, 12))
    if abi >= 2: writes |= 1 << 13
    if abi >= 3: writes |= 1 << 14
    class Ruleset(ctypes.Structure):
        _fields_ = [('handled_access_fs', ctypes.c_uint64)]
    class Rule(ctypes.Structure):
        _pack_ = 1
        _fields_ = [('allowed_access', ctypes.c_uint64), ('parent_fd', ctypes.c_int32)]
    attr = Ruleset(writes)
    fd = libc.syscall(444, ctypes.byref(attr), ctypes.sizeof(attr), 0)
    if fd < 0:
        raise OSError(ctypes.get_errno(), 'create Landlock ruleset')
    for p in (ROOT,):
        pfd = os.open(p, os.O_PATH | os.O_CLOEXEC)
        rule = Rule(writes if p.is_dir() else (1 << 1) | ((1 << 14) if abi >= 3 else 0), pfd)
        if libc.syscall(445, fd, 1, ctypes.byref(rule), 0):
            raise OSError(ctypes.get_errno(), 'add Landlock rule')
        os.close(pfd)
    if libc.prctl(38, 1, 0, 0, 0) or libc.syscall(446, fd, 0):
        raise OSError(ctypes.get_errno(), 'enforce Landlock')
    os.close(fd)
    # Close ABI-1 truncation holes, including read-only O_TRUNC and openat2.
    sec = ctypes.CDLL('libseccomp.so.2', use_errno=True)
    sec.seccomp_init.argtypes = [ctypes.c_uint32]
    sec.seccomp_init.restype = ctypes.c_void_p
    sec.seccomp_syscall_resolve_name.argtypes = [ctypes.c_char_p]
    sec.seccomp_rule_add_array.argtypes = [ctypes.c_void_p,ctypes.c_uint32,ctypes.c_int,ctypes.c_uint,ctypes.c_void_p]
    sec.seccomp_load.argtypes = [ctypes.c_void_p]
    sec.seccomp_release.argtypes = [ctypes.c_void_p]
    ctx = sec.seccomp_init(0x7fff0000)
    if not ctx: raise RuntimeError('seccomp_init failed')
    class Compare(ctypes.Structure):
        _fields_ = [('arg',ctypes.c_uint),('op',ctypes.c_uint),('mask',ctypes.c_uint64),('value',ctypes.c_uint64)]
    for name in ('truncate','ftruncate','openat2','io_uring_setup'):
        nr=sec.seccomp_syscall_resolve_name(name.encode())
        if nr >= 0 and sec.seccomp_rule_add_array(ctx,0x50001,nr,0,None):
            raise RuntimeError('seccomp deny failed: '+name)
    for name,arg in (('open',1),('openat',2)):
        nr=sec.seccomp_syscall_resolve_name(name.encode())
        cmp=Compare(arg,7,os.O_ACCMODE | os.O_TRUNC,os.O_TRUNC)
        if sec.seccomp_rule_add_array(ctx,0x50001,nr,1,ctypes.byref(cmp)):
            raise RuntimeError('seccomp flags failed: '+name)
    if sec.seccomp_load(ctx): raise RuntimeError('seccomp_load failed')
    sec.seccomp_release(ctx)
    return abi


def restrict(output):
    global ROOT
    ROOT = pathlib.Path(output).resolve(strict=True)
    return boundary()
