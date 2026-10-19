"""Upload one bounded project reference with compression and atomic, write-once publication."""

import argparse
import base64
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parents[1] / "outputs"))
import codex22

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("source", type=Path)
parser.add_argument("name", nargs="?")
parser.add_argument("--timeout", type=int, default=150)
args = parser.parse_args()
assert 1 <= args.timeout <= 900
source = args.source.resolve()
assert source.parent == ROOT and source.is_file()
name = args.name or source.name
assert re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{1,120}", name)
data = source.read_bytes()
assert len(data) <= 16 * 1024**2
packet = dict(
    name=name,
    sha256=hashlib.sha256(data).hexdigest(),
    bytes=len(data),
    payload=base64.b64encode(gzip.compress(data)).decode(),
)
script = r"""
from pathlib import Path
import base64,gzip,hashlib,json,os,re,sys,tempfile
p=json.load(sys.stdin)
root=Path('/data4/yuantao/astra-roundtrip-20261009/reference')
assert re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{1,120}',p['name'])
data=gzip.decompress(base64.b64decode(p['payload']))
assert len(data)==p['bytes']<=16*1024**2
assert hashlib.sha256(data).hexdigest()==p['sha256']
out=root/p['name']
if out.exists():
 assert out.read_bytes()==data,'preserve conflicting existing reference'
else:
 fd,tmp=tempfile.mkstemp(prefix='.reference-upload-',dir=root)
 try:
  with os.fdopen(fd,'wb') as stream:
   stream.write(data);stream.flush();os.fsync(stream.fileno())
  os.chmod(tmp,0o444)
  try:os.link(tmp,out)
  except FileExistsError:assert out.read_bytes()==data,'concurrent reference mismatch'
 finally:os.unlink(tmp)
assert hashlib.sha256(out.read_bytes()).hexdigest()==p['sha256']
print(json.dumps({'path':str(out),'bytes':len(data),'sha256':p['sha256']}))
"""
codex22.ensure_proxy()
result = subprocess.run(
    codex22.ssh_base() + [codex22.remote_command(["python3", "-c", script])],
    input=json.dumps(packet),
    text=True,
    capture_output=True,
    timeout=args.timeout,
)
if result.returncode:
    raise SystemExit(result.stderr[-1500:])
print(result.stdout.strip())
