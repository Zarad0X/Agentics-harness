"""Query server22's own Codex account metadata and allowance without credentials."""

from pathlib import Path
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parents[1] / "outputs"))
import codex22

script = r"""
import json,subprocess,queue,threading,time,os
from urllib.parse import urlparse
exe='/home/yuantao/.nvm/versions/node/v24.19.0/bin/codex'
env=os.environ.copy()
for k in ('OPENAI_API_KEY','CODEX_API_KEY','OPENAI_BASE_URL'):env.pop(k,None)
status=subprocess.run([exe,'login','status'],env=env,capture_output=True,text=True,timeout=20)
s=status.stdout+status.stderr
report={'observed_unix':time.time(),'login_chatgpt':status.returncode==0 and 'ChatGPT' in s}
p=subprocess.Popen([exe,'-C','/data4/yuantao/astra-roundtrip-20261009','-c','model_provider="openai"','app-server'],env=env,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True,bufsize=1)
q=queue.Queue()
def reader():
 for line in p.stdout:
  try:q.put(json.loads(line))
  except ValueError:pass
threading.Thread(target=reader,daemon=True).start()
def rpc(i,method,params):
 p.stdin.write(json.dumps({'id':i,'method':method,'params':params})+'\n');p.stdin.flush();end=time.monotonic()+30
 while time.monotonic()<end:
  try:d=q.get(timeout=max(.01,end-time.monotonic()))
  except queue.Empty:break
  if d.get('id')==i:return d
 return {'error':{'message':'timeout'}}
try:
 report['initialized']='result' in rpc(0,'initialize',{'clientInfo':{'name':'robotwin_goal_check','version':'1.0'}})
 p.stdin.write(json.dumps({'method':'initialized','params':{}})+'\n');p.stdin.flush()
 d=rpc(1,'account/read',{'refreshToken':False});a=d.get('result',{}).get('account') or {}
 report['account']={'type':a.get('type'),'planType':a.get('planType')}
 if 'error' in d:report['account_error']=d['error']
 d=rpc(2,'account/rateLimits/read',{});r=d.get('result',{})
 report['limits']=r.get('rateLimitsByLimitId') or r.get('rateLimits')
 if 'error' in d:report['limits_error']=d['error']
 d=rpc(3,'config/read',{'includeLayers':False,'cwd':'/data4/yuantao/astra-roundtrip-20261009'})
 if 'result' in d:
  cfg=d['result'].get('config',{});provider=(cfg.get('model_providers') or {}).get('openai')
  host=urlparse((provider or {}).get('base_url') or '').hostname
  chat_host=urlparse(cfg.get('chatgpt_base_url') or '').hostname
  report['provider_metadata']={'selected':cfg.get('model_provider'),'custom_openai_provider_defined':provider is not None,'provider_base_hostname':host,'chatgpt_base_hostname':chat_host,'provider_has_env_key':bool((provider or {}).get('env_key'))}
  report['official_provider_verified']=cfg.get('model_provider')=='openai' and provider is None and chat_host in (None,'chatgpt.com','chat.openai.com','api.openai.com')
 else:report['provider_config_error_code']=d.get('error',{}).get('code','timeout')
finally:
 p.terminate()
 try:p.wait(timeout=5)
 except subprocess.TimeoutExpired:p.kill();p.wait()
print(json.dumps(report))
"""
codex22.ensure_proxy()
result = subprocess.run(
    codex22.ssh_base() + [codex22.remote_command(["python3", "-c", script])],
    capture_output=True,
    text=True,
    timeout=120,
)
if result.returncode:
    raise SystemExit("Account check transport failed: " + str(result.returncode))
report = json.loads(result.stdout)
path = ROOT / ("account22-" + str(int(report["observed_unix"])) + ".json")
with path.open("x") as stream:
    json.dump(report, stream, indent=2)
print(json.dumps(report, indent=2))
if (
    not report.get("login_chatgpt")
    or not report.get("initialized")
    or report.get("account", {}).get("type") != "chatgpt"
    or not report.get("limits")
    or not report.get("official_provider_verified")
):
    raise SystemExit(2)
