import copy,hashlib,importlib.util,io,json,multiprocessing,os,re,shutil,sys,tempfile,types,unittest
from pathlib import Path
from unittest import mock
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from evolution import Store,CONTRACT,SCOPE,encoded,sha,digest,assess,brief,link_tested_fix,Invalid,scorecard,read_allowed,clean
import usage_accounting as usage
from parent_adapter import import_usage,record_outcome,completion_observed,finish_dispatch
IN=ROOT.parent.parent/'incoming'

def numbers(inp,cache=None,out=10):
    return dict(input_tokens=inp,cached_input_tokens=inp//2 if cache is None else cache,cache_write_input_tokens=0,output_tokens=out,reasoning_output_tokens=out//2)
def empty_ledger():
    return dict(schema='roundtrip-development-usage/1',rows=[],total_incremental={k:0 for k in usage.FIELDS},semantics='existing session cumulative authority')
def status(name,sid,values,origin=None):
    return dict(name=name,session_id=sid,usage=values,completed=True,ssh_exit=0,usage_origin=origin or {'kind':'new_session'},
                completed_event=dict(session_id=sid,usage=values,line_sha256=sha(encoded([name,sid,values])),ordinal=1))
def update(path,name,s):
    return usage.append_completed(path,[name],lambda n:(s,dict(kind='synthetic-completed-source',event_sha256=sha(encoded(s['completed_event'])))))
def writer(path,name,s,queue):
    try:queue.put(dict(ok=True,added=update(path,name,s)['added']))
    except Exception as e:queue.put(dict(ok=False,error=str(e)))
def helper_writer(parent,name,queue):
    try:
        parent=Path(parent);cs=module(parent/'completed_usage_source.py','completed_usage_source')
        with mock.patch.dict(sys.modules,{'completed_usage_source':cs}):
            app=module(parent/'append_completed_usage.py','append_completed_usage')
            queue.put(dict(ok=True,added=app.append_names([name])['added']))
    except Exception as e:queue.put(dict(ok=False,error=str(e)))
def module(path,name):
    spec=importlib.util.spec_from_file_location(name,str(path));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def fixture(store):
    source=store.put(encoded({'synthetic_candidate':'boundary-only'}));config=store.put(encoded({'synthetic_config':True}))
    counter=store.put(encoded({'historical_input':'boundary fixture; not robot execution'}));negative=store.put(encoded({'unsafe':True}));control=store.put(encoded({'safe':True}));output=store.put(encoded({'synthetic_observation':'boundary assertions only'}))
    old=dict(id='fixture/1',kind='regression',counterexamples=[counter],baseline_source_sha256=source,negative_checks=['unsafe'],control_checks=['safe'],status='unresolved',objects=[counter,source])
    store.register(old)
    expected=dict({'old-counterexample':'reject','candidate-counterexample':'accept'},unsafe='reject',safe='accept')
    reg=dict(old,id='fixture/2',revision=2,supersedes='fixture/1',behavior_schema='regression-behavior/2',
             behaviors=dict(corrective=expected,preservation=dict(expected,**{'candidate-counterexample':'reject'})))
    store.register(reg)
    candidate=dict(source_sha256=source,config_sha256=config,baseline_config_sha256=config,contract=CONTRACT,split='development',receipts={},intents={},training_evidence=[])
    review=dict(candidate_sha256=None,baseline_config_sha256=config,reviewed_receipts=[],reviewed_registries=[digest({k:v for k,v in reg.items() if k!='objects'})])
    def receipt(intent,r=reg):
        expectations=r['behaviors'][intent];rh=digest({k:v for k,v in r.items() if k!='objects'})
        cases=[dict(id=n,input_sha256=counter if n.endswith('counterexample') else negative if n=='unsafe' else control,output_sha256=output,actual=v,expected=v) for n,v in expectations.items()]
        row=dict(id='receipt:'+r['id']+':'+intent,kind='regression-receipt',registry_sha256=rh,intent=intent,source_sha256=source,config_sha256=config,baseline_source_sha256=source,
                 counterexample=counter,contract=CONTRACT,scope='local-regression',cases=cases)
        store.register(row);candidate['receipts']={r['id']:row['id']};candidate['intents']={r['id']:intent}
        review['reviewed_receipts']=[digest(row)];review['candidate_sha256']=digest(candidate)
        return row
    return candidate,review,reg,receipt

class RevisionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(dir=str(ROOT/'tests'));self.root=Path(self.tmp.name);self.s=Store(self.root/'store')
        self.authority=self.root/'development-usage-ledger.json';self.authority.write_bytes(encoded(empty_ledger()))
    def tearDown(self):self.tmp.cleanup()
    def test_preservation_reject_reject_stays_unresolved(self):
        c,r,g,receipt=fixture(self.s);receipt('preservation')
        result=assess(self.s,c,r);self.assertTrue(result['diagnostic_launch_eligible']);self.assertEqual(result['preserved'],['fixture/2']);self.assertEqual(result['corrected'],[])
        self.s.register(dict(id='problem@1',kind='issue',issue_id='problem',version=1,scope={k:None for k in SCOPE},symptom='unsafe input correctly rejected',cause=None,hypothesis=None,confidence='fixture',regressions=['fixture/2'],status='unresolved',objects=[]))
        with self.assertRaises(Invalid):link_tested_fix(self.s,'problem',c,r)
        self.assertEqual(self.s.issues()[0]['status'],'unresolved')
    def test_corrective_source_bound_expected_behavior(self):
        c,r,g,receipt=fixture(self.s);receipt('corrective');result=assess(self.s,c,r)
        self.assertTrue(result['diagnostic_launch_eligible']);self.assertEqual(result['corrected'],['fixture/2']);self.assertFalse(result['adoption_allowed'])
        c['source_sha256']=self.s.put(encoded({'other_source':True}));r['candidate_sha256']=digest(c)
        self.assertIn('fixture/2: source_sha256 mismatch',assess(self.s,c,r)['errors'])
    def test_omitted_brief_not_gate_omission(self):
        c,r,g,receipt=fixture(self.s);receipt('preservation');self.assertEqual(brief(self.s,'unrelated')['issues'],[])
        c['receipts']={};r['candidate_sha256']=digest(c)
        self.assertIn('fixture/2: missing regression evidence',assess(self.s,c,r)['errors'])
    def test_weakened_negative_needs_reviewed_registry_revision(self):
        c,r,g,receipt=fixture(self.s);old=receipt('preservation')
        altered=copy.deepcopy(old);altered['id']='weakened';next(x for x in altered['cases'] if x['id']=='unsafe').update(actual='accept',expected='accept')
        self.s.register(altered);c['receipts']['fixture/2']='weakened';r['reviewed_receipts']=[digest(altered)];r['candidate_sha256']=digest(c)
        self.assertIn('fixture/2: absent/failed unsafe',assess(self.s,c,r)['errors'])
        g3=copy.deepcopy(g);g3.update(id='fixture/3',revision=3,supersedes='fixture/2');g3['behaviors']['preservation']['unsafe']='accept'
        self.s.register(g3);receipt('preservation',g3)
        self.assertIn('fixture/3: reviewed registry revision missing',assess(self.s,c,r)['errors'])
    def test_revision_cannot_fork_or_erase_prior(self):
        c,r,g,receipt=fixture(self.s)
        with self.assertRaises(Invalid):self.s.register(dict(g,id='fork/2'))
        self.assertEqual([x['id'] for x in self.s.records('regression')],['fixture/1','fixture/2'])
    def test_authoritative_actual_resumed_increment(self):
        raw=(IN/'development-usage-ledger.json').read_bytes();ledger=json.loads(raw);usage.validate(ledger)
        receipt=usage.selected_receipt(raw,ledger,'diagnose014-slip-onset-candidate-002')
        self.assertEqual(receipt['row']['raw_session_cumulative']['input_tokens'],10185518)
        self.assertEqual(receipt['incremental']['input_tokens'],5217731)
        self.assertEqual(receipt['incremental']['cached_input_tokens'],4915840)
        import_usage(self.s.root,IN/'development-usage-ledger.json',receipt['delegation'])
        self.assertFalse(import_usage(self.s.root,IN/'development-usage-ledger.json',receipt['delegation'])['imported'])
        a=self.s.attempts()[0];self.assertEqual(a['tokens']['total_tokens'],5217731+47508);self.assertEqual(a['accounting_status'],'verified')
        self.assertEqual(scorecard(self.s)['verified_development_tokens']['input_tokens'],5217731)
    def test_historical_rows_and_recovered_provenance_not_rewritten(self):
        raw=(IN/'development-usage-ledger.json').read_bytes();self.authority.write_bytes(raw)
        rows=json.loads(raw)['rows'];recovered=next(r for r in rows if r.get('completion_evidence'))
        wanted=[rows[-1],recovered]
        def resolver(name):
            row=next(r for r in wanted if r['delegation']==name)
            return dict(completed=True,session_id=row['session_id'],usage=row['raw_session_cumulative']),row.get('completion_evidence')
        result=usage.append_completed(self.authority,[r['delegation'] for r in wanted],resolver)
        self.assertEqual(result['added'],0);self.assertEqual(self.authority.read_bytes(),raw)
        self.assertEqual(result['receipts'][1]['row']['completion_evidence'],recovered['completion_evidence'])
    def test_scanner_source_literal_is_data_real_key_header_refused(self):
        source=ROOT.parent/'evolution-harness-001/src/evolution.py'
        if not source.exists():source=ROOT/'fixtures/previous-evolution.py'
        self.assertEqual(sha(read_allowed(source)),sha(source.read_bytes()))
        with self.assertRaises(Invalid):clean('-----BEGIN PRIVATE KEY-----\nSYNTHETIC ONLY')
    def test_concurrent_updates_duplicates_preserve_rows(self):
        baseline=status('baseline','base',numbers(50));update(self.authority,'baseline',baseline);before=json.loads(self.authority.read_bytes())['rows'][0]
        ctx=multiprocessing.get_context('fork');q=ctx.Queue();workers=[]
        for i in [0,1,2,3,2,3]:
            name='job'+str(i);s=status(name,'session'+str(i),numbers(100+i))
            workers.append(ctx.Process(target=writer,args=(self.authority,name,s,q)))
        for w in workers:w.start()
        for w in workers:w.join(15);self.assertEqual(w.exitcode,0)
        results=[q.get(timeout=3) for _ in workers];self.assertTrue(all(x['ok'] for x in results))
        ledger=json.loads(self.authority.read_bytes());usage.validate(ledger);self.assertEqual(len(ledger['rows']),5);self.assertEqual(ledger['rows'][0],before)
        self.assertEqual(sum(x['added'] for x in results),4)
    def test_exact_accounting_helpers_concurrent_completions(self):
        parent=self.root/'helpers';parent.mkdir();shutil.copyfile(self.authority,parent/self.authority.name)
        for name in ('append_completed_usage','completed_usage_source'):
            shutil.copyfile(ROOT/'adapters'/(name+'.py.patched'),parent/(name+'.py'))
        for i in range(4):
            p=parent/'delegations'/('job'+str(i));p.mkdir(parents=True)
            (p/'status.json').write_bytes(encoded(status('job'+str(i),'session'+str(i),numbers(100+i))))
        ctx=multiprocessing.get_context('fork');q=ctx.Queue();workers=[ctx.Process(target=helper_writer,args=(str(parent),'job'+str(i),q)) for i in (0,1,2,3,2)]
        for w in workers:w.start()
        for w in workers:w.join(15);self.assertEqual(w.exitcode,0)
        results=[q.get(timeout=3) for w in workers];self.assertTrue(all(r['ok'] for r in results),results)
        ledger=json.loads((parent/self.authority.name).read_bytes());usage.validate(ledger)
        self.assertEqual(len(ledger['rows']),4);self.assertEqual(sum(x['added'] for x in results),4)
        self.assertTrue(all(r['completed_source']['kind']=='completed_dispatch_status' and r['event_id'] for r in ledger['rows']))
    def test_decreased_counters_and_unknown_history_no_write(self):
        s=status('first','sid',numbers(100));update(self.authority,'first',s);before=self.authority.read_bytes()
        with self.assertRaises(usage.AccountingError):update(self.authority,'decrease',status('decrease','sid',numbers(90),usage.session_origin(self.authority,'sid')))
        with self.assertRaises(usage.AccountingError):update(self.authority,'unknown',status('unknown','missing',numbers(200),usage.session_origin(self.authority,'missing')))
        self.assertEqual(self.authority.read_bytes(),before)
    def test_stale_predecessor_out_of_order_reconcile(self):
        update(self.authority,'one',status('one','sid',numbers(100)))
        origin=usage.session_origin(self.authority,'sid')
        two=status('two','sid',numbers(150,out=20),origin)
        expected=dict(kind='resumed_session',session_id='sid',predecessor_delegation='two',predecessor_cumulative_sha256=usage.digest(two['usage']))
        three=status('three','sid',numbers(200,out=30),expected)
        with self.assertRaises(usage.AccountingError):update(self.authority,'three',three)
        update(self.authority,'two',two);update(self.authority,'three',three)
        self.assertEqual([r['incremental']['input_tokens'] for r in json.loads(self.authority.read_bytes())['rows']],[100,50,50])
    def test_historical_offsets_preserved(self):
        update(self.authority,'one',status('one','sid',numbers(100,out=20)))
        ledger=json.loads(self.authority.read_bytes());row=ledger['rows'][0];row['incremental']=numbers(60,out=10);row['uncached_input_tokens']=30
        ledger['total_incremental']=dict(row['incremental']);self.authority.write_bytes(encoded(ledger));old=copy.deepcopy(row)
        update(self.authority,'two',status('two','sid',numbers(150,out=30),usage.session_origin(self.authority,'sid')))
        ledger=json.loads(self.authority.read_bytes());self.assertEqual(ledger['rows'][0],old);self.assertEqual(ledger['rows'][1]['incremental']['input_tokens'],50)
    def test_duplicate_changed_event_and_alias_refused(self):
        s=status('one','sid',numbers(100));update(self.authority,'one',s)
        changed=copy.deepcopy(s);changed['completed_event']['line_sha256']='0'*64
        with self.assertRaises(usage.AccountingError):update(self.authority,'one',changed)
        with self.assertRaises(usage.AccountingError):update(self.authority,'alias',s)
        self.assertEqual(len(json.loads(self.authority.read_bytes())['rows']),1)
    def test_atomic_failure_preserves_authority(self):
        before=self.authority.read_bytes()
        with mock.patch.object(usage.os,'replace',side_effect=OSError('synthetic disk error')):
            with self.assertRaises(OSError):update(self.authority,'one',status('one','sid',numbers(100)))
        self.assertEqual(self.authority.read_bytes(),before)
    def test_completion_monotonic_and_accounting_failure_separate(self):
        s=status('task','sid',numbers(100));completion_observed(self.s.root,'task','sid',s['completed_event'])
        result=finish_dispatch(self.s.root,self.root,'task','sid',s['completed_event'],lambda names:(_ for _ in ()).throw(usage.AccountingError('unknown history')))
        self.assertEqual(result['accounting_status'],'unknown');self.assertTrue(result['model_completed'])
        record_outcome(self.s.root,'dispatch:task','failed','transport_timeout',domain='orchestration')
        a=next(a for a in self.s.attempts() if a['attempt_id']=='dispatch:task');self.assertEqual(a['outcome'],'completed');self.assertIsNone(a['tokens'])
        update(self.authority,'task',s);import_usage(self.s.root,self.authority,'task')
        record_outcome(self.s.root,'dispatch:task','unknown','late_status',domain='orchestration')
        a=next(a for a in self.s.attempts() if a['attempt_id']=='dispatch:task');self.assertEqual(a['accounting_status'],'verified')
    def test_concurrent_idempotent_authority_import_after_growth(self):
        update(self.authority,'one',status('one','sid',numbers(100)));import_usage(self.s.root,self.authority,'one')
        update(self.authority,'two',status('two','other',numbers(50)))
        self.assertFalse(import_usage(self.s.root,self.authority,'one')['imported']);self.assertEqual(len(self.s.attempts()),1)
    def test_unverified_scalar_usage_never_authoritative_total(self):
        record_outcome(self.s.root,'legacy','completed',None,usage=dict(input_tokens=999,cached_input_tokens=900,output_tokens=10),domain='orchestration')
        self.assertEqual(scorecard(self.s)['verified_development_tokens']['input_tokens'],0)
        self.assertEqual(self.s.attempts()[0]['accounting_status'],'unverified_legacy')
    def test_corrected_dispatch_actual_wrappers_with_stubs(self):
        self._dispatch_case(False)
    def test_dispatch_post_completion_accounting_unknown_not_model_failed(self):
        self._dispatch_case(True)
    def _dispatch_case(self,break_accounting):
        parent=self.root/'parent';parent.mkdir()
        for name in ('dispatch22_data3','append_completed_usage','completed_usage_source'):
            shutil.copyfile(ROOT/'adapters'/(name+'.py.patched'),parent/(name+'.py'))
        shutil.copyfile(self.authority,parent/'development-usage-ledger.json')
        store=Store(parent/'evolution-ledger');fixture(store)
        store.register(dict(id='capture@1',kind='issue',issue_id='capture',version=1,scope={k:'development' if k=='split' else 'capture' if k=='stage' else None for k in SCOPE},symptom='capture',cause=None,hypothesis=None,confidence='stub',regressions=['fixture/2'],status='unresolved',objects=[]))
        authority=parent/'development-usage-ledger.json';update(authority,'previous',status('previous','session',numbers(100)))
        prompt=parent/'task.md';prompt.write_text('Test local integration boundary')
        cs=module(parent/'completed_usage_source.py','completed_usage_source')
        fake=types.ModuleType('codex22');fake.ensure_proxy=lambda:None;fake.REMOTE_CODEX='stub';fake.UUID=re.compile('session');fake.load_sessions=lambda:{'session':{'cwd':'/data4/yuantao/astra-roundtrip-20261009'}};fake.save_sessions=lambda s:None;fake.ssh_base=lambda:['stub'];fake.remote_command=lambda c:'stub'
        values=numbers(170,cache=80,out=20)
        events=[dict(type='thread.started',thread_id='session'),dict(type='turn.completed',usage=values)]
        class Input(io.StringIO):
            def close(inner):inner.saved=inner.getvalue()
        proc=types.SimpleNamespace(stdin=Input(),stdout=io.StringIO(''.join(json.dumps(e)+'\n' for e in events)),wait=lambda:0)
        with mock.patch.dict(sys.modules,{'codex22':fake,'completed_usage_source':cs}):
            app=module(parent/'append_completed_usage.py','append_completed_usage')
            with mock.patch.dict(sys.modules,{'append_completed_usage':app}):
                dispatch=module(parent/'dispatch22_data3.py','dispatch_fixture')
                if break_accounting:dispatch.append_names=lambda names:(_ for _ in ()).throw(usage.AccountingError('synthetic accounting unavailable'))
                with mock.patch.object(dispatch.subprocess,'run',return_value=types.SimpleNamespace(returncode=0,stdout=json.dumps({'official_provider_verified':True,'account':{'type':'chatgpt'}}))),mock.patch.object(dispatch.subprocess,'Popen',return_value=proc),mock.patch.object(sys,'argv',['dispatch','task',str(prompt),'--resume','session','--experience-stage','capture']):
                    with self.assertRaises(SystemExit) as exit:dispatch.main()
        self.assertEqual(exit.exception.code,2 if break_accounting else 0)
        attempts=store.attempts();a=next(a for a in attempts if a['attempt_id']=='dispatch:task');self.assertEqual(a['outcome'],'completed')
        if break_accounting:self.assertIsNone(a['tokens'])
        else:
            self.assertEqual(a['tokens']['input_tokens'],70);self.assertEqual(a['tokens']['cached_input_tokens'],30)
            row=json.loads(authority.read_bytes())['rows'][-1];self.assertIn('completed_event',row['completed_source'])
        self.assertIn('engineering_evidence_data',proc.stdin.saved)
        for record in store.records():self.assertNotIn('official_provider_verified',json.dumps(record))

if __name__=='__main__':unittest.main(verbosity=2)
