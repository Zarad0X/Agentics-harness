"""Bounded, lossless scientific-report transport. No native or physical logic."""
import gzip
import hashlib
import io
import json
import sys
from pathlib import Path

RAW_LIMIT=32*1024*1024
GZIP_LIMIT=900*1024
SUMMARY_LIMIT=64*1024
OUTPUT_PREFLIGHT_LIMIT=2*1024*1024 # leaves >1.5MiB for diagnostic/log/terminal
FULL_NAME='paired-native-report.full.json.gz'

class SerializationFailure(RuntimeError):pass

class CappedBuffer(io.BytesIO):
    def write(self,data):
        if self.tell()+len(data)>GZIP_LIMIT:raise SerializationFailure('compressed_report_budget')
        return super().write(data)

def pack(report):
    buffer=CappedBuffer();h=hashlib.sha256();size=0
    with gzip.GzipFile(fileobj=buffer,mode='wb',mtime=0) as stream:
        for text in json.JSONEncoder(sort_keys=True,allow_nan=False,separators=(',',':')).iterencode(report):
            data=text.encode('utf-8');size+=len(data)
            if size>RAW_LIMIT:raise SerializationFailure('uncompressed_report_budget')
            h.update(data);stream.write(data)
    data=buffer.getvalue()
    return data,dict(path=FULL_NAME,encoding='gzip',compressed_size=len(data),
        compressed_sha256=hashlib.sha256(data).hexdigest(),uncompressed_size=size,
        uncompressed_sha256=h.hexdigest(),uncompressed_limit=RAW_LIMIT)

def scientific_summary(report):
    results={}
    for name,row in report.get('results',{}).items():
        fields=('failure','scope','preview_steps','formal_steps','source_unchanged',
                'separation_continuation_completed','source_restored','no_live_drive',
                'action_count','action_sha256')
        result={k:row[k] for k in fields if k in row}
        # A pathological error must not prevent the bounded failure receipt.
        for k,v in list(result.items()):
            if isinstance(v,str) and len(v)>4096:
                result[k]=dict(prefix=v[:4096],characters=len(v),sha256=hashlib.sha256(v.encode()).hexdigest(),truncated=True)
        result.update(event_count=len(row.get('events',[])),boundary_count=len(row.get('boundaries',[])))
        results[name]=result
    return dict(scientific_status=report.get('status'),results=results,physics=report.get('physics'),
        baseline_reproduced=report.get('baseline_reproduced'),
        candidate_separation_continuation_completed=report.get('candidate_separation_continuation_completed'),
        both_initializations_passed=report.get('both_initializations_passed'),
        exact_new_action_prefix_and_boundary_match=report.get('exact_new_action_prefix_and_boundary_match'))

def write_report(out,report,write):
    out=Path(out)
    summary=dict(schema='native-forecast-report-envelope/1',status='SERIALIZATION_FAILED',
        scope=report.get('scope'),formal_credit=False,goal_complete=False,
        qualification=False,serialization_complete=False,full_report=None,action_artifacts=[],
        **scientific_summary(report))
    try:
        data,binding=pack(report)
        for path in sorted(out.glob('*-actions.jsonl.gz')):
            if path.stat().st_size>1024*1024:raise SerializationFailure('action_file_budget')
            summary['action_artifacts'].append(dict(path=path.name,size=path.stat().st_size,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        summary.update(status=report['status'],serialization_complete=True,full_report=binding)
        raw=json.dumps(summary,sort_keys=True,indent=2,allow_nan=False).encode()
        if len(raw)>SUMMARY_LIMIT:raise SerializationFailure('summary_budget')
        existing=sum(p.stat().st_size for p in out.rglob('*') if p.is_file())
        if existing+len(data)+len(raw)+1>OUTPUT_PREFLIGHT_LIMIT:raise SerializationFailure('aggregate_output_reserve')
        with (out/FULL_NAME).open('xb') as stream:stream.write(data)
        write(out/'paired-native-report.json',summary)
    except Exception as exc:
        summary.update(status='SERIALIZATION_FAILED',serialization_complete=False,qualification=False,
            serialization_error=type(exc).__name__+':'+str(exc),full_report=None)
        # The scientific outcome above is unqualified; never replace it by PASS.
        try:
            write(out/'paired-native-report.json',summary)
        except Exception as receipt_error:
            print('serialization_failure_receipt_error:'+repr(receipt_error),file=sys.stderr)
        raise
    return summary

def load_report(path):
    path=Path(path)
    if path.stat().st_size>SUMMARY_LIMIT:raise SerializationFailure('summary_budget')
    summary=json.loads(path.read_bytes())
    if summary.get('schema')!='native-forecast-report-envelope/1' or summary.get('serialization_complete') is not True:
        raise SerializationFailure('incomplete_report_serialization')
    b=summary['full_report']
    if b['path']!=FULL_NAME or b['encoding']!='gzip':raise SerializationFailure('report_path_or_encoding')
    full=path.parent/FULL_NAME
    if full.stat().st_size!=b['compressed_size'] or full.stat().st_size>GZIP_LIMIT:raise SerializationFailure('compressed_size')
    data=full.read_bytes()
    if hashlib.sha256(data).hexdigest()!=b['compressed_sha256']:raise SerializationFailure('compressed_sha256')
    if not 0<=b['uncompressed_size']<=RAW_LIMIT:raise SerializationFailure('uncompressed_size')
    with gzip.GzipFile(fileobj=io.BytesIO(data),mode='rb') as stream:
        raw=stream.read(b['uncompressed_size']+1)
    if len(raw)!=b['uncompressed_size'] or hashlib.sha256(raw).hexdigest()!=b['uncompressed_sha256']:
        raise SerializationFailure('uncompressed_binding')
    report=json.loads(raw)
    if scientific_summary(report)!={k:summary[k] for k in scientific_summary(report)} or summary['status']!=report['status']:
        raise SerializationFailure('summary_full_disagreement')
    required={name+'-actions.jsonl.gz' for name,row in report.get('results',{}).items() if 'action_count' in row}
    if {a['path'] for a in summary['action_artifacts']}!=required:raise SerializationFailure('action_artifact_set')
    for artifact in summary['action_artifacts']:
        name=artifact['path']
        if Path(name).name!=name or not name.endswith('-actions.jsonl.gz'):raise SerializationFailure('action_path')
        action=path.parent/name
        if action.stat().st_size!=artifact['size'] or artifact['size']>1024*1024:raise SerializationFailure('action_size')
        if hashlib.sha256(action.read_bytes()).hexdigest()!=artifact['sha256']:raise SerializationFailure('action_sha256')
    return report
