"""Bounded-RAM temporary audit index; never modifies source evidence.
SQLite uses a 1MiB page cache and disk temporary storage. One linear ingest,
indexed lookups, no repeated trace scans. The caller owns scratch cleanup.
"""
import json,sqlite3,shutil,zlib
from provenance import canonical
class Map:
 def __init__(self,db,name):self.db=db;self.name=name;db.execute('CREATE TABLE '+name+' (k TEXT PRIMARY KEY,v BLOB)')
 def __len__(self):return self.db.execute('SELECT count(*) FROM '+self.name).fetchone()[0]
 def __contains__(self,key):return self.db.execute('SELECT 1 FROM '+self.name+' WHERE k=?',(str(key),)).fetchone() is not None
 def __setitem__(self,key,value):
  data=canonical(value)
  if shutil.disk_usage(self.db.execute('PRAGMA database_list').fetchone()[2]).free<64*1024*1024+4*1024*1024+len(data):raise ValueError('copy_audit_scratch_reserve')
  self.db.execute('INSERT OR REPLACE INTO '+self.name+' VALUES (?,?)',(str(key),zlib.compress(data)))
 def __getitem__(self,key):
  row=self.db.execute('SELECT v FROM '+self.name+' WHERE k=?',(str(key),)).fetchone()
  if row is None:raise KeyError(key)
  return json.loads(zlib.decompress(row[0]))
 def get(self,key,default=None):return self[key] if key in self else default
 def items(self):
  for key,value in self.db.execute('SELECT k,v FROM '+self.name):yield key,json.loads(zlib.decompress(value))
 def values(self):
  for _,value in self.items():yield value
 def add(self,key):self[key]=True
class Rows:
 def __init__(self,db,source):
  self.db=db;self.count=0;self.terminals=Map(db,'modes');db.execute('CREATE TABLE rows (i INTEGER PRIMARY KEY,v BLOB)')
  for row in source:
   data=canonical(row)
   if len(data)>512*1024 or self.count>=450_000_001:raise ValueError('copy_audit_record_budget')
   if shutil.disk_usage(db.execute('PRAGMA database_list').fetchone()[2]).free<64*1024*1024+4*1024*1024+4*len(data):raise ValueError('copy_audit_scratch_reserve')
   db.execute('INSERT INTO rows VALUES (?,?)',(self.count,zlib.compress(data)));self.count+=1
   if row['kind']=='copy_terminal':
    cid=row['context']['copy_id']
    if cid in self.terminals:raise ValueError('copy_trace_duplicate_terminal')
    self.terminals[cid]=row.get('initialization','exact')
  db.commit()
 def __len__(self):return self.count
 def __getitem__(self,i):
  if i<0:i+=self.count
  row=self.db.execute('SELECT v FROM rows WHERE i=?',(i,)).fetchone()
  if row is None:raise IndexError(i)
  return json.loads(zlib.decompress(row[0]))
 def __iter__(self):
  for row in self.db.execute('SELECT v FROM rows ORDER BY i'):yield json.loads(zlib.decompress(row[0]))
def connect(path):
 db=sqlite3.connect(path);db.execute('PRAGMA journal_mode=OFF');db.execute('PRAGMA cache_size=-1024');db.execute('PRAGMA temp_store=FILE');return db
