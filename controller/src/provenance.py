"""Small explicit manifests only: never scan home directories or credentials."""
import runtime_profile as timing
import hashlib
import json
import os
from pathlib import Path


@timing.measured('canonical')
def canonical(value): return (json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()
@timing.measured('digest')
def digest(value): return hashlib.sha256(canonical(value)).hexdigest()
def file_hash(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_once(path,value):
    path=Path(path)
    with path.open('xb') as f:
        f.write(canonical(value));f.flush();os.fsync(f.fileno())
    path.chmod(0o444)
    return file_hash(path)


def source_files(root):
    root=Path(root)
    files=sorted(root.glob('*.py'))+sorted((root/'fixtures').glob('*'))
    return {str(p.relative_to(root)):file_hash(p) for p in files if p.is_file()}


class PhaseLedger:
    """All-selected-extracted barrier. Smoke is always partial scene coverage."""
    def __init__(self,all_ids,selected):
        self.all_ids=set(map(str,all_ids));self.selected=set(map(str,selected))
        if not self.selected or not self.selected<=self.all_ids: raise ValueError('invalid selected IDs')
        self.extracted=set();self.returned=set();self.phase='extraction'

    def extract(self,oid):
        oid=str(oid)
        if self.phase!='extraction' or oid not in self.selected or oid in self.extracted: raise RuntimeError('invalid extraction')
        self.extracted.add(oid)

    def begin_return(self):
        if self.extracted!=self.selected: raise RuntimeError('all objects must first be extracted')
        self.phase='return'

    def put_back(self,oid):
        oid=str(oid)
        if self.phase!='return' or oid not in self.extracted or oid in self.returned: raise RuntimeError('invalid return')
        self.returned.add(oid)


LEGACY_SCENE_SCHEMA = 'measured-scene/1-contact-list'
SCENE_SCHEMA = 'measured-scene/2-contact-multiset'


def contact_multiset(contacts):
    """Sort full exact JSON records; preserve duplicates and all nested order/values."""
    return sorted(canonical(record).decode('utf-8') for record in contacts)


def legacy_measured_scene_hash(manifest):
    return digest({k:manifest[k] for k in ('objects','robots','initial_contacts','container','physics_mesh_sha256')})


def measured_scene_hash(manifest):
    fields={k:manifest[k] for k in ('objects','robots','container','physics_mesh_sha256')}
    fields['initial_contacts']=contact_multiset(manifest['initial_contacts'])
    return digest(dict(schema=SCENE_SCHEMA,fields=fields))


def verify_selected_scene(manifest, expected):
    if expected.get('fingerprint_schema') != SCENE_SCHEMA:
        raise ValueError('unknown_or_legacy_fingerprint_schema')
    if measured_scene_hash(manifest) != expected['scene_sha256']:
        raise RuntimeError('measured_layout_does_not_match_frozen_selection')
