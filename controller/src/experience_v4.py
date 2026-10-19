"""Development-only positive priors; permutations of fresh inputs, never verdicts.

Order is conditioned on the measured INITIAL scene, not a claimed dependency.
Later r21 depths have no pre-contact context: their scope is explicitly geometry
and configuration only. Promote at most six, alternating with baseline choices.
CandidateSearch retains its bounded visits, cursors and fair deferral unchanged.
"""
import json
from pathlib import Path
import numpy as np
from experience import Experience, policy_hash
from provenance import digest, file_hash, source_files

SCHEMA = 'roundtrip-experience/4'


def validate_provenance(data):
    """Check content links, stage certificates and the closed source registry.

    Hashes detect accidental edits, not signatures. The parent-supplied r21
    independent certificate remains the trust root; frozen bytes are auditable.
    """
    def require(ok):
        if not ok:raise ValueError('v4_invalid_provenance')
    def event(link):
        require(link['event_sha256'] == digest(link['event']))
        require(type(link['selected_event_index']) is int and link['selected_event_index'] >= 0)
        return link['event']
    sources=data['sources'];require(len(sources)==1 and sources[0]['source_id']=='ten-r21')
    source=sources[0]
    require(source.get('split')=='development' and source.get('accepted') is True)
    require(source.get('scope')=='independently_verified_extraction_only')
    require(bool(source.get('files')) and bool(source.get('shared_algorithm_sha256')))
    require(len(source['raw_events_sha256'])==64 and len(source['raw_mesh_sha256'])==64)
    order=data['order']; records=data['records']
    require(len(records)==10 and len({r['object_id'] for r in records})==10)
    initial=event(order['provenance']['initial'])
    require(initial['kind']=='initial_state' and initial['state']==order['initial_state'])
    for r in records:
        p=r['provenance'];oid=r['object_id'];obj=p['object_verification']
        require(p['raw_events_sha256']==source['raw_events_sha256'])
        require(str(obj['object_id'])==oid and all(obj[k] is True for k in ('passed','lifted','attached','released')))
        candidate=event(p['candidate']);committed=event(p['committed']);verified=event(p['verified'])
        require(candidate['kind']=='candidate_preflight' and str(candidate['object_id'])==oid)
        require(candidate['candidate']['surface_depth_m']==r['depth_m'] and 0 < r['depth_m'] <= .06)
        require(committed['kind']=='released' and str(committed['object_id'])==oid and committed['settled'] is True)
        require(verified['kind']=='grasp_candidate_verified' and str(verified['object_id'])==oid
                and verified['independent_trials']==2 and verified['index']==candidate['index'])
        previews=[event(link) for link in p['previews']]
        require(len(previews)==2 and {e['trial'] for e in previews}=={1,2})
        require(all(e['kind']=='grasp_preview' and e['accepted'] is True and str(e['object_id'])==oid
                    and e['index']==candidate['index'] for e in previews))
        indices=[p['candidate']['selected_event_index']]+[v['selected_event_index'] for v in p['previews']]+[
            p['verified']['selected_event_index'],p['committed']['selected_event_index']]
        require(all(a<b for a,b in zip(indices,indices[1:])))
        require(r['geometry_sha256']==p['current_identity']['geometry_sha256']==order['geometry_sha256'][oid])
        require(r['context_scope'] in ('geometry_configuration_only_no_pregrasp_state','measured_pre_action_initial_poses'))
        if r['context_scope']=='measured_pre_action_initial_poses':
            require(oid==order['object_ids'][0] and r['pre_action_state']==initial['state'])
    ordered=sorted(records,key=lambda r:r['provenance']['committed']['selected_event_index'])
    require(order['object_ids']==[r['object_id'] for r in ordered])
    require(order['provenance']['committed']==[r['provenance']['committed'] for r in ordered])
    require(not data['negative_hints'] and not data['return_hints'])


def implementation_hash():
    return digest(source_files(Path(__file__).parent))


def poses_match(actual, expected):
    if set(actual['objects']) != set(expected['objects']):
        return False
    for oid, obj in expected['objects'].items():
        a = np.asarray(actual['objects'][oid]['pose'], float)
        b = np.asarray(obj['pose'], float)
        if a.shape != (7,) or b.shape != (7,) or not np.isfinite([a, b]).all():
            return False
        if np.linalg.norm(a[:3] - b[:3]) > .025:
            return False
        norms = np.linalg.norm(a[3:]) * np.linalg.norm(b[3:])
        if norms == 0 or abs(float(a[3:] @ b[3:])) / norms < np.cos(np.deg2rad(10) / 2):
            return False
    return True


def load_experience(path, config):
    if path is not None and json.loads(Path(path).read_text()).get('schema') == SCHEMA:
        return DevelopmentExperience(Path(path), config)
    return Experience(path, config.experience)


class DevelopmentExperience(Experience):
    def __init__(self, path, config):
        super().__init__(enabled=False)
        data = json.loads(path.read_text())
        if config.split != 'development' or data.get('split') != 'development':
            raise ValueError('v4_heldout_not_declared')
        if data.get('schema') != SCHEMA or data.get('policy_sha256') != digest({k:v for k,v in data.items() if k != 'policy_sha256'}):
            raise ValueError('v4_policy_integrity')
        if data['implementation_sha256'] != implementation_hash() or data['configuration_sha256'] != policy_hash(config):
            raise ValueError('v4_implementation_or_configuration_mismatch')
        if not data.get('records') or not data.get('sources'):
            raise ValueError('v4_missing_provenance')
        source_ids = {s['source_id'] for s in data['sources'] if s.get('split') == 'development' and s.get('accepted') is True}
        if len(source_ids) != len(data['sources']):
            raise ValueError('v4_unverified_or_heldout_source')
        for record in [data['order']] + data['records']:
            if (record.get('split') != 'development' or record.get('stage') != 'extraction'
                    or record.get('outcome') != 'success' or not record.get('provenance')
                    or record['provenance'].get('source_id') not in source_ids
                    or record.get('record_id') != digest({k:v for k,v in record.items() if k != 'record_id'})):
                raise ValueError('v4_unverified_record')
        try:
            validate_provenance(data)
        except (KeyError, TypeError, IndexError) as exc:
            raise ValueError('v4_invalid_provenance') from exc
        self.enabled = config.experience
        self.snapshot_sha256 = file_hash(path)
        self.records = tuple(data['records'])
        self.order = data['order']
        self.configuration_hash = data['configuration_sha256']
        self.last_object_trace = {}

    def rank_objects(self, targets, initial, geometry, stage='extraction'):
        self.last_object_trace = dict(enabled=self.enabled, stage=stage, object_count=len(targets),
            matched_hints=0, used_hints=0, reordered=False, fallback='disabled', scope='compatible_initial_scene_order_prior')
        if not self.enabled:
            return targets
        r = self.order
        reason = ('stage_mismatch' if stage != 'extraction' else
                  'geometry_mismatch' if geometry != r['geometry_sha256'] else
                  'initial_pose_mismatch' if not poses_match(initial, r['initial_state']) else None)
        if reason:
            self.last_object_trace['fallback'] = reason
            return targets
        positions = {oid:i for i,oid in enumerate(r['object_ids'])}
        ranked = sorted(targets, key=lambda oid:positions.get(str(oid), len(positions)))
        self.last_object_trace.update(matched_hints=1, used_hints=1, reordered=ranked != targets,
            fallback=None, source_record_ids=[r['record_id']], before=list(targets), after=ranked)
        return ranked

    def rank(self, candidates, geometry_hash, configuration_hash, scene_descriptor, stage=None,
             object_id=None, current_state=None):
        self.rank_calls += 1
        self.last_trace = dict(enabled=self.enabled, stage=stage, candidate_count=len(candidates),
            matched_hints=0, used_hints=0, reordered=False, fallback='disabled',
            promoted_candidates=0, predicted_first_depth_rank_delta=None)
        if not self.enabled:
            return candidates
        matched = [r for r in self.records if stage == r['stage'] == 'extraction'
            and str(object_id) == r['object_id'] and geometry_hash == r['geometry_sha256']
            and configuration_hash == self.configuration_hash
            and (r['context_scope'] == 'geometry_configuration_only_no_pregrasp_state'
                 or (current_state is not None and poses_match(current_state, r['pre_action_state'])))]
        preferred = [i for i,c in enumerate(candidates)
                     if any(abs(float(c['surface_depth_m']) - r['depth_m']) < 1e-9 for r in matched)]
        # A fixed bounded prefix plus baseline exploration; never duplicate/drop.
        order = []; seen = set(); baseline = iter(range(len(candidates)))
        for i in preferred[:6]:
            if i not in seen:
                order.append(i); seen.add(i)
            j = next((j for j in baseline if j not in seen), None)
            if j is not None:
                order.append(j); seen.add(j)
        order.extend(i for i in range(len(candidates)) if i not in seen)
        ranked = [candidates[i] for i in order]
        changed = order != list(range(len(candidates)))
        used = sum(any(abs(float(c['surface_depth_m'])-r['depth_m']) < 1e-9 for c in candidates) for r in matched)
        self.matches += len(matched); self.used_hints += used; self.reordered_calls += int(changed)
        self.last_trace.update(matched_hints=len(matched), used_hints=used, reordered=changed,
            fallback='no_compatible_hint' if not matched else 'no_fresh_depth_match' if not preferred else None,
            promoted_candidates=sum(order.index(i) < i for i in preferred[:6]),
            predicted_first_depth_rank_delta=preferred[0] if preferred else None,
            source_record_ids=[r['record_id'] for r in matched], context_scopes=[r['context_scope'] for r in matched],
            exploration='at most six preferred candidates interleaved with baseline; full baseline tail retained')
        return ranked
