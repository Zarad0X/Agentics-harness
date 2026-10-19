"""Physical scene preflight evidence. Layout validity grants no task success."""
import argparse
import json
from pathlib import Path
from provenance import write_once, file_hash, digest
from run_config import RunConfig
from scene_pipeline import validate_layout


def report(status, **values):
    return dict(schema='roundtrip-layout/1',mode='layout',layout_status=status,
                layout_valid=status=='valid',accepted=False,whole_episode_success=False,
                extraction_rate=None,exact_pose_return_rate=None,**values)


def initialization_failure(out, config, exc):
    message=str(exc)
    status='invalid_spawn' if message.startswith(('packing_height_budget_exhausted','asset_does_not_fit_container')) else 'initialization_error'
    write_once(out/'layout-report.json',report(status,config=config.to_dict(),errors=[type(exc).__name__+':'+message]))


def finish_layout(out, manifest, rgb, elapsed):
    import imageio.v2 as imageio
    imageio.imwrite(out/'initial-scene.png',rgb)
    validation=manifest['layout_validation']
    value=report('valid' if validation['valid'] else 'invalid_layout',
        scene_count=len(manifest['objects']),errors=validation['errors'],elapsed_s=elapsed,
        scene_manifest_sha256=file_hash(out/'scene-manifest.json'),
        visual_sha256=file_hash(out/'initial-scene.png'),
        density=validation['footprint_aabb_sum_over_container_area'],
        initialization_physics_steps=None,settling_validation_steps=2500,
        scene_instances=1,trajectory_written=False,
        claim='Settled layout only; no extraction, return, or whole-episode evidence.')
    write_once(out/'layout-report.json',value)
    return 0 if value['layout_valid'] else 2


def audit(out, source_root):
    out=Path(out);r=json.loads((out/'layout-report.json').read_text())
    if r['layout_status'] in ('invalid_spawn','initialization_error'):
        return report(r['layout_status'],errors=r['errors'])
    m=json.loads((out/'scene-manifest.json').read_text());cfg=RunConfig(**m['config'])
    meshes=json.loads((out/'physics-meshes.json').read_text());errors=[]
    if cfg.mode!='layout' or m['selected_ids']!=[]:errors.append('scope')
    if RunConfig(**json.loads((out/'effective-config.json').read_text())).to_dict()!=cfg.to_dict():errors.append('config_changed')
    if file_hash(out/'scene-manifest.json')!=r['scene_manifest_sha256']:errors.append('manifest_hash')
    if file_hash(out/'physics-meshes.json')!=m['physics_mesh_sha256']:errors.append('mesh_hash')
    if file_hash(out/'initial-scene.png')!=r['visual_sha256']:errors.append('visual_hash')
    if set(meshes)!=set(m['objects']):errors.append('mesh_ids')
    for oid,o in m['objects'].items():
        if digest(meshes[oid])!=o['geometry_sha256']:errors.append('object_geometry:'+oid)
    source_root=Path(source_root).resolve()
    for name,h in m['source_sha256'].items():
        p=(source_root/name).resolve()
        if not p.is_relative_to(source_root) or not p.is_file() or file_hash(p)!=h:errors.append('source_changed:'+name)
    measured=validate_layout(dict(objects=m['objects']),m['earlier_poses'],meshes,m['initial_contacts'],cfg)
    if measured!=m['layout_validation'] or measured!=json.loads((out/'layout-validation.json').read_text()):errors.append('validation_mismatch')
    errors+=measured['errors']
    return report('invalid_evidence' if errors and measured['valid'] else ('valid' if not errors else 'invalid_layout'),
                  errors=errors,scene_count=cfg.count,scene_manifest_sha256=file_hash(out/'scene-manifest.json'),
                  elapsed_s=r['elapsed_s'],density=r['density'])


def main():
    ap=argparse.ArgumentParser();ap.add_argument('run',type=Path);ap.add_argument('--source-root',type=Path,required=True);a=ap.parse_args()
    try:r=audit(a.run,a.source_root)
    except Exception as exc:r=report('invalid_evidence',errors=[type(exc).__name__+':'+str(exc)])
    write_once(a.run/'independent-verification.json',r);print(json.dumps(r))
    return 0 if r['layout_valid'] else 2

if __name__=='__main__':raise SystemExit(main())
