"""Archive the single completed run; no native imports or new simulation."""
import gzip,hashlib,importlib.util,json,pathlib,shutil

root=pathlib.Path('/data_robot/projects/.worktrees/nero_dh116_finger22_risk_review')
parent=pathlib.Path('/data_robot/verification/nero_dh116_right/runs/issue5_functional_v1')
source=parent/'close149_B_functional_v1_once'
dest=root/'examples/libero_eval/evidence/functional_v1'
raw=dest/'raw';raw.mkdir()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
records=[]
media=[]
for p in sorted(source.rglob('*')):
 if not p.is_file():continue
 rel=p.relative_to(source)
 if rel.parts[0] in ('frames','videos'):
  media.append((p,rel));continue
 target=raw/rel;target.parent.mkdir(parents=True,exist_ok=True)
 if rel.parent==pathlib.Path('.') and p.name in ('robot-physical-preflight.json','frozen-before-first-step.json','result.json'):
  target=target.with_suffix(target.suffix+'.gz')
  with target.open('wb') as stream:
   with gzip.GzipFile(filename='',mode='wb',fileobj=stream,mtime=0) as out:out.write(p.read_bytes())
 else:shutil.copyfile(p,target)
 records.append({'source':str(p),'path':str(target.relative_to(dest)),
                 'original_sha256':sha(p),'archived_sha256':sha(target),'bytes':p.stat().st_size})
for name in ('execution.json','execution-claim.json','stdout.log','stderr.log'):
 shutil.copyfile(parent/name,raw/name)
for name,source_path in [('functional-adapter.executed.py',root/'examples/right_hand/close149_functional.py'),
                         ('functional-tests.executed.py',root/'examples/right_hand/test_close149_functional.py'),
                         ('launcher.executed.py',pathlib.Path('/tmp/nero-functional-launch.py'))]:
 shutil.copyfile(source_path,raw/name)
execution=json.loads((parent/'execution.json').read_text())
assert sha(raw/'functional-adapter.executed.py')==execution['entry_sha256']
assert sha(raw/'functional-tests.executed.py')==execution['test_sha256']

spec=importlib.util.spec_from_file_location('recorded_episode',root/'examples/libero_eval/episode.py')
episode=importlib.util.module_from_spec(spec);spec.loader.exec_module(episode)
episode.export_episode(root,'R','research',dest/'episode',raw)
# Keep each image/video once, in the existing unified episode layout.
for p,rel in media:
 target=dest/'episode'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,target)
 records.append({'source':str(p),'path':str(target.relative_to(dest)),
                 'original_sha256':sha(p),'archived_sha256':sha(target),'bytes':p.stat().st_size})
camera_path=dest/'episode/cameras.json';camera=json.loads(camera_path.read_text())
camera['videos']=[{'path':str(p.relative_to(dest/'episode')),'sha256':sha(p)} for p in sorted((dest/'episode/videos').glob('*.mp4'))]
camera_path.write_text(json.dumps(camera,indent=2)+'\n')
manifest_path=dest/'episode/manifest.json';manifest=json.loads(manifest_path.read_text())
old_identity=manifest['model'];material=json.loads((raw/'drive-materialization.json').read_text())
manifest['model']={'path':str((raw/'drive-model/model.xml').relative_to(root)),
 'sha256':sha(raw/'drive-model/model.xml'),'hand_collision_proxies':81,'wrist_collision_proxies':306,
 'profile':'R_close149_left_drive_profile_v1','base_model':old_identity,
 'raw_original_context_xml':str((raw/'drive-model/original-context.xml').relative_to(root)),
 'raw_original_context_sha256':material['raw_sha256'],'actual_runtime_path':material['resolved_model']}
manifest['measurement_protocol']='functional_v1'
manifest['post_export_metadata']={'original_exporter_model':old_identity,
 'reason':'Existing exporter has fixed A identity; bind this new B episode to the actually used XML and explicit protocol. Exporter unchanged.',
 'no_physics_or_state_changes':True,'raw_materialization_sha256':sha(raw/'drive-materialization.json')}
manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
result_path=dest/'episode/result.json';result=json.loads(result_path.read_text())
result['measurement_protocol']='functional_v1'
result['functional_diagnostics']=json.loads((raw/'functional-diagnostics.json').read_text())
result_path.write_text(json.dumps(result,indent=2)+'\n')
(raw/'artifact-source.json').write_text(json.dumps({'source_run':str(source),'artifacts':records,
 'media_stored_once_under_episode':True,'postprocessing_new_physics_steps':0},indent=2)+'\n')
for stem in ('protection','grasp'):
 shutil.copyfile('/tmp/nero-functional-'+stem+'-review.json',dest/(stem+'-review.json'))
 shutil.copyfile('/tmp/nero-functional-'+stem+'-review.py',dest/(stem+'-review.py'))
shutil.copyfile('/tmp/nero-functional-package.py',raw/'package-evidence.py')
print('Archived one 6500-step run with original state/contact bytes and single-copy dual-camera media.')
print('Archive bytes',sum(p.stat().st_size for p in dest.rglob('*') if p.is_file()))
