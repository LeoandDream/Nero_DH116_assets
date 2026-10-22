import gzip,hashlib,importlib.util,json,pathlib,shutil,subprocess

root=pathlib.Path('/data_robot/projects/.worktrees/nero_dh116_finger22_risk_review')
parent=pathlib.Path('/data_robot/verification/nero_dh116_right/runs/issue5_left_drive_v1')
source=parent/'close149_B_once'
dest=root/'examples/libero_eval/evidence/left_drive_profile_v1'
raw=dest/'raw';raw.mkdir()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
records=[]
for p in sorted(source.rglob('*')):
 if not p.is_file():continue
 rel=p.relative_to(source);target=raw/rel;target.parent.mkdir(parents=True,exist_ok=True)
 if rel.parent==pathlib.Path('.') and p.name in ('robot-physical-preflight.json','frozen-before-first-step.json','result.json'):
  target=target.with_suffix(target.suffix+'.gz')
  with target.open('wb') as stream:
   with gzip.GzipFile(filename='',mode='wb',fileobj=stream,mtime=0) as out:out.write(p.read_bytes())
 else:shutil.copyfile(p,target)
 records.append({'source':str(p),'path':str(target.relative_to(dest)), 'original_sha256':sha(p),'archived_sha256':sha(target),'bytes':p.stat().st_size})
for name in ('execution.json','execution-claim.json','stdout.log','stderr.log'):
 shutil.copyfile(parent/name,raw/name)
for name,source_path in [('drive-adapter.executed.py',root/'examples/right_hand/close149_drive.py'),('drive-adapter-tests.executed.py',root/'examples/right_hand/test_close149_drive.py'),('launcher.executed.py',pathlib.Path('/tmp/nero-drive-B-launch.py'))]:
 shutil.copyfile(source_path,raw/name)
assert sha(raw/'drive-adapter.executed.py')==json.loads((parent/'execution.json').read_text())['entry_sha256']
(raw/'artifact-source.json').write_text(json.dumps({'source_run':str(source),'artifacts':records,'postprocessing_new_physics_steps':0},indent=2)+'\n')

# Reuse the existing pure recorded-episode exporter. Correct only this new
# manifest's formerly hard-coded model identity; do not change the exporter.
spec=importlib.util.spec_from_file_location('recorded_episode',root/'examples/libero_eval/episode.py')
episode=importlib.util.module_from_spec(spec);spec.loader.exec_module(episode)
episode.export_episode(root,'R','research',dest/'episode',raw)
manifest_path=dest/'episode/manifest.json';manifest=json.loads(manifest_path.read_text())
old_identity=manifest['model']
material=json.loads((raw/'drive-materialization.json').read_text())
manifest['model']={'path':str((raw/'drive-model/model.xml').relative_to(root)),
 'sha256':sha(raw/'drive-model/model.xml'),'hand_collision_proxies':81,'wrist_collision_proxies':306,
 'profile':'R_close149_left_drive_profile_v1','base_model':old_identity,
 'raw_original_context_xml':str((raw/'drive-model/original-context.xml').relative_to(root)),
 'raw_original_context_sha256':material['raw_sha256'],
 'actual_runtime_path':material['resolved_model'],
 'frozen_profile':'examples/right_hand/candidates/left_drive_profile_v1/profile.json'}
manifest['post_export_model_identity_correction']={
 'reason':'Unchanged generic exporter assumes right base model A; explicitly bind this B episode to its actual archived B XML and source profile.',
 'original_exporter_model':old_identity,'raw_materialization_sha256':sha(raw/'drive-materialization.json'),
 'physics_or_state_changes':False}
manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')

# Preserve the final actually executed offline tests and code hashes.
logs=[]
for name in ('test_close149_drive.py','test_close149.py'):
 r=subprocess.run(['python3','-B',str(root/'examples/right_hand'/name)],capture_output=True,text=True)
 logs.append(r.stdout+r.stderr);assert r.returncode==0
(dest/'tests.log').write_text('\n'.join(logs))
print('Archived raw evidence and exported 245 saved steps; no engine imported or simulation run.')
