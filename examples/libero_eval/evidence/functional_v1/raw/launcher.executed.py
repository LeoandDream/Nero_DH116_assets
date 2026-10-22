import datetime,hashlib,json,os,pathlib,shlex,subprocess,sys

root=pathlib.Path('/data_robot/projects/.worktrees/nero_dh116_finger22_risk_review')
runtime=pathlib.Path('/data_robot/verification/nero_dh116_right')
parent=runtime/'runs/issue5_functional_v1';output=parent/'close149_B_functional_v1_once'
authorization='https://github.com/LeoandDream/Nero_DH116_assets/issues/5#issuecomment-6346434609'
stamp=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
protected=json.loads(pathlib.Path('/tmp/nero-functional-protected.json').read_text())
assert subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()==protected['baseline']
assert all((root/p).is_file() and sha(root/p)==v for p,v in protected['files'].items()),'Frozen original source changed'
assert sha(pathlib.Path('/data_robot/projects/nero_dh116_assets/README.md'))==protected['readme_sha256']
parent.mkdir()  # Exclusive one-attempt claim; existing directory is never reused.
record={'authorization':authorization,'baseline':protected['baseline'],'protocol':'functional_v1',
 'profile':'R_close149_left_drive_profile_v1','attempt_budget':1,'automatic_retries':0,
 'prepared_utc':stamp(),'launcher_pid':os.getpid(),'native_attempt_started':False,
 'entry_sha256':sha(root/'examples/right_hand/close149_functional.py'),
 'test_sha256':sha(root/'examples/right_hand/test_close149_functional.py'),
 'old_B_entry_sha256':sha(root/'examples/right_hand/close149_drive.py'),
 'old_runs_hidden':True,'only_new_parent_writable':True}
execution=parent/'execution.json'
def save():execution.write_text(json.dumps(record,indent=2)+'\n')
with (parent/'execution-claim.json').open('x') as stream:json.dump(record,stream,indent=2)
old=json.loads((root/'examples/libero_eval/evidence/left_drive_profile_v1/raw/execution.json').read_text())
old_parent=str(runtime/'runs/issue5_left_drive_v1')
argv=[str(parent) if a==old_parent else a for a in old['argv']]
command=argv[-1]
command=command.replace('examples/right_hand/close149_drive.py --output runs/issue5_left_drive_v1/close149_B_once',
 'examples/right_hand/close149_functional.py --protocol functional_v1 --output runs/issue5_functional_v1/close149_B_functional_v1_once')
assert command!=argv[-1];argv[-1]=command;record['argv']=argv;save()
probe="import sys; from pathlib import Path; sys.path.insert(0,'examples/right_hand'); import close149_functional as F; F.validate_sources(); assert F.D.C.summarize(F.D.C.planned_schedule())['physics_steps_max']==6500; assert not any(x in sys.modules for x in ('mujoco','robosuite','numpy')); assert not Path('runs/issue5_functional_v1/close149_B_functional_v1_once').exists(); print('FUNCTIONAL_V1_OVERLAY_STDLIB_PASS')"
probe_cmd=command[:command.index('python -B')]+'python -B -c '+shlex.quote(probe)
result=subprocess.run(argv[:-1]+[probe_cmd],capture_output=True,text=True)
record.update(stdlib_probe_exit_code=result.returncode,stdlib_probe_stdout=result.stdout,stdlib_probe_stderr=result.stderr);save()
if result.returncode:print(result.stdout+result.stderr,flush=True);sys.exit(result.returncode)
print(result.stdout,flush=True)
record.update(native_attempt_started=True,started_utc=stamp());save()
with (parent/'stdout.log').open('w') as stdout,(parent/'stderr.log').open('w') as stderr:
 completed=subprocess.run(argv,stdout=stdout,stderr=stderr)
record.update(actual_process_exit_code=completed.returncode,ended_utc=stamp());save()
if (output/'result.json').is_file():
 result=json.loads((output/'result.json').read_text())
 record.update({k:result.get(k) for k in ('physics_steps','execution','first_failure','native_task_result','grasp_result','control_health','longest_continuous_valid_hold_s','lift_from_own_settled_m','functional_v1')});save()
print(json.dumps({k:v for k,v in record.items() if k!='argv'},indent=2),flush=True)
print((parent/'stdout.log').read_text()[-1500:],flush=True)
print((parent/'stderr.log').read_text()[-2000:],flush=True)
sys.exit(completed.returncode)
