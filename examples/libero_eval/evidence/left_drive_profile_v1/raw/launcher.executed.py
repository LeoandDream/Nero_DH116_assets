import datetime,hashlib,json,os,pathlib,shlex,subprocess,sys

root=pathlib.Path('/data_robot/projects/.worktrees/nero_dh116_finger22_risk_review')
runtime=pathlib.Path('/data_robot/verification/nero_dh116_right')
parent=runtime/'runs/issue5_left_drive_v1'
output=parent/'close149_B_once'
authorization='https://github.com/LeoandDream/Nero_DH116_assets/issues/5#issuecomment-6297636306'
stamp=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
protected=json.loads(pathlib.Path('/tmp/nero-drive-B-protected.json').read_text())
assert subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()==protected['baseline']
assert all((root/p).is_file() and sha(root/p)==v for p,v in protected['files'].items()),'Existing tracked file changed'
assert sha(pathlib.Path('/data_robot/projects/nero_dh116_assets/README.md'))==protected['readme_sha256']
# Fresh exclusive parent is the one-attempt lock. Never recover/reuse it here.
parent.mkdir()
record={'authorization':authorization,'baseline':protected['baseline'],
 'profile':'R_close149_left_drive_profile_v1','attempt_budget':1,'automatic_retries':0,
 'prepared_utc':stamp(),'launcher_pid':os.getpid(),'native_attempt_started':False,
 'entry_sha256':sha(root/'examples/right_hand/close149_drive.py'),
 'test_sha256':sha(root/'examples/right_hand/test_close149_drive.py'),
 'only_new_parent_writable':True,'old_runs_hidden':True}
execution=parent/'execution.json'
def save():execution.write_text(json.dumps(record,indent=2)+'\n')
with (parent/'execution-claim.json').open('x') as stream:json.dump(record,stream,indent=2)
command=('set -e; source examples/libero_reference/env/activate.sh; '
 'export MPLCONFIGDIR=/tmp/mpl NUMBA_CACHE_DIR=/tmp/numba XDG_CACHE_HOME=/tmp/xdg TMPDIR=/tmp '
 'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1; '
 'python -B examples/right_hand/close149_drive.py --output runs/issue5_left_drive_v1/close149_B_once')
argv=['bwrap','--ro-bind','/','/','--ro-bind',str(root),str(runtime),
 '--ro-bind',str(runtime/'.reference-env'),str(runtime/'.reference-env'),
 '--ro-bind',str(runtime/'.reference-deps'),str(runtime/'.reference-deps'),
 '--tmpfs',str(runtime/'runs'),'--bind',str(parent),str(parent),
 '--tmpfs','/data_robot/projects','--tmpfs','/tmp','--dev-bind','/dev','/dev','--proc','/proc',
 '--chdir',str(runtime),'/bin/bash','-c',command]
record['argv']=argv;save()
probe="import sys; from pathlib import Path; sys.path.insert(0,'examples/right_hand'); import close149_drive as B; B.load_profile(); assert B.C.canonical_sha(B.C.planned_schedule())=='ebf188b0bf49ad3b74a042c52b5fa4ad5df02dcb8f84caea565fcdaf9fdc7e32'; assert not any(x in sys.modules for x in ('mujoco','robosuite','numpy')); assert not Path('runs/issue5_left_drive_v1/close149_B_once').exists(); print('B_OVERLAY_STDLIB_PREFLIGHT_PASS')"
probe_command=command[:command.index('python -B')]+ 'python -B -c '+shlex.quote(probe)
probe_result=subprocess.run(argv[:-1]+[probe_command],capture_output=True,text=True)
record.update(stdlib_probe_exit_code=probe_result.returncode,stdlib_probe_stdout=probe_result.stdout,stdlib_probe_stderr=probe_result.stderr)
save()
if probe_result.returncode:
 print(probe_result.stdout+probe_result.stderr,flush=True);sys.exit(probe_result.returncode)
print(probe_result.stdout,flush=True)
record.update(native_attempt_started=True,started_utc=stamp());save()
with (parent/'stdout.log').open('w') as stdout,(parent/'stderr.log').open('w') as stderr:
 completed=subprocess.run(argv,stdout=stdout,stderr=stderr)
record.update(actual_process_exit_code=completed.returncode,ended_utc=stamp());save()
if (output/'result.json').is_file():
 result=json.loads((output/'result.json').read_text())
 record.update({k:result.get(k) for k in ('physics_steps','execution','first_failure','native_task_result','grasp_result','control_health','longest_continuous_valid_hold_s','lift_from_own_settled_m')});save()
print(json.dumps(record,indent=2),flush=True)
print((parent/'stdout.log').read_text()[-2500:],flush=True)
print((parent/'stderr.log').read_text()[-3500:],flush=True)
sys.exit(completed.returncode)
