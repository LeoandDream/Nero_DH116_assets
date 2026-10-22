import sys, importlib.abc
class Block(importlib.abc.MetaPathFinder):
 def find_spec(self, fullname, path=None, target=None):
  if fullname.split('.')[0] in {'mujoco','robosuite','libero','numpy','scipy','PIL'}: raise RuntimeError('Forbidden native import: '+fullname)
sys.meta_path.insert(0,Block())
import gzip,json,math,itertools,hashlib
from pathlib import Path
from collections import Counter
ROOT=Path('/data_robot/projects/.worktrees/nero_dh116_finger22_risk_review')
sys.path.insert(0,str(ROOT/'examples/right_hand'))
import libero_research as R
RUN=Path('/data_robot/verification/nero_dh116_right/runs/issue5_functional_v1/close149_B_functional_v1_once')
OUT=Path('/tmp/nero-functional-protection-review.json')
def read(name):return json.loads((RUN/name).read_text())
def rows(name):
 with gzip.open(RUN/name,'rt') as h:
  for line in h:yield json.loads(line)
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for chunk in iter(lambda:f.read(1<<20),b''):h.update(chunk)
 return h.hexdigest()
def stat(table, reasons, step):
 for name in set(reasons):
  item=table.setdefault(name,{'first_step':step,'last_step':step,'steps':0})
  item['last_step']=step;item['steps']+=1
frozen=read('frozen-before-first-step.json');criteria=frozen['criteria'];protocol=read('registered-protocol.json')
limits=[j['fields']['jnt_range'] for j in read('robot-physical-preflight.json')['actual_snapshot']['objects']['JOINT']]
rules,_=R.load_contact_policy();policy=R.ResearchPolicy(rules)
DIAG=('OUTSIDE_REGISTERED_PATH_ENVELOPE','ACTIVE_HOLD_TRACKING')
maxima={'arm_tracking_rad':0.,'active_tracking_rad':0.,'active_hold_tracking_rad':0.,'mimic_rad':0.,'joint_overshoot_rad':0.,'robot_overlap_m':0.,'bowl_lift_m':0.}
diagnostic_stats={};old_research_stats={};formal_stats={};hard_stats={};warn_stats={};event_stats={}
checks=Counter();last_time=read('drive-native-reset-cache.json')['time_s'];steps=0;peak_normal=0.;peak_normal_record=None;max_force_group=0.;max_force_group_record=None
for s,c in itertools.zip_longest(rows('state.jsonl.gz'),rows('contacts.jsonl.gz')):
 assert s is not None and c is not None,'unequal stream lengths'
 steps+=1;assert s['step']==c['step']==steps
 q=s['robot_state_18'];a=s['action_13'];target=s['target_qpos_18'];contacts=c['contacts']
 assert len(q)==18 and len(a)==13 and len(target)==18
 arm=[abs(q[i]-a[i]) for i in range(7)];active=[abs(q[i]-target[i]) for i in (7,8,10,12,14,16)]
 expected=[.004592+.8808*q[8]]+[.011072+.9206*q[i] for i in (10,12,14,16)]
 mimic=max(abs(q[i]-v) for i,v in zip((9,11,13,15,17),expected))
 overshoot=max([0.]+[v for x,(lo,hi) in zip(q,limits) for v in (lo-x,x-hi)])
 eligible=[x for x in contacts if not x['filter_reasons'] and any(b.startswith(('robot0_','gripper0_','mount0_')) for b in x['body'])]
 overlap=max([0.]+[-x['distance_m'] for x in eligible])
 finite=all(all(isinstance(x,(int,float)) and math.isfinite(x) for x in s[k]) for k in ('full_qpos','full_qvel','full_qacc_cached','ctrl','actuator_force'))
 assert arm==s['arm_error_rad'] and active==s['active_error_rad'] and mimic==s['mimic_rad'] and overshoot==s['joint_overshoot_rad'] and overlap==s['robot_overlap_m'] and finite==s['finite'],('health metric mismatch',steps)
 independent=[]
 if not finite:independent.append('NONFINITE_STATE')
 if s['warnings'] or not math.isclose(s['time_s']-last_time,.002,rel_tol=0,abs_tol=1e-12):independent.append('NUMERICAL_WARNING_OR_TIME_RESET')
 if max(arm)>criteria['arm_tracking_error_rad_max']:independent.append('ARM_TRACKING')
 if s['phase']=='hold' and max(active)>criteria['steady_active_error_rad_max']:independent.append('ACTIVE_HOLD_TRACKING')
 if mimic>criteria['linear_mimic_error_rad_max']:independent.append('MIMIC')
 if overshoot>criteria['joint_limit_overshoot_rad_max']:independent.append('JOINT_LIMIT')
 wrist=sum(any((n or '').startswith('robot0_wrist_reuse_') for n in x['geom']) and not set(x['body']).issubset({'robot0_link5','robot0_gripper_flange'}) and not x['filter_reasons'] for x in contacts)
 assert wrist==s['wrist_external_contact_count']
 if wrist:independent.append('UNEXPECTED_WRIST_EXTERNAL_CONTACT')
 if any(x<lo or x>hi for x,lo,hi in zip(q,protocol['qpos_envelope_lower_rad'],protocol['qpos_envelope_upper_rad'])):independent.append('OUTSIDE_REGISTERED_PATH_ENVELOPE')
 old=policy.evaluate(steps,s['phase'],contacts,independent)
 saved=s['research'];expected_diag=[x for x in old['research_failures'] if x in DIAG];expected_hard=[x for x in old['research_failures'] if x not in DIAG]
 assert old['legacy_failures']==s['legacy_failures']==saved['legacy_failures'],('formal mismatch',steps)
 assert old['research_failures']==saved['old_research_failures']==s['functional_v1']['old_research_failures'],('old B mismatch',steps)
 assert expected_hard==s['failures']==saved['research_failures'],('hard mismatch',steps)
 assert expected_diag==saved['diagnostics']==s['functional_v1']['diagnostics'],('diagnostic mismatch',steps)
 for field in ('warnings','groups','contact_events'):assert old[field]==saved[field],(field,'mismatch',steps)
 assert s['functional_v1']['continued_despite_diagnostics']==bool(expected_diag and not expected_hard)
 stat(diagnostic_stats,expected_diag,steps);stat(old_research_stats,old['research_failures'],steps);stat(formal_stats,old['legacy_failures'],steps);stat(hard_stats,expected_hard,steps)
 stat(warn_stats,[x['reason'] for x in old['warnings']],steps);stat(event_stats,[x['reason'] for x in old['contact_events']],steps)
 vals=(max(arm),max(active),max(active) if s['phase']=='hold' else 0.,mimic,overshoot,overlap,s['lift_from_own_settled_m'] or 0.)
 for k,v in zip(maxima,vals):maxima[k]=max(maxima[k],v)
 for x in contacts:
  if x['force_contact_frame'][0]>peak_normal:
   peak_normal=x['force_contact_frame'][0];peak_normal_record={'step':steps,'geom':x['geom'],'body':x['body'],'normal_force_N':peak_normal}
 for group in old['groups']:
  if group['normal_force_N']>max_force_group:
   max_force_group=group['normal_force_N'];max_force_group_record={'step':steps,'level':group['level'],'key':group['key'],'normal_force_N':max_force_group}
 checks.update({'raw_health_metrics':1,'old_formal_decision':1,'old_B_research_decision':1,'functional_decision':1,'diagnostic_decision':1,'full_V1_groups_warnings_events_exact':1})
 last_time=s['time_s']
result=read('result.json')
assert steps==6500 and maxima==result['maxima'],('steps/maxima mismatch',steps)
assert not hard_stats,'unexpected true hard stop'
assert diagnostic_stats['OUTSIDE_REGISTERED_PATH_ENVELOPE']['first_step']==245
assert diagnostic_stats['ACTIVE_HOLD_TRACKING']['first_step']==5751
assert not event_stats and 'ROBOT_CONTACT_OVERLAP' not in formal_stats
assert result['dynamic_attempts']==1 and result['completed_commands']==260 and result['execution']=='FINISHED'
assert result['native_task_result'] is False
report={'status':'PASS_OFFLINE_PROTECTION_RECOMPUTATION','runtime':str(RUN),'method':'Pure standard library; streaming paired saved state/contact lines; fresh frozen ResearchPolicy and ContactTracker evaluated once per saved physical step; no native engine, compile, Reset, forward or integration.',
 'steps_checked':steps,'checks':dict(checks),'removed_from_actual_hard_stop_only':list(DIAG),'diagnostic_stats':diagnostic_stats,'old_B_research_failures':old_research_stats,'formal_legacy_failures':formal_stats,'actual_hard_failures':hard_stats,'contact_warning_stats':warn_stats,'contact_stop_stats':event_stats,
 'old_2mm_first_trigger':None,'old_2mm_triggered':False,'maxima_recomputed_exact':maxima,'all_result_maxima_exact':True,'peak_individual_contact_normal_force':peak_normal_record,'peak_scoped_aggregated_contact_normal_force':max_force_group_record,
 'contact_tracker_history':'One sequential tracker retained across all phase transitions; all serialized groups, warnings and stop events match original policy exactly.',
 'thresholds_unchanged':criteria,'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'examples/right_hand/libero_research.py',ROOT/'examples/right_hand/evidence/d3_research_contact_policy/shadow.py',ROOT/'examples/right_hand/evidence/d3_research_contact_policy/research-policy.json')},
 'raw_sha256':{name:sha(RUN/name) for name in ('state.jsonl.gz','contacts.jsonl.gz','result.json','registered-protocol.json')},'blocked_native_modules_present':[m for m in sys.modules if m.split('.')[0] in {'mujoco','robosuite','libero','numpy','scipy','PIL'}],
 'limits':'Qualification fields are the original saved RK4 contact qualification; no collision geometry or model compilation performed. Post-step qpos/health vs final RK4-stage contacts/forces retain documented timing difference.'}
assert not report['blocked_native_modules_present']
OUT.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:report[k] for k in ('status','steps_checked','diagnostic_stats','actual_hard_failures','old_2mm_first_trigger','maxima_recomputed_exact','contact_warning_stats','contact_stop_stats')},indent=2))
