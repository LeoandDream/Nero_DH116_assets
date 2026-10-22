import gzip, hashlib, json, math
from itertools import zip_longest
from pathlib import Path
RUN=Path('/data_robot/verification/nero_dh116_right/runs/issue5_functional_v1/close149_B_functional_v1_once')
OLD=Path('/data_robot/projects/.worktrees/nero_dh116_finger22_risk_review/examples/libero_eval/evidence/left_drive_profile_v1/raw')
OUT=Path('/tmp/nero-functional-grasp-review.json')
def read(p):
 with gzip.open(p,'rt') if p.suffix=='.gz' else p.open() as f:return json.load(f)
def stream(p):
 with gzip.open(p,'rt') as f:
  for x in f:yield json.loads(x)
def stable(v):return json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False)
def diff(a,b,path='$'):
 if stable(a)==stable(b):return None
 if isinstance(a,dict) and isinstance(b,dict) and set(a)==set(b):
  for k in a:
   d=diff(a[k],b[k],path+'.'+k)
   if d:return d
 elif isinstance(a,list) and isinstance(b,list) and len(a)==len(b):
  for i,(x,y) in enumerate(zip(a,b)):
   d=diff(x,y,path+'['+str(i)+']')
   if d:return d
 return {'path':path,'old':a,'new':b}
base=None
for s in stream(RUN/'state.jsonl.gz'):
 if s['step']==250:base=s['bowl_position_m'][2];break
assert base is not None
old_states=stream(OLD/'state.jsonl.gz');old_contacts=stream(OLD/'contacts.jsonl.gz')
first={};best=consecutive=0;first_hold=None;holdsteps=0;bad_hold=[];prefix=[];state_disagreements=[]
maxlift=-math.inf;maxliftstep=None;minhold=math.inf;maxhold=-math.inf
nonhand_intervals=[];current_interval=None;unsupported_after_table_intervals=[];current_unsupported=None
hand_force_max=0.;minholdthumb=math.inf;minholdindex=math.inf
seen_table=False;seen_hand=False;hard=[];warnings=[];phase_ranges={};rowcount=0
for s,c in zip_longest(stream(RUN/'state.jsonl.gz'),stream(RUN/'contacts.jsonl.gz')):
 assert s is not None and c is not None and s['step']==c['step']
 step=s['step'];rowcount+=1;assert step==rowcount
 phase=s['phase'];phase_ranges.setdefault(phase,{'first':step,'last':step,'count':0});phase_ranges[phase]['last']=step;phase_ranges[phase]['count']+=1
 if step<=245:
  os=next(old_states);oc=next(old_contacts)
  keys=[k for k in os if k not in ('failures','legacy_failures','research')]
  new={k:s[k] for k in keys if k in s};old={k:os[k] for k in keys}
  d=diff(old,new,'state') or diff(oc,c,'contacts')
  if d:prefix.append({'step':step,**d})
 support=[];thumb=[];index=[];hand=[];table=[]
 for t in c['contacts']:
  if 'white_bowl_1_main' not in t['body'] or t['force_contact_frame'][0]<=0:continue
  other=t['body'][1] if t['body'][0]=='white_bowl_1_main' else t['body'][0]
  if not other.startswith('gripper0_'):support.append(t)
  else:hand.append(t)
  if 'table' in other.lower():table.append(t)
  radial=[t['position_world_m'][i]-s['bowl_position_m'][i] for i in (0,1)]
  if other.startswith('gripper0_finger1'):thumb.append((radial,t))
  if other.startswith('gripper0_finger2'):index.append((radial,t))
 pairs=[(a,b) for a in thumb for b in index if a[0][0]*b[0][0]+a[0][1]*b[0][1]<0]
 opposed=bool(pairs);supported=bool(support)
 if table and not seen_table:first['positive_table_support']={'step':step,'phase':phase};seen_table=True
 if hand and not seen_hand:first['positive_hand_bowl_contact']={'step':step,'phase':phase,'geoms':[t['geom'] for t in hand],'normal_force_N':[t['force_contact_frame'][0] for t in hand]};seen_hand=True
 if opposed and 'opposed_thumb_index' not in first:first['opposed_thumb_index']={'step':step,'phase':phase,'opposed_pair_geoms':[[a[1]['geom'],b[1]['geom']] for a,b in pairs]}
 if seen_table and not supported:
  first.setdefault('unsupported_after_table',{'step':step,'phase':phase})
  if current_unsupported is None:current_unsupported={'first':step,'last':step,'phase_first':phase}
  current_unsupported['last']=step
 else:
  if current_unsupported:unsupported_after_table_intervals.append(current_unsupported);current_unsupported=None
 if seen_hand and not supported:first.setdefault('unsupported_after_hand_contact',{'step':step,'phase':phase})
 if phase=='lift' and not supported:first.setdefault('unsupported_in_lift',{'step':step,'phase':phase})
 lift=s['bowl_position_m'][2]-base
 if lift>maxlift:maxlift=lift;maxliftstep=step
 functional=(lift>=.02 and not supported and opposed)
 if functional:first.setdefault('20mm_unsupported_opposed_any_phase',{'step':step,'phase':phase,'lift_m':lift})
 validhold=phase=='hold' and functional
 consecutive=consecutive+1 if validhold else 0;best=max(best,consecutive)
 if phase=='hold':
  holdsteps+=1;minhold=min(minhold,lift);maxhold=max(maxhold,lift)
  tf=sum(t[1]['force_contact_frame'][0] for t in thumb);ixf=sum(t[1]['force_contact_frame'][0] for t in index)
  minholdthumb=min(minholdthumb,tf);minholdindex=min(minholdindex,ixf)
  if not validhold:bad_hold.append({'step':step,'lift_m':lift,'support':supported,'opposed':opposed})
 for key,a,b in [('environment_support',supported,s['environment_support']),('opposed_thumb_index_contact',opposed,s['opposed_thumb_index_contact']),('continuous_valid_hold_steps',consecutive,s['continuous_valid_hold_steps'])]:
  if a!=b:state_disagreements.append({'step':step,'field':key,'recomputed':a,'recorded':b})
 if s['lift_from_own_settled_m'] is not None and s['lift_from_own_settled_m']!=lift:state_disagreements.append({'step':step,'field':'lift_from_own_settled_m','recomputed':lift,'recorded':s['lift_from_own_settled_m']})
 if s['failures']:hard.append({'step':step,'failures':s['failures']})
 if s['warnings'] or not s['finite']:warnings.append({'step':step,'warnings':s['warnings'],'finite':s['finite']})
 final_z=s['bowl_position_m'][2];final_support=supported;final_opposed=opposed
if current_unsupported:unsupported_after_table_intervals.append(current_unsupported)
try:next(old_states);raise AssertionError('old states longer than245')
except StopIteration:pass
try:next(old_contacts);raise AssertionError('old contacts longer than245')
except StopIteration:pass
new_reset=read(RUN/'frozen-before-first-step.json');old_reset=read(OLD/'frozen-before-first-step.json.gz')
resetkeys=('qpos','qvel','ctrl','controller','criteria','native_scene','initial_contact_cache')
resetchecks={k:diff(old_reset[k],new_reset[k],k) for k in resetkeys}
new_cache=read(RUN/'drive-native-reset-cache.json');old_cache=read(OLD/'drive-native-reset-cache.json')
cachechecks={k:diff(old_cache[k],new_cache[k],k) for k in ('time_s','cache','warning_counts')}
old_snap=read(OLD/'robot-physical-preflight.json.gz');new_snap=read(RUN/'robot-physical-preflight.json')
compiledchecks={k:diff(old_snap[k],new_snap[k],k) for k in ('formal_snapshot','actual_snapshot')}
r={'mode':'PURE_STDLIB_STREAMING_NO_ENGINE_IMPORT_OR_OPERATIONS','source_run':str(RUN),'rows':rowcount,'sample_period_s':.002,
'contact_timing':'final RK4 internal-stage contact/force/body cache; qpos and bowl center in support/opposition predicate are recorded post-step; no new forward',
'criteria_recomputed':'Hold phase AND lift from step250 post-step>=0.020m AND zero positive-normal-force bowl/nonhand contacts AND positive-force thumb/index contacts with negative XY radial dot product; original function does not filter on filter_reasons.',
'first_events':first,'phase_ranges':phase_ranges,'step250_settled_z_m':base,'final_z_m':final_z,'final_lift_m':final_z-base,'maximum_lift_m':maxlift,'maximum_lift_step':maxliftstep,
'hold':{'planned_steps':750,'recorded_steps':holdsteps,'all750_valid':holdsteps==750 and not bad_hold,'bad_steps':bad_hold,'minimum_lift_m':minhold,'maximum_lift_m':maxhold,'longest_continuous_valid_steps':best,'longest_continuous_valid_s':best*.002,'original_1s_predicate':best*.002>=1.,'full1_5s_completed':best==750,'minimum_thumb_normal_force_sum_N':minholdthumb,'minimum_index_normal_force_sum_N':minholdindex},
'unsupported_after_positive_table_intervals':unsupported_after_table_intervals,'final_environment_support':final_support,'final_opposed':final_opposed,
'state_derived_field_mismatches':state_disagreements,'hard_failures':hard,'numerical_health_events':warnings,
'prefix':{'required_steps':245,'nonmetadata_state_fields':35,'state_exclusions':['failures','legacy_failures','research'],'all_contact_cache_fields':True,'exact_canonical_JSON':True,'differences':prefix,'pass':not prefix},
'reset':{'frozen_fields':resetchecks,'RK4_native_Reset_cache':cachechecks,'compiled_snapshots':compiledchecks,'pass':not any(resetchecks.values()) and not any(cachechecks.values()) and not any(compiledchecks.values())},
'native_task_result':read(RUN/'result.json')['native_task_result'],'native_task_result_scope':'saved actual env.check_success() value; cannot independently recompute without engine, reported separately',
'pass':rowcount==6500 and best==750 and not bad_hold and not prefix and not state_disagreements and not hard and not warnings}
OUT.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:r[k] for k in ('rows','first_events','step250_settled_z_m','final_lift_m','maximum_lift_m','maximum_lift_step','hold','unsupported_after_positive_table_intervals','prefix','reset','native_task_result','pass')},indent=2))
