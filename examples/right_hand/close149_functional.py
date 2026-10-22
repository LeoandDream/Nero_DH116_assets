#!/usr/bin/env python3
"""Explicit functional_v1: two diagnostics, frozen B physics, one attempt."""
import argparse
from copy import deepcopy
import difflib
import gzip
import json
import math
from pathlib import Path

import close149_drive as D

R,ROOT=D.R,D.ROOT
BASELINE='1cec8e94930d19223d94c11fe4dbf5c51b92024f'
AUTHORIZATION='https://github.com/LeoandDream/Nero_DH116_assets/issues/5#issuecomment-6346434609'
OLD=ROOT/'examples/libero_eval/evidence/left_drive_profile_v1/raw'
DRIVE_SHA='7ba5a51c70e97f34be588771d908db1fbd009255c7b790984fac719c9aa6cbeb'
DIAGNOSTICS=('OUTSIDE_REGISTERED_PATH_ENVELOPE','ACTIVE_HOLD_TRACKING')
PREFIX_SHA={'state.jsonl.gz':'9fcf6944319346bbb73ee6f4fe62b073bb2b4c15e94a867be563d2db6ef5cce1',
            'contacts.jsonl.gz':'5c057c9d6a9cf1cfe24deaf2700b8f2a3b988fc8b63ac5d5ed2c50f554bba9b5'}


def validate_sources():
    if R.digest(Path(D.__file__))!=DRIVE_SHA:raise ValueError('Frozen B runner changed')
    for name,sha in PREFIX_SHA.items():
        if R.digest(OLD/name)!=sha:raise ValueError('Frozen B prefix changed: '+name)
    D.load_profile()
    R.load_contact_policy()


class FunctionalPolicy:
    """Call the original tracker once; only classify two returned reasons."""
    def __init__(self,rules):
        self.original=R.ResearchPolicy(rules)
        self.first_research=None

    @property
    def first_legacy(self):return self.original.first_legacy

    def evaluate(self,step,phase,contacts,independent_reasons):
        original=self.original.evaluate(step,phase,contacts,independent_reasons)
        row=deepcopy(original)
        row['old_research_failures']=list(original['research_failures'])
        row['diagnostics']=[r for r in original['research_failures'] if r in DIAGNOSTICS]
        row['research_failures']=[r for r in original['research_failures'] if r not in DIAGNOSTICS]
        row['measurement_protocol']='functional_v1'
        if row['research_failures'] and self.first_research is None:
            self.first_research={'step':step,'reasons':row['research_failures']}
        return row


def stable(value):
    # Same null representation as the frozen finite_json writer; a genuine
    # NONFINITE hard stop must still retain its failed step and contact cache.
    def finite(v):
        if isinstance(v,float) and not math.isfinite(v):return None
        if isinstance(v,dict):return {k:finite(x) for k,x in v.items()}
        if isinstance(v,list):return [finite(x) for x in v]
        return v
    return json.dumps(finite(value),sort_keys=True,separators=(',',':'),allow_nan=False)


def first_difference(a,b,path='$'):
    if stable(a)==stable(b):return None
    if isinstance(a,dict) and isinstance(b,dict) and set(a)==set(b):
        for key in a:
            difference=first_difference(a[key],b[key],path+'.'+str(key))
            if difference:return difference
    elif isinstance(a,list) and isinstance(b,list) and len(a)==len(b):
        for i,(x,y) in enumerate(zip(a,b)):
            difference=first_difference(x,y,path+f'[{i}]')
            if difference:return difference
    return {'path':path,'old':json.loads(stable(a)),'new':json.loads(stable(b))}


class Recorder:
    def __init__(self,output,policy):
        self.output,self.policy=Path(output),policy
        self.old_states,self.old_contacts=[],[]
        for name,target in [('state.jsonl.gz',self.old_states),('contacts.jsonl.gz',self.old_contacts)]:
            with gzip.open(OLD/name,'rt') as stream:target.extend(json.loads(line) for line in stream)
        if len(self.old_states)!=245 or len(self.old_contacts)!=245:raise ValueError('Expected complete old B prefix')
        self.protocol=R.read(OLD/'registered-protocol.json')
        self.criteria=R.read(OLD/'frozen-before-first-step.json.gz')['criteria']
        snapshot=R.read(OLD/'robot-physical-preflight.json.gz')['formal_snapshot']
        self.names=[row['name'] for row in snapshot['objects']['JOINT']]
        self.first_diagnostics={};self.diagnostic_steps={r:0 for r in DIAGNOSTICS}
        self.prefix_checked=0;self.prefix_mismatch=None

    def check_reset(self,value):
        old=R.read(OLD/'frozen-before-first-step.json.gz')
        for key in ('qpos','qvel','ctrl','controller','criteria','native_scene','initial_contact_cache'):
            if stable(value[key])!=stable(old[key]):raise ValueError('Old B Reset differs: '+key)
        new_cache=R.read(self.output/'drive-native-reset-cache.json')
        old_cache=R.read(OLD/'drive-native-reset-cache.json')
        for key in ('time_s','cache','warning_counts'):
            if stable(new_cache[key])!=stable(old_cache[key]):raise ValueError('Old B Reset cache differs: '+key)
        new=R.read(self.output/'robot-physical-preflight.json')
        old=R.read(OLD/'robot-physical-preflight.json.gz')
        for key in ('formal_snapshot','actual_snapshot'):
            if stable(new[key])!=stable(old[key]):raise ValueError('Old B compiled snapshot differs: '+key)
        R.write(self.output/'functional-reset-comparison.json',{
            'passed':True,'old_B_Reset_and_compiled_snapshots_exact':True,
            'old_B_native_Reset_caches_exact':True,'new_integration_steps':0})

    def record_step(self,state,cache,reasons,current,result):
        step=state['step'];details={}
        diagnostics=state['research'].get('diagnostics',[])
        if 'OUTSIDE_REGISTERED_PATH_ENVELOPE' in diagnostics:
            details['OUTSIDE_REGISTERED_PATH_ENVELOPE']=[]
            for name,q,lo,hi in zip(self.names,state['robot_state_18'],
                                  self.protocol['qpos_envelope_lower_rad'],self.protocol['qpos_envelope_upper_rad']):
                if q<lo or q>hi:
                    details['OUTSIDE_REGISTERED_PATH_ENVELOPE'].append({'joint':name,'q_rad':q,
                        'lower_rad':lo,'upper_rad':hi,'outside_rad':max(lo-q,q-hi),'side':'lower' if q<lo else 'upper'})
        if 'ACTIVE_HOLD_TRACKING' in diagnostics:
            details['ACTIVE_HOLD_TRACKING']={'active_errors_rad':state['active_error_rad'],
                'maximum_rad':max(state['active_error_rad']),
                'original_diagnostic_threshold_rad':self.criteria['steady_active_error_rad_max']}
        for reason in diagnostics:
            self.diagnostic_steps[reason]+=1
            if reason not in self.first_diagnostics:
                self.first_diagnostics[reason]={'step':step,'phase':state['phase'],'details':details.get(reason)}
        state['functional_v1']={'diagnostics':diagnostics,'details':details,
            'old_research_failures':state['research'].get('old_research_failures',[]),
            'continued_despite_diagnostics':bool(diagnostics and not reasons)}
        if step<=245:
            old=self.old_states[step-1]
            keys=[k for k in old if k not in ('failures','legacy_failures','research')]
            projected={k:state[k] for k in keys if k in state}
            expected={k:old[k] for k in keys}
            difference=first_difference(expected,projected,'state')
            if difference is None:difference=first_difference(self.old_contacts[step-1],cache,'contact_cache')
            if difference:
                self.prefix_mismatch={'step':step,**difference}
                R.write(self.output/'prefix-physics-mismatch.json',self.prefix_mismatch)
                reasons.append('PREFIX_PHYSICS_MISMATCH')
                state['failures']=reasons
                state['functional_v1']['continued_despite_diagnostics']=False
                result['first_failure']={'step':step,'macro':current.get('macro'),
                    'phase':current.get('phase'),'reasons':list(reasons)}
            else:self.prefix_checked+=1

    def summary(self):
        return {'protocol':'functional_v1','diagnostic_only':list(DIAGNOSTICS),
            'first_diagnostics':self.first_diagnostics,'diagnostic_step_counts':self.diagnostic_steps,
            'old_research_first_failure':self.policy.original.first_research,
            'old_legacy_first_failure':self.policy.original.first_legacy,
            'functional_first_failure':self.policy.first_research,
            'prefix_physics':{'matched_steps':self.prefix_checked,'required_steps':245,
                'first_difference':self.prefix_mismatch,'state_exclusions':['failures','legacy_failures','research'],
                'all_contact_cache_fields_compared':True,'comparison':'exact canonical JSON; no tolerance'},
            'previous_finite_path_geometry_scope':'Not asserted after the original envelope is exceeded.'}


def adapted_executor(legacy,policy,observations,output):
    execute=D.adapted_executor(legacy,policy,observations,output)
    scope=execute.__globals__;old_write=scope['write']
    recorder=Recorder(output,policy)
    def write(path,value):
        if Path(path).name=='frozen-before-first-step.json':
            recorder.check_reset(value)
            value['measurement_protocol']='functional_v1'
        if Path(path).name=='result.json':
            report=recorder.summary()
            report['functional_first_failure']=value.get('first_failure')
            value.update(measurement_protocol='functional_v1',functional_v1=report)
            R.write(Path(output)/'functional-diagnostics.json',report)
        old_write(path,value)
    scope.update(write=write,_functional=recorder)
    path=Path(output)/'executor.executed.py';original=path.read_text()
    changed=D.C.replace_once(original,"            trace.write(json.dumps(finite_json(state), allow_nan=False)+'\\n')",'')
    changed=D.C.replace_once(changed,"            contact_log.write(json.dumps(finite_json(cache), allow_nan=False)+'\\n')",
        "            _functional.record_step(state, cache, reasons, current, result)\n"
        "            trace.write(json.dumps(finite_json(state), allow_nan=False)+'\\n')\n"
        "            contact_log.write(json.dumps(finite_json(cache), allow_nan=False)+'\\n')")
    (Path(output)/'functional-executor.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True),changed.splitlines(True))))
    path.write_text(changed)
    exec(compile(changed,str(path),'exec'),scope)
    return scope['execute']


def prepared_entry(output):
    validate_sources()
    entry,original,changed=D.prepared_entry(output)
    changed=D.C.replace_once(changed,"experiment='issue5_close149_left_drive_profile_v1_once'",
                            "experiment='issue5_close149_B_functional_v1_once'")
    changed=D.C.replace_once(changed,'(Path(__file__), _close149_source,',
                            '(Path(__file__), _drive_source, _close149_source,')
    scope=dict(entry.__globals__)
    scope.update(__file__=__file__,BASELINE=BASELINE,AUTHORIZATION=AUTHORIZATION,
        ResearchPolicy=FunctionalPolicy,adapted_executor=adapted_executor,_drive_source=Path(D.__file__))
    scope['_close149_registration']=dict(scope['_close149_registration'],
        measurement_protocol='functional_v1',diagnostic_only=list(DIAGNOSTICS),
        all_other_research_contact_and_independent_protections='UNCHANGED',
        physics_prefix_to_old_B='REQUIRED_EXACT_FIRST245_ONLINE',
        old_B_prefix_sources={name:str(OLD/name) for name in PREFIX_SHA},
        old_B_prefix_sha256=PREFIX_SHA,
        finite_path_geometry_scope='Not asserted after old envelope exceedance')
    exec(compile(changed,str(Path(__file__)),'exec'),scope)
    return scope['run_once'],original,changed


def run_once(output,protocol=None):
    if protocol!='functional_v1':raise ValueError('Explicit --protocol functional_v1 required')
    output=Path(output).resolve()
    if output.exists():raise ValueError('Fresh output required; refusing another attempt')
    entry,original,changed=prepared_entry(output)
    code=entry(output)
    if output.exists():
        (output/'functional-entry.executed.py').write_text(changed+'\n')
        (output/'functional-entry.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True),changed.splitlines(True))))
    return code


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol',required=True,choices=['functional_v1'])
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    raise SystemExit(run_once(args.output,args.protocol))
