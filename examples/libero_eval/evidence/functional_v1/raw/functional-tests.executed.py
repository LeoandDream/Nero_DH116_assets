"""Offline contracts for the two-reason functional_v1 classification change."""
from copy import deepcopy
import importlib.abc
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))


class NoEngine(importlib.abc.MetaPathFinder):
    def find_spec(self,fullname,path=None,target=None):
        if fullname.split('.')[0] in {'mujoco','robosuite','libero','numpy','scipy','PIL'}:
            raise AssertionError('Native dependency in offline test: '+fullname)


sys.meta_path.insert(0,NoEngine())
import close149_functional as F


class FunctionalTests(unittest.TestCase):
    def test_exactly_two_diagnostic_reasons_and_mixed_hard_stops(self):
        rules,_=F.R.load_contact_policy()
        for diagnostic in [[],['OUTSIDE_REGISTERED_PATH_ENVELOPE'],['ACTIVE_HOLD_TRACKING'],list(F.DIAGNOSTICS)]:
            for hard in [[],['ARM_TRACKING'],['MIMIC'],['JOINT_LIMIT'],['NONFINITE_STATE'],
                         ['NUMERICAL_WARNING_OR_TIME_RESET'],['UNEXPECTED_WRIST_EXTERNAL_CONTACT'],
                         ['ROBOT_CONTACT_OVERLAP'],['UNRECOGNIZED_PROTECTION']]:
                with self.subTest(diagnostic=diagnostic,hard=hard):
                    p=F.FunctionalPolicy(rules)
                    r=p.evaluate(1,'hold',[],diagnostic+hard)
                    self.assertEqual(r['research_failures'],hard)
                    self.assertEqual(r['old_research_failures'],diagnostic+hard)
                    self.assertEqual(r['legacy_failures'],diagnostic+hard)
                    self.assertEqual(set(r['diagnostics']),set(diagnostic))
                    self.assertEqual(p.first_research is not None,bool(hard))
                    self.assertEqual(p.original.first_research is not None,bool(diagnostic+hard))

    def test_contact_policy_results_and_history_are_unchanged(self):
        rules,_=F.R.load_contact_policy()
        old=F.R.ResearchPolicy(rules);new=F.FunctionalPolicy(rules)
        contact={'geom':['white_bowl_1_g23','gripper0_proxy76_finger23_link_06'],
            'body':['white_bowl_1_main','gripper0_finger23_link'], 'distance_m':-.0021,
            'position_world_m':[0.,0.,0.], 'frame':[1.,0.,0.,0.,1.,0.,0.,0.,1.],
            'normal_world':[1.,0.,0.], 'force_contact_frame':[0.]*6,'filter_reasons':[]}
        original=deepcopy(contact)
        for step in range(1,18):
            phase='close' if step<=10 else 'lift'
            a=old.evaluate(step,phase,[contact],['OUTSIDE_REGISTERED_PATH_ENVELOPE'])
            b=new.evaluate(step,phase,[contact],['OUTSIDE_REGISTERED_PATH_ENVELOPE'])
            for key in ('groups','warnings','contact_events','legacy_failures'):
                self.assertEqual(a[key],b[key])
            self.assertEqual([x for x in a['research_failures'] if x not in F.DIAGNOSTICS],b['research_failures'])
        self.assertIn('DEEP_CONTACT_PERSISTENT',b['research_failures'])
        self.assertEqual(new.first_research['step'],17)
        self.assertEqual(new.original.first_research['step'],1)
        self.assertEqual(contact,original)

    def test_old_B_prefix_and_diagnostic_magnitude(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder=F.Recorder(Path(tmp),mock.Mock())
            for old,cache in zip(recorder.old_states,recorder.old_contacts):
                state=deepcopy(old);reasons=[r for r in state['failures'] if r not in F.DIAGNOSTICS]
                state['failures']=reasons
                state['research']['diagnostics']=[r for r in old['failures'] if r in F.DIAGNOSTICS]
                recorder.record_step(state,deepcopy(cache),reasons,{}, {})
                self.assertFalse(reasons)
            self.assertEqual(recorder.prefix_checked,245)
            self.assertEqual(recorder.first_diagnostics['OUTSIDE_REGISTERED_PATH_ENVELOPE']['step'],245)
            item=state['functional_v1']['details']['OUTSIDE_REGISTERED_PATH_ENVELOPE'][0]
            self.assertEqual(item['joint'],'gripper0_finger11')
            self.assertAlmostEqual(item['outside_rad'],2.013680576538e-5)

    def test_any_physics_or_contact_difference_stops_before_next_step(self):
        for target in ('state','contact','nonfinite','missing_null_field'):
            with self.subTest(target=target),tempfile.TemporaryDirectory() as tmp:
                recorder=F.Recorder(Path(tmp),mock.Mock())
                state=deepcopy(recorder.old_states[0]);cache=deepcopy(recorder.old_contacts[0]);reasons=[]
                if target=='state': state['ctrl'][0]+=.000001
                elif target=='contact': cache['body_poses']['gripper0_base_link']['position_world_m'][0]+=.000001
                elif target=='nonfinite': state['full_qpos'][0]=float('nan');state['finite']=False;reasons.append('NONFINITE_STATE')
                else: del state['lift_from_own_settled_m']
                result={}
                recorder.record_step(state,cache,reasons,{'macro':0,'phase':'settle_preshape'},result)
                self.assertEqual(reasons,(['NONFINITE_STATE'] if target=='nonfinite' else [])+['PREFIX_PHYSICS_MISMATCH'])
                self.assertEqual(result['first_failure']['step'],1)
                self.assertTrue(recorder.prefix_mismatch)

    def test_tracking_diagnostic_does_not_change_grasp_predicate(self):
        with tempfile.TemporaryDirectory() as tmp:
            legacy=types.SimpleNamespace(robot_audit=lambda *_:None,write=lambda *_:None)
            output=Path(tmp)
            (output/'registered-protocol.json').write_text(json.dumps(F.R.read(F.OLD/'registered-protocol.json')))
            p=F.FunctionalPolicy(F.R.load_contact_policy()[0])
            F.adapted_executor(legacy,p,object(),output)
            source=(output/'executor.executed.py').read_text()
            old=(F.OLD/'executor.executed.py').read_text()
            for line in old.splitlines():
                if any(x in line for x in ['holding =','hold_count =','best_hold =',"result['grasp_result'] =",'ACTIVE_HOLD_TRACKING','criteria[']):
                    self.assertIn(line,source)
            self.assertIn('_functional.record_step(state, cache, reasons, current, result)',source)

    def test_explicit_protocol_existing_output_and_exit_code(self):
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp);marker=out/'keep';marker.write_text('unchanged')
            with self.assertRaises(ValueError): F.run_once(out,'functional_v1')
            self.assertEqual(list(out.iterdir()),[marker])
            with self.assertRaises(ValueError): F.run_once(out/'new',None)
        for code in (0,1,2):
            with tempfile.TemporaryDirectory() as tmp:
                p=Path(tmp)/'new';entry=mock.Mock(return_value=code)
                with mock.patch.object(F,'prepared_entry',return_value=(entry,'old','new')):
                    self.assertEqual(F.run_once(p,'functional_v1'),code)
                entry.assert_called_once_with(p)

    def test_old_physics_sources_and_actions_pinned(self):
        F.validate_sources()
        self.assertEqual(F.D.C.canonical_sha(F.D.C.planned_schedule()),F.D.load_profile()['schedule_canonical_json_sha256'])
        entry,_,source=F.prepared_entry(F.ROOT/'runs/offline_functional_test')
        self.assertIs(entry.__globals__['ResearchPolicy'],F.FunctionalPolicy)
        self.assertEqual(entry.__globals__['BASELINE'],F.BASELINE)
        self.assertIn('_drive_source',source)


if __name__=='__main__':unittest.main(verbosity=2)
