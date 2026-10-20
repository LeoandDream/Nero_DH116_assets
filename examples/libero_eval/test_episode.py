"""Offline schema checks; never construct an environment."""
import gzip
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import subprocess
import sys
import shutil

HERE = Path(__file__).resolve().parent


class EpisodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not (HERE / 'episode.py').exists():
            raise AssertionError('The offline episode exporter is not implemented')
        spec = importlib.util.spec_from_file_location('lr_episode_test', HERE / 'episode.py')
        cls.api = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.api)

    def test_left_export_preserves_missing_native_signal_and_recorded_actions(self):
        root = HERE.parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / 'episode'
            result = self.api.export_episode(root, 'L', 'baseline', output)
            with gzip.open(output / 'steps.jsonl.gz', 'rt') as stream:
                first = json.loads(next(stream))
            self.assertEqual(first['physics_step'], 1)
            self.assertEqual(len(first['state_18']), 18)
            self.assertEqual(len(first['action_13']), 13)
            self.assertIsNone(first['native_task_success'])
            self.assertIsNone(first['cameras'])
            self.assertEqual(result['physics_steps'], 5750)
            self.assertFalse(result['native_task_success'])
            self.assertEqual(result['episode_status'], 'COMPLETED')
            self.assertEqual(result['new_physics_steps'], 0)
            manifest = json.loads((output / 'manifest.json').read_text())
            self.assertEqual(manifest['initialization']['kind'], 'native_seeded_reset')
            self.assertFalse(manifest['initialization']['official_panda_init_state_supported'])
            self.assertEqual(manifest['model']['hand_collision_proxies'], 76)

    def test_right_partial_episode_is_truncated_and_preserves_rk4_cache(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = HERE.parents[1]
            raw = Path(tmp) / 'raw'; raw.mkdir()
            state = {'step': 1, 'macro': 0, 'time_s': .002, 'phase': 'close',
                     'robot_state_18': [0] * 18, 'action_13': [0] * 13,
                     'ctrl': [0] * 13, 'warnings': {}, 'full_qpos': [0] * 33,
                     'full_qvel': [0] * 31, 'finite': True, 'mimic_rad': .001,
                     'legacy_failures': ['ROBOT_CONTACT_OVERLAP'],
                     'research': {'research_failures': [], 'warnings': ['CONTACT_DEPTH']}}
            (raw / 'state.jsonl').write_text(json.dumps(state)+'\n')
            contact = {'step': 1, 'timing': 'final RK4 internal-stage cache',
                       'contacts': [{'geom': ['a', 'b'], 'position_world_m': [1, 2, 3]}],
                       'body_poses': {'a': {'position_world_m': [1, 2, 3]}}}
            (raw / 'contacts.jsonl').write_text(json.dumps(contact)+'\n')
            (raw / 'result.json').write_text(json.dumps({'physics_steps': 1,
                'planned_physics_steps': 6750, 'execution': 'STOPPED_AT_FIRST_FAILURE',
                'native_task_result': False, 'first_failure': {'step': 1},
                'research_policy_sha256': self.api.digest(root / 'examples/right_hand/evidence/d3_research_contact_policy/research-policy.json'),
                'grasp_result': 'UNKNOWN_EARLY_STOP', 'control_health': 'FAIL'}))
            result = self.api.export_episode(root, 'R', 'research', Path(tmp) / 'episode', raw)
            self.assertEqual(result['episode_status'], 'TRUNCATED')
            self.assertFalse(result['native_task_success'])
            with gzip.open(Path(tmp) / 'episode/steps.jsonl.gz', 'rt') as stream:
                row = json.loads(next(stream))
            self.assertEqual(row['contact_cache_timing'], contact['timing'])
            self.assertEqual(row['contacts'][0]['geom'], contact['contacts'][0]['geom'])
            self.assertEqual(row['contacts'][0]['position_world_m'], [1, 2, 3])
            self.assertIsNone(row['contacts'][0]['frame'])
            self.assertEqual(row['rk4_body_poses'], contact['body_poses'])
            self.assertEqual(row['legacy_failures'], state['legacy_failures'])
            self.assertEqual(row['research'], state['research'])
            self.assertEqual(result['simulator_health']['status'], 'PASS')

    def test_explicit_left_source_uses_its_images_and_no_historical_fallback(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = HERE.parents[1]
            source = Path(tmp) / 'raw'; source.mkdir()
            delivered = root / 'examples/libero_reference/evidence'
            for name in ('bounded_grasp.jsonl.gz', 'contact-details.jsonl.gz', 'run-result.json'):
                shutil.copyfile(delivered / name, source / name)
            (source / 'agentview_openpi_final.png').write_bytes(b'new-source-camera')
            result = self.api.export_episode(root, 'L', 'baseline', Path(tmp) / 'first', source)
            cameras = json.loads((Path(tmp) / 'first/cameras.json').read_text())
            self.assertEqual(cameras['final_images']['images']['agentview']['sha256'],
                             self.api.digest(source / 'agentview_openpi_final.png'))
            self.assertIsNone(cameras['final_images']['images']['robot0_eye_in_hand'])
            self.assertEqual(result['simulator_health']['status'], 'UNKNOWN')
            self.assertEqual(result['simulator_health']['checks']['recorded_finite_arrays'], 'PASS')
            (source / 'agentview_openpi_final.png').unlink()
            self.api.export_episode(root, 'L', 'baseline', Path(tmp) / 'second', source)
            missing = json.loads((Path(tmp) / 'second/cameras.json').read_text())
            self.assertIsNone(missing['final_images'])

    def test_execute_preserves_backend_stop_exit_code_after_export(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'checkout'
            (root / 'examples/right_hand').mkdir(parents=True)
            (root / 'examples/libero_eval').mkdir()
            policy_dir = root / 'examples/right_hand/evidence/d3_research_contact_policy'
            policy_dir.mkdir(parents=True)
            shutil.copyfile(HERE.parents[1] / 'examples/right_hand/evidence/d3_research_contact_policy/research-policy.json', policy_dir / 'research-policy.json')
            (root / 'examples/right_hand/candidates/finger22_local').mkdir(parents=True)
            (root / 'examples/right_hand/candidates/finger22_local/model.xml').write_text('<test/>')
            for name in ('run.py', 'episode.py'):
                shutil.copyfile(HERE / name, root / 'examples/libero_eval' / name)
            backend = '''
import hashlib,json
from pathlib import Path
def run_once(output):
 output.mkdir(parents=True)
 state={'step':1,'macro':0,'time_s':.002,'phase':'close','robot_state_18':[0]*18,'action_13':[0]*13,'ctrl':[0]*13,'warnings':{},'finite':True}
 (output/'state.jsonl').write_text(json.dumps(state)+'\\n')
 (output/'contacts.jsonl').write_text(json.dumps({'step':1,'contacts':[]})+'\\n')
 policy=Path(__file__).parent/'evidence/d3_research_contact_policy/research-policy.json'
 (output/'result.json').write_text(json.dumps({'physics_steps':1,'planned_physics_steps':6750,'native_task_result':False,'execution':'STOPPED_AT_FIRST_FAILURE','research_policy_sha256':hashlib.sha256(policy.read_bytes()).hexdigest()}))
 return 2
'''
            (root / 'examples/right_hand/libero_research.py').write_text(backend)
            output = root / 'runs/episode'
            process = subprocess.run([sys.executable, '-B', str(root / 'examples/libero_eval/run.py'),
                '--hand', 'R', '--mode', 'research', '--execute', '--output', str(output)], capture_output=True, text=True)
            self.assertEqual(process.returncode, 2, process.stderr)
            self.assertTrue((output / 'manifest.json').is_file())
            self.assertFalse(json.loads((output / 'result.json').read_text())['native_task_success'])

    def test_existing_output_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            marker = output / 'keep'; marker.write_text('unchanged')
            with self.assertRaises(FileExistsError):
                self.api.export_episode(HERE.parents[1], 'L', 'baseline', output)
            self.assertEqual(marker.read_text(), 'unchanged')

    def test_aggregate_keeps_truncation_out_of_completed_only_denominator(self):
        with tempfile.TemporaryDirectory() as tmp:
            paths = []
            for i, (status, success) in enumerate([('COMPLETED', True), ('COMPLETED', False),
                                                   ('TRUNCATED', False), ('INCOMPLETE', None)]):
                p = Path(tmp) / str(i); p.mkdir(); paths.append(p)
                (p / 'manifest.json').write_text(json.dumps({'episode_id': str(i), 'hand': 'L', 'mode': 'baseline',
                    'source_files': [{'path': 'bounded_grasp.jsonl', 'sha256': f'{i:064x}'}]}))
                (p / 'result.json').write_text(json.dumps({'episode_status': status,
                    'native_task_success': success, 'physics_steps': 1}))
            result = self.api.aggregate(paths)
            self.assertEqual(result['n_episodes'], 2)
            self.assertEqual(result['n_success'], 1)
            self.assertEqual(result['success_rate'], .5)
            self.assertEqual(result['n_truncated'], 1)
            self.assertEqual(result['n_incomplete'], 1)
            self.assertAlmostEqual(result['attempted_success_rate'], 1/3)

    def test_aggregate_rejects_same_actual_trace_exported_under_different_names(self):
        with tempfile.TemporaryDirectory() as tmp:
            actual = HERE / 'evidence/left_001'
            manifest = json.loads((actual / 'manifest.json').read_text())
            source_trace = next(item for item in manifest['source_files']
                                if Path(item['path']).name == 'bounded_grasp.jsonl.gz')
            self.assertEqual(source_trace['sha256'], self.api.digest(HERE.parents[1] / source_trace['path']))
            paths = []
            for name in ('export-original', 'export-renamed-copy'):
                output = Path(tmp) / name; output.mkdir(); paths.append(output)
                manifest['episode_id'] = name
                (output / 'manifest.json').write_text(json.dumps(manifest))
                shutil.copyfile(actual / 'result.json', output / 'result.json')
            with self.assertRaisesRegex(ValueError, 'Duplicate source trajectory'):
                self.api.aggregate(paths)

    def test_aggregate_allows_distinct_declared_run_sources_with_identical_trace_sha(self):
        with tempfile.TemporaryDirectory() as tmp:
            actual = HERE / 'evidence/left_001'
            original = json.loads((actual / 'manifest.json').read_text())
            paths = []
            for name in ('attempt-a', 'attempt-b'):
                output = Path(tmp) / name; output.mkdir(); paths.append(output)
                manifest = json.loads(json.dumps(original)); manifest['episode_id'] = name
                for record in manifest['source_files']:
                    if Path(record['path']).name == 'bounded_grasp.jsonl.gz':
                        record['path'] = str(Path(tmp) / name / 'raw/bounded_grasp.jsonl.gz')
                (output / 'manifest.json').write_text(json.dumps(manifest))
                shutil.copyfile(actual / 'result.json', output / 'result.json')
            result = self.api.aggregate(paths)
            self.assertEqual(result['n_episodes'], 2)
            self.assertIn('do not certify independent execution', result['independence'])

    def test_research_requires_correct_hand_and_raw_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                self.api.export_episode(HERE.parents[1], 'L', 'research', Path(tmp) / 'a')
            with self.assertRaises(ValueError):
                self.api.export_episode(HERE.parents[1], 'R', 'research', Path(tmp) / 'b')
            with self.assertRaisesRegex(ValueError, 'cannot relabel'):
                self.api.export_episode(HERE.parents[1], 'R', 'research', Path(tmp) / 'c',
                    HERE.parents[1] / 'examples/right_hand/evidence/d3_finger22_exploratory/run')

    def test_default_cli_exports_without_engine_or_scientific_imports(self):
        with tempfile.TemporaryDirectory() as tmp:
            code = '''
import importlib.abc, runpy, sys
class NoEngine(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('mujoco','robosuite','libero','numpy','scipy'):
            raise AssertionError('Offline export imported an engine dependency: '+fullname)
sys.meta_path.insert(0, NoEngine())
sys.path.insert(0, sys.argv[1])
sys.argv = [sys.argv[1]+'/run.py','--hand','L','--output',sys.argv[2]]
runpy.run_path(sys.argv[0],run_name='__main__')
'''
            process = subprocess.run([sys.executable, '-B', '-c', code, str(HERE), str(Path(tmp) / 'episode')],
                                     capture_output=True, text=True)
            self.assertEqual(process.returncode, 0, process.stderr)
            self.assertEqual(json.loads(process.stdout)['physics_steps'], 5750)


if __name__ == '__main__':
    unittest.main()
