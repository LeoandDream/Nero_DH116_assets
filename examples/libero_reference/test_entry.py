import subprocess
import importlib.util
import sys
from pathlib import Path
import tempfile
import unittest

HERE = Path(__file__).resolve().parent


class EntryTests(unittest.TestCase):
    def classify(self, case, equal, steps, commands):
        spec = importlib.util.spec_from_file_location('reference_entry', HERE / 'run.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertTrue(hasattr(module, 'classify_outcome'), 'runtime failure classification is missing')
        return module.classify_outcome(case, equal, steps, commands)

    def test_runtime_or_final_output_failure_is_not_a_completed_comparison(self):
        for case, steps, commands in [
            ({'status': 'DIAGNOSTIC_STOPPED', 'stop_reason': 'RUNTIME_ERROR: OSError: disk full'}, 1, 0),
            ({'status': 'DIAGNOSTIC_STOPPED', 'stop_reason': 'SETUP_OR_OUTPUT_ERROR: OSError: image write failed'}, 5750, 230),
            ({'status': 'DIAGNOSTIC_STOPPED', 'stop_reason': 'NUMERICAL_WARNING_OR_RESET', 'recording_error': 'disk full'}, 1, 0),
        ]:
            self.assertEqual(self.classify(case, True, steps, commands), ('FAILED', 1))

    def test_original_diagnostic_bound_stop_remains_a_reported_difference(self):
        case = {'status': 'DIAGNOSTIC_STOPPED', 'stop_reason': 'NUMERICAL_WARNING_OR_RESET'}
        self.assertEqual(self.classify(case, False, 1, 0), ('COMPLETED_WITH_DIFFERENCES', 2))

    def test_entry_documents_bundled_reference_and_refuses_external_outputs(self):
        runner = HERE / 'run.py'
        self.assertTrue(runner.exists(), 'standalone entry is not implemented')
        help_result = subprocess.run([sys.executable, str(runner), '--help'], capture_output=True, text=True)
        self.assertEqual(help_result.returncode, 0, help_result.stderr)
        self.assertIn('--reference', help_result.stdout)
        self.assertNotIn('--history-root', help_result.stdout)
        with tempfile.TemporaryDirectory() as directory:
            outside = Path(directory) / 'must-not-exist'
            result = subprocess.run([sys.executable, str(runner), '--output', str(outside)], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('checkout', result.stderr)
            self.assertFalse(outside.exists())


if __name__ == '__main__':
    unittest.main()
