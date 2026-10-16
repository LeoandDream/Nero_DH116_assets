"""Replay selection and output safety require no simulator or video encoder."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

HERE = Path(__file__).resolve().parent


def renderer():
    spec = importlib.util.spec_from_file_location('reference_render_video', HERE / 'render_video.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class VideoTests(unittest.TestCase):
    def test_frame_indices_preserve_recorded_command_endpoints(self):
        indices = renderer().frame_indices(5750)
        self.assertEqual(len(indices), 230)
        self.assertEqual((indices[0], indices[-1]), (24, 5749))
        self.assertTrue(all(b - a == 25 for a, b in zip(indices, indices[1:])))
        with self.assertRaises(ValueError):
            renderer().frame_indices(5749)

    def test_existing_output_is_rejected_before_reading_run_or_loading_simulator(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / 'existing.mp4'
            output.write_bytes(b'preserve me')
            with self.assertRaises(FileExistsError):
                renderer().render_video(Path(temporary) / 'absent-run', output)
            self.assertEqual(output.read_bytes(), b'preserve me')


if __name__ == '__main__':
    unittest.main()
