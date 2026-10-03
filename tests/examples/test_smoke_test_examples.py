"""Regression coverage for smoke-result classification and executable paths."""
import importlib.util
import subprocess
import unittest
from pathlib import Path
from unittest.mock import patch

MODULE_PATH = Path(__file__).resolve().parents[2] / "tools/smoke_test_examples.py"
spec = importlib.util.spec_from_file_location("smoke", MODULE_PATH)
smoke = importlib.util.module_from_spec(spec)
spec.loader.exec_module(smoke)


class SmokeResultsTest(unittest.TestCase):
    def completed(self, stdout="", stderr="", returncode=0):
        result = subprocess.CompletedProcess([], returncode, stdout, stderr)
        with patch.object(smoke.subprocess, "run", return_value=result):
            return smoke.run_example(Path("sample"), 1, False)[0]

    def timeout(self, stdout=b"", stderr=b""):
        error = subprocess.TimeoutExpired("sample", 1, output=stdout, stderr=stderr)
        with patch.object(smoke.subprocess, "run", side_effect=error):
            return smoke.run_example(Path("sample"), 1, False)[0]

    def test_nan_is_case_insensitive(self):
        self.assertEqual(self.completed(stderr="NaN detected"), "FAIL")

    def test_timeout_checks_stdout_and_stderr(self):
        self.assertEqual(self.timeout(stdout=b"assertion failed"), "FAIL")
        self.assertEqual(self.timeout(stderr=b"NaN detected"), "FAIL")

    def test_timeout_checks_output_limit(self):
        self.assertEqual(self.timeout(stderr=b"line\n" * 51), "FAIL")

    def test_normal_event_loop_timeout_passes(self):
        self.assertEqual(self.timeout(), "PASS")

    def test_sdl_message_does_not_hide_crash(self):
        self.assertEqual(self.completed(stderr="SDL_CreateWindow failed", returncode=-11), "FAIL")
        self.assertEqual(self.completed(stderr="SDL_CreateWindow failed", returncode=139), "FAIL")
        self.assertEqual(self.completed(stderr="SDL_CreateWindow failed\nassertion failed", returncode=1), "FAIL")

    def test_expected_video_failure_skips(self):
        self.assertEqual(self.completed(stderr="SDL_CreateWindow failed", returncode=1), "SKIP")

    def test_successful_sdl_message_does_not_skip(self):
        self.assertEqual(self.completed(stderr="SDL_Init succeeded"), "PASS")

    def test_relative_path_is_resolved_before_launch(self):
        with patch.object(smoke.subprocess, "run", return_value=subprocess.CompletedProcess([], 0, "", "")) as run:
            smoke.run_example(Path("sample"), 1, False)
            self.assertEqual(run.call_args.args[0], [str(Path("sample").resolve())])

    def test_unexpected_exit_fails(self):
        self.assertEqual(self.completed(returncode=2), "FAIL")


if __name__ == "__main__":
    unittest.main()
