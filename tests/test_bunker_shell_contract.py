#!/usr/bin/env python3

from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
WRAPPER = ROOT / "scripts/with_bunker_env.bash"
VALIDATOR = ROOT / "scripts/validate_bunker_install.bash"
SMOKE = ROOT / "scripts/smoke_bunker_standalone.bash"


class BunkerShellContractTest(unittest.TestCase):
    def test_tools_have_side_effect_free_help(self):
        for program in (WRAPPER, VALIDATOR, SMOKE):
            completed = subprocess.run(
                [str(program), "--help"], cwd=ROOT, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                timeout=5, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("usage", completed.stdout.lower())

    def test_wrapper_preserves_arguments_and_localizes_state(self):
        with tempfile.TemporaryDirectory(prefix="bunker-env-") as directory:
            completed = subprocess.run(
                [
                    str(WRAPPER), "--run-dir", directory, "--",
                    "/usr/bin/python3", "-c",
                    "import os,sys; print(sys.argv[1]); print(os.environ['HOME']); print(os.environ['ROS_LOG_DIR']); print(os.environ['GAZEBO_PLUGIN_PATH'])",
                    "argument with spaces",
                ],
                cwd=ROOT, text=True, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, timeout=10, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            lines = completed.stdout.splitlines()
            self.assertEqual(lines[0], "argument with spaces")
            self.assertEqual(lines[1], str(Path(directory) / "home"))
            self.assertEqual(lines[2], str(Path(directory) / "ros-log"))
            self.assertIn(
                str(ROOT / "install/p450-clean/lib"),
                lines[3].split(":"),
            )

    def test_invalid_smoke_option_is_rejected_without_starting_runtime(self):
        completed = subprocess.run(
            [str(SMOKE), "--not-an-option"], cwd=ROOT, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=5, check=False)
        self.assertEqual(completed.returncode, 64)
        self.assertIn("unknown option", completed.stderr)

    def test_smoke_lets_roslaunch_own_ordered_child_shutdown(self):
        source = SMOKE.read_text(encoding="utf-8")
        self.assertIn('/bin/kill -INT "$launch_pid"', source)
        self.assertNotIn('/bin/kill -INT -- "-$launch_pid"', source)
        self.assertNotIn(
            'wait "$launch_pid" 2>/dev/null || true', source)

    def test_smoke_unloads_the_model_before_gazebo_shutdown(self):
        source = SMOKE.read_text(encoding="utf-8")
        checker = source.index("check_bunker_runtime.py")
        deletion = source.index("rosservice call /gazebo/delete_model")
        self.assertLess(checker, deletion)
        self.assertIn("success: True", source[deletion:])
        post_delete = source[deletion:]
        self.assertIn('/bin/kill -0 "$launch_pid"', post_delete)
        self.assertIn("launcher exited during model unload", post_delete)

    def test_checker_has_a_wall_clock_timeout_even_without_a_ros_master(self):
        source = SMOKE.read_text(encoding="utf-8")
        checker = source.index("check_bunker_runtime.py")
        timeout = source.rfind("/usr/bin/timeout", 0, checker)
        self.assertGreater(timeout, source.index("setsid roslaunch"))
        invocation = source[timeout:checker]
        self.assertIn("--signal=TERM", invocation)
        self.assertIn("--kill-after=5s", invocation)


if __name__ == "__main__":
    unittest.main()
