import importlib.util
import io
import tempfile
import unittest
from pathlib import Path

from bunker_sim_runtime.contracts import DEFAULT_POSE, SPAWN_EXECUTABLE
from bunker_sim_runtime.sim_time import SimTimeContractError


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / (
    "src/platform/bunker_sim_runtime/scripts/spawn_bunker_preflight.py")
TEMP_ROOT = ROOT / "logs/bunker_standalone/engineering-tmp"


def _load_script():
    spec = importlib.util.spec_from_file_location(
        "spawn_bunker_preflight_test_target", str(SCRIPT))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SpawnBunkerPreflightTest(unittest.TestCase):
    def setUp(self):
        TEMP_ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(dir=str(TEMP_ROOT))
        self.addCleanup(self.temporary.cleanup)
        self.log_dir = Path(self.temporary.name) / "ros-log"
        self.log_dir.mkdir(mode=0o700)
        self.environment = {
            "ROS_LOG_DIR": str(self.log_dir),
            "ROS_MASTER_URI": "http://127.0.0.1:11311/",
        }
        self.command = [
            SPAWN_EXECUTABLE,
            "-urdf", "-param", "robot_description",
            "-model", "bunker", "-b",
            "-x", DEFAULT_POSE[0], "-y", DEFAULT_POSE[1],
            "-z", DEFAULT_POSE[2], "-R", DEFAULT_POSE[3],
            "-P", DEFAULT_POSE[4], "-Y", DEFAULT_POSE[5],
            "__name:=spawn_bunker",
            "__log:=%s" % (self.log_dir / "ground-spawn_bunker-1.log"),
        ]
        self.argv = list(DEFAULT_POSE) + ["--"] + self.command
        self.module = _load_script()

    @staticmethod
    def _sim_time(master_uri, caller_id):
        if master_uri != "http://127.0.0.1:11311/":
            raise AssertionError(master_uri)
        if caller_id != "/ground/spawn_bunker":
            raise AssertionError(caller_id)
        return True

    def test_delegates_the_exact_command_after_preflight(self):
        calls = []
        errors = io.StringIO()
        status = self.module.main(
            argv=self.argv, environ=self.environment, stderr=errors,
            sim_time_reader=self._sim_time,
            execv=lambda executable, command: calls.append(
                (executable, tuple(command))),
            isfile=lambda path: True, access=lambda path, mode: True)
        self.assertEqual(70, status)
        self.assertEqual(
            [(SPAWN_EXECUTABLE, tuple(self.command))], calls)
        self.assertIn("execv returned unexpectedly", errors.getvalue())

    def test_rejects_bad_arguments_and_missing_environment(self):
        self.assertEqual(64, self.module.main(argv=(), stderr=io.StringIO()))
        self.assertEqual(
            65,
            self.module.main(
                argv=self.argv, environ={}, stderr=io.StringIO(),
                isfile=lambda path: True, access=lambda path, mode: True),
        )

    def test_rejects_sim_time_failure(self):
        def reject(master_uri, caller_id):
            raise SimTimeContractError("not boolean true")

        self.assertEqual(
            65,
            self.module.main(
                argv=self.argv, environ=self.environment,
                stderr=io.StringIO(), sim_time_reader=reject,
                isfile=lambda path: True, access=lambda path, mode: True),
        )

    def test_rejects_unavailable_spawn_and_exec_failure(self):
        self.assertEqual(
            66,
            self.module.main(
                argv=self.argv, environ=self.environment,
                stderr=io.StringIO(), isfile=lambda path: False,
                access=lambda path, mode: False),
        )

        def fail_exec(executable, command):
            raise OSError("fixture exec failure")

        self.assertEqual(
            66,
            self.module.main(
                argv=self.argv, environ=self.environment,
                stderr=io.StringIO(), sim_time_reader=self._sim_time,
                execv=fail_exec, isfile=lambda path: True,
                access=lambda path, mode: True),
        )


if __name__ == "__main__":
    unittest.main()
