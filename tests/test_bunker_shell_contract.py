import json
import os
import shutil
import stat
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WRAPPER = ROOT / "scripts/with_bunker_env.bash"
VALIDATOR = ROOT / "scripts/validate_bunker_install.bash"
RUN_ROOT = ROOT / "logs/bunker_standalone"
TEMP_ROOT = RUN_ROOT / "engineering-tmp"
STATE_NAMES = (
    "tmp", "ros-home", "ros-log", "gazebo-log", "ign-fuel-cache",
    "xdg-cache", "xdg-config", "xdg-data",
)
PATH_LABELS = (
    "ROS_PACKAGE_PATH", "CMAKE_PREFIX_PATH", "PYTHONPATH",
    "LD_LIBRARY_PATH", "GAZEBO_PLUGIN_PATH", "GAZEBO_MODEL_PATH",
    "GAZEBO_RESOURCE_PATH", "PKG_CONFIG_PATH", "OGRE_RESOURCE_PATH",
)
PLUGIN_FILES = (
    "/opt/ros/noetic/lib/libgazebo_ros_planar_move.so",
    "/opt/ros/noetic/lib/libgazebo_ros_laser.so",
    "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libRayPlugin.so",
)


class BunkerShellContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        RUN_ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
        TEMP_ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
        RUN_ROOT.chmod(0o700)
        TEMP_ROOT.chmod(0o700)

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(dir=str(TEMP_ROOT))
        self.addCleanup(self.temporary.cleanup)
        self.fixture_root = Path(self.temporary.name)

    def _run(self, *arguments):
        return subprocess.run(
            [str(WRAPPER)] + [str(item) for item in arguments],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env={
                "PATH": "/usr/bin:/bin",
                "LANG": "C.UTF-8",
                "LC_ALL": "C.UTF-8",
            },
            check=False,
        )

    def _run_validator(self, arguments, environment):
        return subprocess.run(
            [str(VALIDATOR)] + list(arguments),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=environment,
            check=False,
        )

    def _validator_environment(self, run_dir):
        install = str(ROOT / "install/p450-clean")
        state = {
            "TMPDIR": "tmp", "ROS_HOME": "ros-home",
            "ROS_LOG_DIR": "ros-log", "GAZEBO_LOG_PATH": "gazebo-log",
            "IGN_FUEL_CACHE_PATH": "ign-fuel-cache",
            "XDG_CACHE_HOME": "xdg-cache",
            "XDG_CONFIG_HOME": "xdg-config",
            "XDG_DATA_HOME": "xdg-data",
        }
        for name in state.values():
            path = run_dir / name
            path.mkdir(mode=0o700)
        environment = {
            "HOME": os.environ.get("HOME", "/home/lu"),
            "USER": os.environ.get("USER", "lu"),
            "LOGNAME": os.environ.get("LOGNAME", "lu"),
            "SHELL": "/bin/bash",
            "BUNKER_REPO_ROOT": str(ROOT),
            "BUNKER_RUN_DIR": str(run_dir),
            "PATH": "/opt/ros/noetic/bin:/usr/bin:/bin",
            "LANG": "C.UTF-8", "LC_ALL": "C.UTF-8",
            "ROS_PACKAGE_PATH": install + "/share:/opt/ros/noetic/share",
            "CMAKE_PREFIX_PATH": install + ":/opt/ros/noetic",
            "PYTHONPATH": (
                install + "/lib/python3/dist-packages:"
                "/opt/ros/noetic/lib/python3/dist-packages"),
            "LD_LIBRARY_PATH": (
                install + "/lib:/opt/ros/noetic/lib:"
                "/opt/ros/noetic/lib/x86_64-linux-gnu:"
                "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"),
            "GAZEBO_PLUGIN_PATH": (
                "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:"
                "/opt/ros/noetic/lib"),
            "GAZEBO_MODEL_PATH": "/usr/share/gazebo-11/models",
            "GAZEBO_RESOURCE_PATH": "/usr/share/gazebo-11",
            "PKG_CONFIG_PATH": (
                install + "/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig:"
                "/opt/ros/noetic/lib/x86_64-linux-gnu/pkgconfig"),
            "OGRE_RESOURCE_PATH": "/usr/lib/x86_64-linux-gnu/OGRE-1.9.0",
            "ROS_ETC_DIR": "/opt/ros/noetic/etc/ros",
            "ROS_ROOT": "/opt/ros/noetic/share/ros",
            "ROSLISP_PACKAGE_DIRECTORIES": "",
            "GAZEBO_MODEL_DATABASE_URI": "",
        }
        environment.update({
            variable: str(run_dir / name) for variable, name in state.items()})
        return environment

    def _fresh_run_dir(self):
        return Path(tempfile.mkdtemp(prefix="shell-run-", dir=str(RUN_ROOT)))

    def test_run_dir_mode_accepts_only_fresh_canonical_private_descendants(self):
        accepted = self._fresh_run_dir()
        self.addCleanup(accepted.rmdir)
        accepted.chmod(0o700)
        result = self._run("--test-run-dir", accepted)
        self.assertEqual(0, result.returncode, result.stderr.decode())
        self.assertEqual(str(accepted.resolve()) + "\n", result.stdout.decode())
        self.assertEqual([], list(accepted.iterdir()))

        noncanonical = str(accepted.parent / "." / accepted.name / ".." /
                           accepted.name)
        outside = Path(tempfile.mkdtemp(
            prefix="bunker-shell-outside-", dir=str(ROOT / "logs")))
        self.addCleanup(shutil.rmtree, outside)
        outside.chmod(0o700)
        missing = accepted.parent / (accepted.name + "-missing")
        symlink = self.fixture_root / "run-link"
        symlink.symlink_to(accepted, target_is_directory=True)
        rejected = (noncanonical, outside, missing, symlink, ROOT, RUN_ROOT)
        for path in rejected:
            with self.subTest(path=path):
                self.assertNotEqual(
                    0, self._run("--test-run-dir", path).returncode)

        accepted.chmod(0o755)
        self.assertNotEqual(
            0, self._run("--test-run-dir", accepted).returncode)
        accepted.chmod(0o700)

    def test_path_list_mode_accepts_each_known_label_and_canonical_roots(self):
        run_dir = self._fresh_run_dir()
        self.addCleanup(shutil.rmtree, run_dir)
        run_dir.chmod(0o700)
        local = run_dir / "fixture"
        local.mkdir(mode=0o700)
        value = "/opt:/usr:%s" % local
        for label in PATH_LABELS:
            with self.subTest(label=label):
                result = self._run(
                    "--test-path-list", label, value, run_dir)
                self.assertEqual(0, result.returncode, result.stderr.decode())
                self.assertEqual(value + "\n", result.stdout.decode())

    def test_path_list_mode_rejects_unsafe_components(self):
        run_dir = self._fresh_run_dir()
        self.addCleanup(shutil.rmtree, run_dir)
        run_dir.chmod(0o700)
        allowed = run_dir / "allowed"
        allowed.mkdir(mode=0o700)
        symlink = run_dir / "alias"
        symlink.symlink_to(allowed, target_is_directory=True)
        unreadable = run_dir / "unreadable"
        unreadable.mkdir(mode=0o700)
        unreadable.chmod(0o000)
        self.addCleanup(unreadable.chmod, 0o700)
        generated = []
        for name in ("source", "devel", "build"):
            path = run_dir / name / "fixture"
            path.mkdir(mode=0o700, parents=True)
            generated.append(path)
        invalid = (
            "", ":/opt", "/opt:", "/opt::/usr", "relative",
            "/opt:/opt", str(symlink), str(unreadable),
            str(run_dir / "missing"), str(ROOT / "src"),
            *(str(path) for path in generated),
        )
        for value in invalid:
            with self.subTest(value=value):
                self.assertNotEqual(0, self._run(
                    "--test-path-list", "PYTHONPATH", value,
                    run_dir).returncode)
        self.assertNotEqual(0, self._run(
            "--test-path-list", "UNKNOWN_PATH", "/opt", run_dir).returncode)

    def test_fixed_environment_mode_is_literal_and_fail_closed(self):
        accepted = (
            "/opt/ros/noetic/bin:/usr/bin:/bin",
            "C.UTF-8", "C.UTF-8",
            "/opt/ros/noetic/etc/ros",
            "/opt/ros/noetic/share/ros",
            "",
        )
        self.assertEqual(
            0, self._run("--test-fixed-environment", *accepted).returncode)
        for index in range(len(accepted)):
            altered = list(accepted)
            altered[index] = "changed"
            with self.subTest(index=index):
                self.assertNotEqual(0, self._run(
                    "--test-fixed-environment", *altered).returncode)

    def test_plugin_candidates_are_exact_deduplicated_and_unshadowed(self):
        gazebo = (
            "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:"
            "/opt/ros/noetic/lib"
        )
        library = (
            "/opt/ros/noetic/lib:"
            "/opt/ros/noetic/lib/x86_64-linux-gnu:"
            "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
        )
        result = self._run("--test-plugin-candidates", gazebo, library)
        self.assertEqual(0, result.returncode, result.stderr.decode())
        self.assertEqual(list(PLUGIN_FILES), result.stdout.decode().splitlines())

        shadow = self.fixture_root / "shadow"
        shadow.mkdir(mode=0o700)
        (shadow / "libgazebo_ros_laser.so").write_bytes(b"shadow")
        for bad_gazebo, bad_library in (
                (str(shadow) + ":" + gazebo, library),
                ("/opt/ros/noetic/lib", "/opt/ros/noetic/lib"),
                ("/opt/ros/noetic/lib/..:" + gazebo, library),
                (gazebo + ":/missing/plugin-path", library)):
            with self.subTest(gazebo=bad_gazebo, library=bad_library):
                self.assertNotEqual(0, self._run(
                    "--test-plugin-candidates", bad_gazebo,
                    bad_library).returncode)

    def test_install_validator_environment_branch_is_exact_and_nonmutating(self):
        run_dir = self._fresh_run_dir()
        self.addCleanup(shutil.rmtree, run_dir)
        run_dir.chmod(0o700)
        environment = self._validator_environment(run_dir)
        before = sorted(path.name for path in run_dir.iterdir())
        result = self._run_validator(("--test-environment",), environment)
        self.assertEqual(0, result.returncode, result.stderr.decode())
        self.assertEqual(before, sorted(path.name for path in run_dir.iterdir()))
        self.assertEqual(b"", result.stdout)

        mutations = []
        missing = dict(environment)
        missing.pop("ROS_ROOT")
        mutations.append(missing)
        source = dict(environment)
        source["PYTHONPATH"] = str(ROOT / "src") + ":" + source["PYTHONPATH"]
        mutations.append(source)
        shadowed = dict(environment)
        shadowed["GAZEBO_PLUGIN_PATH"] = (
            str(self.fixture_root) + ":" + shadowed["GAZEBO_PLUGIN_PATH"])
        mutations.append(shadowed)
        for altered in mutations:
            with self.subTest(altered=altered):
                self.assertNotEqual(0, self._run_validator(
                    ("--test-environment",), altered).returncode)

    def test_install_validator_plugin_ldd_branch_is_exact(self):
        environment = {
            "PATH": "/opt/ros/noetic/bin:/usr/bin:/bin",
            "LANG": "C.UTF-8", "LC_ALL": "C.UTF-8",
            "LD_LIBRARY_PATH": (
                "/opt/ros/noetic/lib:"
                "/opt/ros/noetic/lib/x86_64-linux-gnu:"
                "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"),
        }
        result = self._run_validator(
            ("--test-plugin-ldd",) + PLUGIN_FILES, environment)
        self.assertEqual(0, result.returncode, result.stderr.decode())
        self.assertEqual(
            [path + "\tPASS" for path in PLUGIN_FILES],
            result.stdout.decode().splitlines(),
        )
        for arguments in (
                ("--test-plugin-ldd",),
                ("--test-plugin-ldd",) + PLUGIN_FILES[:-1],
                ("--test-plugin-ldd", str(ROOT / "src")) + PLUGIN_FILES[1:],
                ("--unknown",)):
            with self.subTest(arguments=arguments):
                self.assertNotEqual(0, self._run_validator(
                    arguments, environment).returncode)
        shadow = self.fixture_root / "plugin-shadow"
        shadow.mkdir(mode=0o700)
        (shadow / "libRayPlugin.so").write_bytes(b"shadow")
        shadowed = dict(environment)
        shadowed["LD_LIBRARY_PATH"] = (
            str(shadow) + ":" + environment["LD_LIBRARY_PATH"])
        self.assertNotEqual(0, self._run_validator(
            ("--test-plugin-ldd",) + PLUGIN_FILES, shadowed).returncode)

    def test_wrong_interfaces_fail_before_creating_state(self):
        run_dir = self._fresh_run_dir()
        self.addCleanup(run_dir.rmdir)
        run_dir.chmod(0o700)
        for arguments in (
                (), ("--test-run-dir",),
                ("--test-path-list", "PYTHONPATH", "/opt"),
                ("--test-fixed-environment", "too", "few"),
                ("--test-plugin-candidates", "/opt"),
                ("--run-dir", run_dir),
                ("--run-dir", run_dir, "--"),
                ("--run-dir", run_dir, "/usr/bin/true"),
                ("--unknown",)):
            with self.subTest(arguments=arguments):
                self.assertNotEqual(0, self._run(*arguments).returncode)
                self.assertEqual([], list(run_dir.iterdir()))

    def test_normal_interface_sanitizes_state_and_preserves_command_argv(self):
        run_dir = self._fresh_run_dir()
        self.addCleanup(shutil.rmtree, run_dir)
        run_dir.chmod(0o700)
        program = (
            "import json,os,sys; print(json.dumps({"
            "'argv':sys.argv[1:],"
            "'environment':{key:os.environ.get(key) for key in "
            "('BUNKER_REPO_ROOT','BUNKER_RUN_DIR','TMPDIR','ROS_HOME',"
            "'ROS_LOG_DIR','GAZEBO_LOG_PATH','IGN_FUEL_CACHE_PATH',"
            "'XDG_CACHE_HOME','XDG_CONFIG_HOME','XDG_DATA_HOME',"
            "'ROS_PACKAGE_PATH','CMAKE_PREFIX_PATH','PYTHONPATH',"
            "'LD_LIBRARY_PATH','GAZEBO_PLUGIN_PATH','GAZEBO_MODEL_PATH',"
            "'GAZEBO_RESOURCE_PATH','PKG_CONFIG_PATH','OGRE_RESOURCE_PATH',"
            "'ROS_ETC_DIR','ROS_ROOT','ROSLISP_PACKAGE_DIRECTORIES',"
            "'PATH','LANG','LC_ALL')}}))"
        )
        forwarded = ["alpha beta", "", "--literal", "$HOME"]
        result = self._run(
            "--run-dir", run_dir, "--", "/usr/bin/python3", "-c",
            program, *forwarded)
        self.assertEqual(0, result.returncode, result.stderr.decode())
        payload = json.loads(result.stdout.decode())
        self.assertEqual(forwarded, payload["argv"])
        environment = payload["environment"]
        install = str(ROOT / "install/p450-clean")
        self.assertEqual(str(ROOT), environment["BUNKER_REPO_ROOT"])
        self.assertEqual(str(run_dir), environment["BUNKER_RUN_DIR"])
        expected_state = {
            "TMPDIR": "tmp", "ROS_HOME": "ros-home",
            "ROS_LOG_DIR": "ros-log", "GAZEBO_LOG_PATH": "gazebo-log",
            "IGN_FUEL_CACHE_PATH": "ign-fuel-cache",
            "XDG_CACHE_HOME": "xdg-cache",
            "XDG_CONFIG_HOME": "xdg-config",
            "XDG_DATA_HOME": "xdg-data",
        }
        for variable, name in expected_state.items():
            path = run_dir / name
            self.assertEqual(str(path), environment[variable])
            self.assertTrue(path.is_dir())
            self.assertEqual(0o700, stat.S_IMODE(path.stat().st_mode))
        self.assertEqual({
            "ROS_PACKAGE_PATH": install + "/share:/opt/ros/noetic/share",
            "CMAKE_PREFIX_PATH": install + ":/opt/ros/noetic",
            "PYTHONPATH": (
                install + "/lib/python3/dist-packages:"
                "/opt/ros/noetic/lib/python3/dist-packages"),
            "LD_LIBRARY_PATH": (
                install + "/lib:/opt/ros/noetic/lib:"
                "/opt/ros/noetic/lib/x86_64-linux-gnu:"
                "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"),
            "GAZEBO_PLUGIN_PATH": (
                "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:"
                "/opt/ros/noetic/lib"),
            "GAZEBO_MODEL_PATH": "/usr/share/gazebo-11/models",
            "GAZEBO_RESOURCE_PATH": "/usr/share/gazebo-11",
            "PKG_CONFIG_PATH": (
                install + "/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig:"
                "/opt/ros/noetic/lib/x86_64-linux-gnu/pkgconfig"),
            "OGRE_RESOURCE_PATH": "/usr/lib/x86_64-linux-gnu/OGRE-1.9.0",
            "ROS_ETC_DIR": "/opt/ros/noetic/etc/ros",
            "ROS_ROOT": "/opt/ros/noetic/share/ros",
            "ROSLISP_PACKAGE_DIRECTORIES": "",
            "PATH": "/opt/ros/noetic/bin:/usr/bin:/bin",
            "LANG": "C.UTF-8",
            "LC_ALL": "C.UTF-8",
        }, {key: environment[key] for key in (
            "ROS_PACKAGE_PATH", "CMAKE_PREFIX_PATH", "PYTHONPATH",
            "LD_LIBRARY_PATH", "GAZEBO_PLUGIN_PATH", "GAZEBO_MODEL_PATH",
            "GAZEBO_RESOURCE_PATH", "PKG_CONFIG_PATH", "OGRE_RESOURCE_PATH",
            "ROS_ETC_DIR", "ROS_ROOT", "ROSLISP_PACKAGE_DIRECTORIES",
            "PATH", "LANG", "LC_ALL")})


if __name__ == "__main__":
    unittest.main()
