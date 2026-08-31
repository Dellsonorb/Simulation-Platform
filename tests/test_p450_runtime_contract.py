import copy
import json
import os
import shutil
import stat
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNTIME_MODULE = ROOT / "tools/p450_runtime.py"
CONFIG_PATH = ROOT / "config/p450_runtime.json"
P450_ENV_WRAPPER = ROOT / "scripts/with_p450_env.bash"
PLATFORM = ROOT / "src/platform/sim_platform_bringup"
PX4_NODE_WRAPPER = PLATFORM / "scripts/px4_sitl_node.bash"

if RUNTIME_MODULE.is_file():
    from tools.p450_runtime import (
        RuntimeConfigError,
        RuntimeValidationError,
        load_runtime_config,
        validate_px4_checkout,
    )
    RUNTIME_MODULE_MISSING = False
else:
    RUNTIME_MODULE_MISSING = True

EXPECTED_PX4_COMMIT = "713814f4eea5990e49dd776a38a36ad53e171f60"
EXPECTED_SITL_GAZEBO_COMMIT = (
    "9566172e7c0760f66681304b963d675c5b120daf"
)
EXPECTED_PLUGINS = (
    "libgazebo_barometer_plugin.so",
    "libgazebo_gps_plugin.so",
    "libgazebo_groundtruth_plugin.so",
    "libgazebo_imu_plugin.so",
    "libgazebo_magnetometer_plugin.so",
    "libgazebo_mavlink_interface.so",
    "libgazebo_motor_model.so",
    "libgazebo_multirotor_base_plugin.so",
)
EXPECTED_SUPPORT_LIBRARIES = (
    "libmav_msgs.so",
    "libnav_msgs.so",
    "libphysics_msgs.so",
    "libsensor_msgs.so",
    "libstd_msgs.so",
)
EXPECTED_LOCAL_MODEL_ROOTS = (
    "gazebo_models/uav_models",
    "gazebo_models/sensor_models",
    "gazebo_models/scene_models",
    "gazebo_models/r200_models",
    "gazebo_models/texture",
)
EXPECTED_REQUIRED_PACKAGES = (
    "prometheus_msgs",
    "realsense_ros_gazebo",
    "prometheus_gazebo",
    "prometheus_uav_control",
    "brick_aerial_perception",
)


def _run(command, cwd):
    return subprocess.run(
        list(command),
        cwd=str(cwd),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=True,
    ).stdout.strip()


def _git_init(path):
    path.mkdir(parents=True, exist_ok=True)
    _run(("/usr/bin/git", "init", "-q"), path)


def _git_commit_all(path, message):
    _run(("/usr/bin/git", "add", "-A"), path)
    _run((
        "/usr/bin/git",
        "-c", "user.name=P450 Runtime Test",
        "-c", "user.email=p450-runtime@example.invalid",
        "commit", "-q", "-m", message,
    ), path)
    return _run(("/usr/bin/git", "rev-parse", "HEAD"), path)


class FakePx4Checkout:
    def __init__(self, root):
        self.root = Path(root)
        self.sitl = self.root / "Tools/sitl_gazebo"
        _git_init(self.sitl)
        self._write("Tools/sitl_gazebo/scripts/jinja_gen.py", "# fake\n")
        self._write("Tools/sitl_gazebo/models/gps/model.config", "fake\n")
        self._write("Tools/sitl_gazebo/models/gps/gps.sdf", "<sdf/>\n")
        self.sitl_commit = _git_commit_all(self.sitl, "fake sitl gazebo")

        _git_init(self.root)
        self._write("README.test", "fake PX4 checkout\n")
        _run(("/usr/bin/git", "add", "README.test"), self.root)
        _run((
            "/usr/bin/git", "update-index", "--add", "--cacheinfo",
            "160000,%s,Tools/sitl_gazebo" % self.sitl_commit,
        ), self.root)
        _run((
            "/usr/bin/git",
            "-c", "user.name=P450 Runtime Test",
            "-c", "user.email=p450-runtime@example.invalid",
            "commit", "-q", "-m", "fake PX4",
        ), self.root)
        self.px4_commit = _run(
            ("/usr/bin/git", "rev-parse", "HEAD"), self.root)

        self._write("build/amovlab_sitl_default/bin/px4", "#!/bin/bash\n")
        (self.root / "build/amovlab_sitl_default/bin/px4").chmod(0o755)
        self._write("ROMFS/px4fmu_common/init.d-posix/rcS", "# fake\n")
        plugin_root = self.root / "build/amovlab_sitl_default/build_gazebo"
        for name in EXPECTED_PLUGINS + EXPECTED_SUPPORT_LIBRARIES:
            self._write(
                (plugin_root / name).relative_to(self.root).as_posix(),
                "fake shared object\n",
            )

    def _write(self, relative, content):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        return path

    def payload(self):
        return {
            "schema_version": 1,
            "ros": {
                "distribution": "noetic",
                "root": "/opt/ros/noetic",
                "install_space": "install/p450-clean",
                "required_packages": list(EXPECTED_REQUIRED_PACKAGES),
            },
            "px4": {
                "commit": self.px4_commit,
                "sitl_gazebo": {
                    "path": "Tools/sitl_gazebo",
                    "commit": self.sitl_commit,
                },
                "artifacts": {
                    "binary": "build/amovlab_sitl_default/bin/px4",
                    "romfs": "ROMFS/px4fmu_common",
                    "startup_script": (
                        "ROMFS/px4fmu_common/init.d-posix/rcS"
                    ),
                    "jinja_generator": (
                        "Tools/sitl_gazebo/scripts/jinja_gen.py"
                    ),
                    "model_root": "Tools/sitl_gazebo/models",
                    "required_model_files": [
                        "Tools/sitl_gazebo/models/gps/model.config",
                        "Tools/sitl_gazebo/models/gps/gps.sdf",
                    ],
                    "plugin_root": (
                        "build/amovlab_sitl_default/build_gazebo"
                    ),
                    "plugins": list(EXPECTED_PLUGINS),
                    "support_libraries": list(EXPECTED_SUPPORT_LIBRARIES),
                },
            },
            "gazebo": {
                "local_model_roots": list(EXPECTED_LOCAL_MODEL_ROOTS),
            },
        }


class RuntimeModulePresenceTest(unittest.TestCase):
    def test_runtime_validator_is_materialized(self):
        self.assertTrue(RUNTIME_MODULE.is_file(), RUNTIME_MODULE.as_posix())


@unittest.skipIf(RUNTIME_MODULE_MISSING, "runtime validator not materialized")
class RuntimeConfigContractTest(unittest.TestCase):
    def test_real_config_is_exact_and_machine_path_free(self):
        config = load_runtime_config(CONFIG_PATH)
        self.assertEqual(1, config.schema_version)
        self.assertEqual("noetic", config.ros_distribution)
        self.assertEqual("/opt/ros/noetic", config.ros_root)
        self.assertEqual("install/p450-clean", config.install_space)
        self.assertEqual(EXPECTED_REQUIRED_PACKAGES, config.required_packages)
        self.assertEqual(EXPECTED_PX4_COMMIT, config.px4_commit)
        self.assertEqual("Tools/sitl_gazebo", config.sitl_gazebo_path)
        self.assertEqual(
            EXPECTED_SITL_GAZEBO_COMMIT, config.sitl_gazebo_commit)
        self.assertEqual(
            "build/amovlab_sitl_default/bin/px4", config.binary)
        self.assertEqual("ROMFS/px4fmu_common", config.romfs)
        self.assertEqual(
            "ROMFS/px4fmu_common/init.d-posix/rcS",
            config.startup_script,
        )
        self.assertEqual(
            "Tools/sitl_gazebo/scripts/jinja_gen.py",
            config.jinja_generator,
        )
        self.assertEqual("Tools/sitl_gazebo/models", config.model_root)
        self.assertEqual(
            (
                "Tools/sitl_gazebo/models/gps/model.config",
                "Tools/sitl_gazebo/models/gps/gps.sdf",
            ),
            config.required_model_files,
        )
        self.assertEqual(
            "build/amovlab_sitl_default/build_gazebo", config.plugin_root)
        self.assertEqual(EXPECTED_PLUGINS, config.plugins)
        self.assertEqual(
            EXPECTED_SUPPORT_LIBRARIES, config.support_libraries)
        self.assertEqual(EXPECTED_LOCAL_MODEL_ROOTS, config.local_model_roots)

        raw = CONFIG_PATH.read_text(encoding="utf-8")
        for forbidden in (
            "/home/", "/media/", "P450-PAPER", "build/p450-clean",
        ):
            self.assertNotIn(forbidden, raw)

    def test_schema_rejects_unknown_keys_unsafe_paths_and_duplicates(self):
        with tempfile.TemporaryDirectory() as temporary:
            fake = FakePx4Checkout(Path(temporary) / "px4")
            baseline = fake.payload()
            cases = []

            unknown = copy.deepcopy(baseline)
            unknown["paper_evidence"] = {}
            cases.append(unknown)

            unsafe = copy.deepcopy(baseline)
            unsafe["px4"]["artifacts"]["binary"] = "../outside/px4"
            cases.append(unsafe)

            absolute = copy.deepcopy(baseline)
            absolute["px4"]["artifacts"]["model_root"] = "/tmp/models"
            cases.append(absolute)

            duplicate = copy.deepcopy(baseline)
            duplicate["px4"]["artifacts"]["plugins"].append(
                duplicate["px4"]["artifacts"]["plugins"][0])
            cases.append(duplicate)

            generated_dependency = copy.deepcopy(baseline)
            generated_dependency["px4"]["artifacts"]["plugin_root"] = (
                "build/p450-clean/external-px4")
            cases.append(generated_dependency)

            for index, payload in enumerate(cases):
                with self.subTest(index=index):
                    path = Path(temporary) / ("invalid-%d.json" % index)
                    path.write_text(json.dumps(payload), encoding="utf-8")
                    with self.assertRaises(RuntimeConfigError):
                        load_runtime_config(path)


@unittest.skipIf(RUNTIME_MODULE_MISSING, "runtime validator not materialized")
class Px4CheckoutValidationTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.checkout = FakePx4Checkout(self.root / "external-px4")
        self.config_path = self.root / "runtime.json"
        self.config_path.write_text(
            json.dumps(self.checkout.payload()), encoding="utf-8")
        self.config = load_runtime_config(self.config_path)

    @staticmethod
    def _clean_ldd(_path, _environment):
        return "libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6"

    def test_fake_checkout_resolves_only_pinned_artifacts(self):
        resolved = validate_px4_checkout(
            self.config,
            self.checkout.root,
            repository_root=self.root / "repository",
            ldd_runner=self._clean_ldd,
        )
        self.assertEqual(self.checkout.root.resolve(), resolved.root)
        self.assertEqual(
            self.checkout.root / self.config.binary, resolved.binary)
        self.assertEqual(
            self.checkout.root / self.config.model_root,
            resolved.model_root,
        )
        self.assertEqual(
            self.checkout.root / self.config.plugin_root,
            resolved.plugin_root,
        )
        self.assertEqual(
            tuple(resolved.plugin_root / name for name in EXPECTED_PLUGINS),
            resolved.plugins,
        )

    def test_wrong_commits_dirty_trees_and_missing_artifacts_fail(self):
        cases = []

        wrong_commit = copy.deepcopy(self.checkout.payload())
        wrong_commit["px4"]["commit"] = "0" * 40
        cases.append(("wrong-commit", wrong_commit, None))

        wrong_submodule = copy.deepcopy(self.checkout.payload())
        wrong_submodule["px4"]["sitl_gazebo"]["commit"] = "1" * 40
        cases.append(("wrong-submodule", wrong_submodule, None))

        for name, payload, _mutation in cases:
            with self.subTest(name=name):
                path = self.root / (name + ".json")
                path.write_text(json.dumps(payload), encoding="utf-8")
                with self.assertRaises(RuntimeValidationError):
                    validate_px4_checkout(
                        load_runtime_config(path),
                        self.checkout.root,
                        repository_root=self.root / "repository",
                        ldd_runner=self._clean_ldd,
                    )

        tracked = self.checkout.root / "README.test"
        original = tracked.read_text(encoding="utf-8")
        tracked.write_text("dirty\n", encoding="utf-8")
        with self.assertRaises(RuntimeValidationError):
            validate_px4_checkout(
                self.config, self.checkout.root,
                repository_root=self.root / "repository",
                ldd_runner=self._clean_ldd)
        tracked.write_text(original, encoding="utf-8")

        nested = self.checkout.sitl / "scripts/jinja_gen.py"
        original = nested.read_text(encoding="utf-8")
        nested.write_text("dirty\n", encoding="utf-8")
        with self.assertRaises(RuntimeValidationError):
            validate_px4_checkout(
                self.config, self.checkout.root,
                repository_root=self.root / "repository",
                ldd_runner=self._clean_ldd)
        nested.write_text(original, encoding="utf-8")

        missing = self.checkout.root / self.config.startup_script
        missing.unlink()
        with self.assertRaises(RuntimeValidationError):
            validate_px4_checkout(
                self.config, self.checkout.root,
                repository_root=self.root / "repository",
                ldd_runner=self._clean_ldd)

    def test_checkout_inside_repository_and_unresolved_plugin_fail(self):
        with self.assertRaises(RuntimeValidationError):
            validate_px4_checkout(
                self.config,
                self.checkout.root,
                repository_root=self.root,
                ldd_runner=self._clean_ldd,
            )

        def missing_library(_path, _environment):
            return "libmissing.so => not found"

        with self.assertRaises(RuntimeValidationError):
            validate_px4_checkout(
                self.config,
                self.checkout.root,
                repository_root=self.root / "repository",
                ldd_runner=missing_library,
            )

    def test_checkout_root_rejects_path_list_delimiters(self):
        checkout = FakePx4Checkout(self.root / "external:px4")
        config_path = self.root / "colon-runtime.json"
        config_path.write_text(
            json.dumps(checkout.payload()), encoding="utf-8")
        with self.assertRaises(RuntimeValidationError):
            validate_px4_checkout(
                load_runtime_config(config_path),
                checkout.root,
                repository_root=self.root / "repository",
                ldd_runner=self._clean_ldd,
            )

    def test_support_library_resolution_allows_spaces_in_checkout_root(self):
        checkout = FakePx4Checkout(self.root / "external px4")
        config_path = self.root / "space-runtime.json"
        config_path.write_text(
            json.dumps(checkout.payload()), encoding="utf-8")

        def support_ldd(_path, environment):
            return "libmav_msgs.so => %s/libmav_msgs.so (0x1234)" % (
                environment["LD_LIBRARY_PATH"])

        try:
            resolved = validate_px4_checkout(
                load_runtime_config(config_path),
                checkout.root,
                repository_root=self.root / "repository",
                ldd_runner=support_ldd,
            )
        except RuntimeValidationError as error:
            self.fail(str(error))
        self.assertEqual(checkout.root.resolve(), resolved.root)


class RuntimeWrapperContractTest(unittest.TestCase):
    def _make_wrapper_fixture(self, root):
        runtime_root = root / "runtime repo"
        checkout = FakePx4Checkout(root / "external px4")
        for name in EXPECTED_PLUGINS + EXPECTED_SUPPORT_LIBRARIES:
            shutil.copy2(
                "/bin/true",
                checkout.root /
                "build/amovlab_sitl_default/build_gazebo" / name,
            )

        for relative in (
            "scripts/with_p450_env.bash",
            "scripts/with_noetic_env.bash",
            "tools/p450_runtime.py",
        ):
            destination = runtime_root / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / relative, destination)

        config_path = runtime_root / "config/p450_runtime.json"
        config_path.parent.mkdir(parents=True)
        config_path.write_text(
            json.dumps(checkout.payload()), encoding="utf-8")

        install_root = runtime_root / "install/p450-clean"
        package_root = install_root / "share/prometheus_gazebo"
        for relative in EXPECTED_LOCAL_MODEL_ROOTS:
            (package_root / relative).mkdir(parents=True)
        for name in EXPECTED_REQUIRED_PACKAGES:
            (install_root / "share" / name).mkdir(parents=True, exist_ok=True)
        setup = install_root / "setup.bash"
        setup.write_text(
            "export CMAKE_PREFIX_PATH='%s:/opt/ros/noetic'\n"
            "export ROS_PACKAGE_PATH='%s/share:/opt/ros/noetic/share'\n"
            "export PYTHONPATH='/opt/ros/noetic/lib/python3/dist-packages'\n"
            "export LD_LIBRARY_PATH='%s/lib:/opt/ros/noetic/lib'\n" % (
                install_root, install_root, install_root),
            encoding="utf-8",
        )

        probe = root / "probe.py"
        probe.write_text(
            "import json, os, sys\n"
            "print(json.dumps({'argv': sys.argv[1:], "
            "'environment': dict(os.environ)}, sort_keys=True))\n",
            encoding="utf-8",
        )
        return runtime_root, checkout, probe

    def test_platform_package_installs_exact_px4_node_wrapper(self):
        self.assertTrue((PLATFORM / "package.xml").is_file())
        package = ET.parse(str(PLATFORM / "package.xml")).getroot()
        self.assertEqual("sim_platform_bringup", package.findtext("name"))
        cmake = (PLATFORM / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/px4_sitl_node.bash", cmake)
        self.assertIn("CATKIN_PACKAGE_BIN_DESTINATION", cmake)

        self.assertTrue(PX4_NODE_WRAPPER.is_file())
        self.assertFalse(PX4_NODE_WRAPPER.is_symlink())
        self.assertTrue(PX4_NODE_WRAPPER.stat().st_mode & stat.S_IXUSR)
        text = PX4_NODE_WRAPPER.read_text(encoding="utf-8")
        self.assertIn('P450_PX4_ROOT', text)
        self.assertIn('build/amovlab_sitl_default/bin/px4', text)
        self.assertNotIn("rosrun px4", text)
        self.assertNotIn("/home/", text)
        self.assertNotIn("P450-PAPER", text)

    def test_px4_node_wrapper_preserves_binary_arguments(self):
        self.assertTrue(PX4_NODE_WRAPPER.is_file(), PX4_NODE_WRAPPER.as_posix())
        with tempfile.TemporaryDirectory() as temporary:
            px4_root = Path(temporary) / "px4 root"
            binary = px4_root / "build/amovlab_sitl_default/bin/px4"
            binary.parent.mkdir(parents=True)
            binary.write_text(
                "#!/bin/bash\nprintf '<%s>\\n' \"$@\"\n",
                encoding="utf-8",
            )
            binary.chmod(0o755)
            result = subprocess.run(
                [str(PX4_NODE_WRAPPER), "--help", "two words"],
                env={
                    "P450_PX4_ROOT": str(px4_root.resolve()),
                    "PATH": "/usr/bin:/bin",
                },
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
            )
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual("<--help>\n<two words>\n", result.stdout)

    def test_runtime_wrapper_has_clean_explicit_external_boundary(self):
        self.assertTrue(P450_ENV_WRAPPER.is_file())
        self.assertFalse(P450_ENV_WRAPPER.is_symlink())
        self.assertTrue(P450_ENV_WRAPPER.stat().st_mode & stat.S_IXUSR)
        text = P450_ENV_WRAPPER.read_text(encoding="utf-8")
        for token in (
            "P450_PX4_ROOT",
            "config/p450_runtime.json",
            "scripts/with_noetic_env.bash",
            "install/p450-clean/setup.bash",
            "GAZEBO_MODEL_PATH",
            "GAZEBO_PLUGIN_PATH",
            "LD_LIBRARY_PATH",
            "ROS_PACKAGE_PATH",
        ):
            self.assertIn(token, text)
        self.assertNotIn("/home/", text)
        self.assertNotIn("P450-PAPER", text)

        result = subprocess.run(
            [str(P450_ENV_WRAPPER), "/usr/bin/true"],
            env={"PATH": "/usr/bin:/bin"},
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        self.assertNotEqual(0, result.returncode)
        self.assertIn("P450_PX4_ROOT", result.stderr)

    def test_runtime_wrapper_cleans_environment_and_preserves_arguments(self):
        self.assertTrue(P450_ENV_WRAPPER.is_file(), P450_ENV_WRAPPER.as_posix())
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runtime_root, checkout, probe = self._make_wrapper_fixture(root)
            checkout_link = root / "px4 link"
            checkout_link.symlink_to(checkout.root, target_is_directory=True)
            wrapper = runtime_root / "scripts/with_p450_env.bash"
            before_parent = _run(
                ("/usr/bin/git", "status", "--porcelain=v1", "--untracked-files=all"),
                checkout.root,
            )
            before_child = _run(
                ("/usr/bin/git", "status", "--porcelain=v1", "--untracked-files=all"),
                checkout.sitl,
            )
            result = subprocess.run(
                [
                    str(wrapper), "/usr/bin/python3", str(probe),
                    "--help", "two words",
                ],
                env={
                    "PATH": "/poison:/usr/bin:/bin",
                    "HOME": "/poison/home",
                    "CMAKE_PREFIX_PATH": "/poison/cmake",
                    "ROS_PACKAGE_PATH": "/poison/ros",
                    "PYTHONPATH": "/poison/python",
                    "LD_LIBRARY_PATH": "/poison/library",
                    "GAZEBO_MODEL_PATH": "/poison/models",
                    "GAZEBO_PLUGIN_PATH": "/poison/plugins",
                    "DISPLAY": ":99",
                    "P450_PX4_ROOT": str(checkout_link),
                },
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
            )
            after_parent = _run(
                ("/usr/bin/git", "status", "--porcelain=v1", "--untracked-files=all"),
                checkout.root,
            )
            after_child = _run(
                ("/usr/bin/git", "status", "--porcelain=v1", "--untracked-files=all"),
                checkout.sitl,
            )

        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual(before_parent, after_parent)
        self.assertEqual(before_child, after_child)
        payload = json.loads(result.stdout)
        self.assertEqual(["--help", "two words"], payload["argv"])
        environment = payload["environment"]
        self.assertEqual(str(checkout.root.resolve()), environment["P450_PX4_ROOT"])
        self.assertNotIn("DISPLAY", environment)
        self.assertNotIn("/poison", "\n".join(environment.values()))
        self.assertEqual(
            str(runtime_root / "logs/ros"), environment["ROS_HOME"])

        install_models = runtime_root / (
            "install/p450-clean/share/prometheus_gazebo")
        expected_models = [
            str(install_models / relative)
            for relative in EXPECTED_LOCAL_MODEL_ROOTS
        ] + [str(checkout.root / "Tools/sitl_gazebo/models")]
        self.assertEqual(
            ":".join(expected_models), environment["GAZEBO_MODEL_PATH"])
        plugin_root = checkout.root / (
            "build/amovlab_sitl_default/build_gazebo")
        self.assertEqual(str(plugin_root), environment["GAZEBO_PLUGIN_PATH"])
        self.assertEqual(
            str(plugin_root), environment["LD_LIBRARY_PATH"].split(":")[0])
        self.assertEqual(
            "%s/share:/opt/ros/noetic/share:%s:%s" % (
                runtime_root / "install/p450-clean",
                checkout.root,
                checkout.sitl,
            ),
            environment["ROS_PACKAGE_PATH"],
        )

    def test_runtime_wrapper_preserves_command_exit_status(self):
        self.assertTrue(P450_ENV_WRAPPER.is_file(), P450_ENV_WRAPPER.as_posix())
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runtime_root, checkout, _probe = self._make_wrapper_fixture(root)
            result = subprocess.run(
                [
                    str(runtime_root / "scripts/with_p450_env.bash"),
                    "/bin/bash", "--noprofile", "--norc", "-c", "exit 37",
                ],
                env={
                    "PATH": "/usr/bin:/bin",
                    "P450_PX4_ROOT": str(checkout.root),
                },
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
            )
        self.assertEqual(37, result.returncode, result.stderr)


if __name__ == "__main__":
    unittest.main()
