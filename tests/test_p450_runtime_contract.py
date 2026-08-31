import copy
import hashlib
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
OVERLAY_BUILD_WRAPPER = ROOT / "scripts/build_p450_runtime_overlays.bash"
PX4_GAZEBO_OVERLAY_PROJECT = (
    ROOT / "runtime_overlays/px4_gazebo_plugins"
)
PLATFORM = ROOT / "src/platform/sim_platform_bringup"
PX4_NODE_WRAPPER = PLATFORM / "scripts/px4_sitl_node.bash"

if RUNTIME_MODULE.is_file():
    from tools import p450_runtime as p450_runtime_module
    from tools.p450_runtime import (
        RuntimeConfigError,
        RuntimeValidationError,
        load_runtime_config,
        validate_px4_checkout,
    )
    validate_runtime_plugin_overlay = getattr(
        p450_runtime_module, "validate_runtime_plugin_overlay", None)
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
EXPECTED_LOCAL_PLUGIN_ROOT = "install/p450-runtime-overlays/lib"
EXPECTED_GPS_PLUGIN = "libgazebo_gps_plugin.so"
EXPECTED_GPS_SOURCE = "Tools/sitl_gazebo/src/gazebo_gps_plugin.cpp"
EXPECTED_GPS_SOURCE_SHA256 = (
    "128a20dc01e140ebba7e7c889cce5c729aa1a013bcafd3b272ea0360f7a42c75"
)
EXPECTED_PATCHED_GPS_SOURCE_SHA256 = (
    "dc66bf5b1d19e350edcff2bcdd533c659f5836360a5f71562de41a4001ff304c"
)
EXPECTED_GPS_HEADER = "Tools/sitl_gazebo/include/gazebo_gps_plugin.h"
EXPECTED_GPS_HEADER_SHA256 = (
    "fcaa736262d89575a6084e659500cb8889d6a927c91f1c044eab5587e74f357f"
)
EXPECTED_GPS_GENERATED_HEADER = (
    "build/amovlab_sitl_default/build_gazebo/SITLGps.pb.h"
)
EXPECTED_GPS_GENERATED_HEADER_SHA256 = (
    "537727dde21637fcd8cf630da2c49f8d7004f868638c23b05939b17a5908a434"
)
EXPECTED_GROUNDTRUTH_PLUGIN = "libgazebo_groundtruth_plugin.so"
EXPECTED_GROUNDTRUTH_SOURCE = (
    "Tools/sitl_gazebo/src/gazebo_groundtruth_plugin.cpp"
)
EXPECTED_GROUNDTRUTH_SOURCE_SHA256 = (
    "8dd9d6bd2c7619f38fa0104b730c8bf5653243d80762bf26ade0eec4b94266a3"
)
EXPECTED_PATCHED_GROUNDTRUTH_SOURCE_SHA256 = (
    "7e1caae1e01dafbacd2b1e3ffb80efdfe36ace0c6b06cbeefd261d9c85d8aa97"
)
EXPECTED_GROUNDTRUTH_HEADER = (
    "Tools/sitl_gazebo/include/gazebo_groundtruth_plugin.h"
)
EXPECTED_GROUNDTRUTH_HEADER_SHA256 = (
    "aabc2cee0d9ee378aa25f9b7d62d5d7239682c129c115e97cc56f4577f39761f"
)
EXPECTED_GROUNDTRUTH_GENERATED_HEADER = (
    "build/amovlab_sitl_default/build_gazebo/Groundtruth.pb.h"
)
EXPECTED_GROUNDTRUTH_GENERATED_HEADER_SHA256 = (
    "80229d01b44dbe7bd07b36f3c24dc2bfdb5156c23fd9a573d2ce73514e1e374b"
)
EXPECTED_OVERLAY_SUPPORT_LIBRARY = (
    "build/amovlab_sitl_default/build_gazebo/libsensor_msgs.so"
)
EXPECTED_OVERLAY_SUPPORT_LIBRARY_SHA256 = (
    "203679a0bca52a1839a108ea3c66ab431004ee13a4f1e6b78775d549c74d6ce5"
)
EXPECTED_OVERLAY_COMMON_HEADER = "Tools/sitl_gazebo/include/common.h"
EXPECTED_OVERLAY_COMMON_HEADER_SHA256 = (
    "329133e49171793664184dd9c60f4f24305e280600d9a274286ca5ec2eb84241"
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
        self._write(
            "Tools/sitl_gazebo/package.xml",
            self._package_xml("mavlink_sitl_gazebo"),
        )
        self._write("Tools/sitl_gazebo/scripts/jinja_gen.py", "# fake\n")
        self._write("Tools/sitl_gazebo/models/gps/model.config", "fake\n")
        self._write("Tools/sitl_gazebo/models/gps/gps.sdf", "<sdf/>\n")
        self.overlay_hashes = {}
        for name, source_path, header_path, connection in (
            (
                "gps",
                EXPECTED_GPS_SOURCE,
                EXPECTED_GPS_HEADER,
                "updateSensorConnection_",
            ),
            (
                "groundtruth",
                EXPECTED_GROUNDTRUTH_SOURCE,
                EXPECTED_GROUNDTRUTH_HEADER,
                "updateConnection_",
            ),
        ):
            source = self._write(
                source_path,
                "void destroy_%s() {\n"
                "  %s->~Connection();\n"
                "  world_->Reset();\n"
                "}\n" % (name, connection),
            )
            header = self._write(
                header_path, "// fake %s plugin header\n" % name)
            source_bytes = source.read_bytes()
            patched_bytes = source_bytes.replace(
                ("%s->~Connection();" % connection).encode("ascii"),
                ("%s.reset();" % connection).encode("ascii"),
            ).replace(b"  world_->Reset();\n", b"")
            self.overlay_hashes[name] = {
                "source": hashlib.sha256(source_bytes).hexdigest(),
                "patched_source": hashlib.sha256(patched_bytes).hexdigest(),
                "header": hashlib.sha256(header.read_bytes()).hexdigest(),
            }
        common_header = self._write(
            EXPECTED_OVERLAY_COMMON_HEADER, "// fake common\n")
        self.common_header_sha256 = hashlib.sha256(
            common_header.read_bytes()).hexdigest()
        self.sitl_commit = _git_commit_all(self.sitl, "fake sitl gazebo")

        _git_init(self.root)
        self._write("README.test", "fake PX4 checkout\n")
        self._write("package.xml", self._package_xml("px4"))
        _run(("/usr/bin/git", "add", "README.test", "package.xml"), self.root)
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
        self._write(EXPECTED_GPS_GENERATED_HEADER, "// fake GPS protobuf\n")
        self._write(
            EXPECTED_GROUNDTRUTH_GENERATED_HEADER,
            "// fake Groundtruth protobuf\n",
        )
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

    @staticmethod
    def _package_xml(name):
        return (
            "<package format=\"2\"><name>%s</name><version>0.0.0</version>"
            "<description>fixture</description>"
            "<maintainer email=\"test@example.invalid\">Test</maintainer>"
            "<license>BSD</license></package>\n" % name
        )

    def payload(self):
        return {
            "schema_version": 2,
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
                "local_plugin_overlay": {
                    "plugin_root": EXPECTED_LOCAL_PLUGIN_ROOT,
                    "support_library": EXPECTED_OVERLAY_SUPPORT_LIBRARY,
                    "support_library_sha256": hashlib.sha256(
                        (self.root / EXPECTED_OVERLAY_SUPPORT_LIBRARY).
                        read_bytes()
                    ).hexdigest(),
                    "common_header": EXPECTED_OVERLAY_COMMON_HEADER,
                    "common_header_sha256": self.common_header_sha256,
                    "plugins": [
                        self._overlay_payload(
                            "gps",
                            EXPECTED_GPS_PLUGIN,
                            EXPECTED_GPS_SOURCE,
                            EXPECTED_GPS_HEADER,
                            EXPECTED_GPS_GENERATED_HEADER,
                        ),
                        self._overlay_payload(
                            "groundtruth",
                            EXPECTED_GROUNDTRUTH_PLUGIN,
                            EXPECTED_GROUNDTRUTH_SOURCE,
                            EXPECTED_GROUNDTRUTH_HEADER,
                            EXPECTED_GROUNDTRUTH_GENERATED_HEADER,
                        ),
                    ],
                },
            },
        }

    def _overlay_payload(
            self, name, plugin, source, header, generated_header):
        return {
            "plugin": plugin,
            "upstream_source": source,
            "upstream_source_sha256": self.overlay_hashes[name]["source"],
            "patched_source_sha256": (
                self.overlay_hashes[name]["patched_source"]),
            "upstream_header": header,
            "upstream_header_sha256": self.overlay_hashes[name]["header"],
            "generated_header": generated_header,
            "generated_header_sha256": hashlib.sha256(
                (self.root / generated_header).read_bytes()).hexdigest(),
        }


class RuntimeModulePresenceTest(unittest.TestCase):
    def test_runtime_validator_is_materialized(self):
        self.assertTrue(RUNTIME_MODULE.is_file(), RUNTIME_MODULE.as_posix())

    def test_local_plugin_overlay_validator_is_materialized(self):
        self.assertTrue(callable(validate_runtime_plugin_overlay))


class Px4GazeboOverlayBuildContractTest(unittest.TestCase):
    def test_overlay_build_is_sim_owned_hash_pinned_and_rpath_free(self):
        cmake_path = PX4_GAZEBO_OVERLAY_PROJECT / "CMakeLists.txt"
        self.assertTrue(cmake_path.is_file(), cmake_path.as_posix())
        cmake = cmake_path.read_text(encoding="utf-8")
        for token in (
            "P450_PX4_ROOT",
            EXPECTED_GPS_SOURCE,
            EXPECTED_GPS_SOURCE_SHA256,
            EXPECTED_PATCHED_GPS_SOURCE_SHA256,
            EXPECTED_GPS_HEADER,
            EXPECTED_GPS_HEADER_SHA256,
            EXPECTED_GPS_GENERATED_HEADER,
            EXPECTED_GPS_GENERATED_HEADER_SHA256,
            EXPECTED_GROUNDTRUTH_SOURCE,
            EXPECTED_GROUNDTRUTH_SOURCE_SHA256,
            EXPECTED_PATCHED_GROUNDTRUTH_SOURCE_SHA256,
            EXPECTED_GROUNDTRUTH_HEADER,
            EXPECTED_GROUNDTRUTH_HEADER_SHA256,
            EXPECTED_GROUNDTRUTH_GENERATED_HEADER,
            EXPECTED_GROUNDTRUTH_GENERATED_HEADER_SHA256,
            EXPECTED_OVERLAY_SUPPORT_LIBRARY_SHA256,
            EXPECTED_OVERLAY_COMMON_HEADER,
            EXPECTED_OVERLAY_COMMON_HEADER_SHA256,
            "world_->Reset();",
            "updateSensorConnection_.reset();",
            "updateConnection_.reset();",
            "file(SHA256",
            "configure_file",
            "P450_SNAPSHOT_ROOT",
            "P450_SNAPSHOT_INCLUDE",
            "P450_SNAPSHOT_SENSOR_MESSAGES",
            "string(REPLACE",
            "string(LENGTH",
            "CMAKE_SKIP_RPATH",
            "libsensor_msgs.so",
            "install(TARGETS",
        ):
            self.assertIn(token, cmake)
        self.assertNotIn("/media/", cmake)
        self.assertNotIn("P450-PAPER", cmake)
        self.assertNotIn("REGEX MATCHALL", cmake)
        compile_section = cmake.split(
            "find_package(Protobuf 3.6.1 EXACT REQUIRED)", 1)[1]
        self.assertNotIn("P450_PX4_ROOT", compile_section)

    def test_build_wrapper_validates_external_then_installs_and_revalidates(self):
        self.assertTrue(
            OVERLAY_BUILD_WRAPPER.is_file(),
            OVERLAY_BUILD_WRAPPER.as_posix(),
        )
        self.assertFalse(OVERLAY_BUILD_WRAPPER.is_symlink())
        self.assertTrue(OVERLAY_BUILD_WRAPPER.stat().st_mode & stat.S_IXUSR)
        script = OVERLAY_BUILD_WRAPPER.read_text(encoding="utf-8")
        for token in (
            "P450_PX4_ROOT",
            "--external-only",
            "runtime_overlays/px4_gazebo_plugins",
            "build/p450-runtime-overlays",
            "install/p450-runtime-overlays",
            "cmake --build",
            "cmake --install",
            "tools/p450_runtime.py",
        ):
            self.assertIn(token, script)
        self.assertNotIn("/media/", script)
        self.assertNotIn("P450-PAPER", script)
        self.assertNotIn("sed -i", script)
        self.assertNotIn("git apply", script)

    def test_build_wrapper_rejects_output_symlink_escape(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            repository = root / "runtime repository"
            repository.mkdir()
            outside = root / "outside"
            outside.mkdir()
            (repository / "build").symlink_to(
                outside, target_is_directory=True)

            escaped = subprocess.run(
                [
                    str(OVERLAY_BUILD_WRAPPER),
                    "--test-output-path",
                    str(repository),
                    "build/p450-runtime-overlays",
                ],
                env={"PATH": "/usr/bin:/bin"},
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
            )
            safe = subprocess.run(
                [
                    str(OVERLAY_BUILD_WRAPPER),
                    "--test-output-path",
                    str(repository),
                    "install/p450-runtime-overlays",
                ],
                env={"PATH": "/usr/bin:/bin"},
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
            )

        self.assertNotEqual(0, escaped.returncode)
        self.assertIn("symbolic link", escaped.stderr)
        self.assertEqual(0, safe.returncode, safe.stderr)
        self.assertEqual(
            str(repository / "install/p450-runtime-overlays") + "\n",
            safe.stdout,
        )


@unittest.skipIf(RUNTIME_MODULE_MISSING, "runtime validator not materialized")
class RuntimeConfigContractTest(unittest.TestCase):
    def test_real_config_is_exact_and_machine_path_free(self):
        config = load_runtime_config(CONFIG_PATH)
        self.assertEqual(2, config.schema_version)
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
        self.assertEqual(EXPECTED_LOCAL_PLUGIN_ROOT, config.local_plugin_root)
        self.assertEqual(2, len(config.local_plugin_specs))
        gps, groundtruth = config.local_plugin_specs
        self.assertEqual(EXPECTED_GPS_PLUGIN, gps.plugin)
        self.assertEqual(EXPECTED_GPS_SOURCE, gps.upstream_source)
        self.assertEqual(
            EXPECTED_GPS_SOURCE_SHA256, gps.upstream_source_sha256)
        self.assertEqual(
            EXPECTED_PATCHED_GPS_SOURCE_SHA256,
            gps.patched_source_sha256,
        )
        self.assertEqual(EXPECTED_GPS_HEADER, gps.upstream_header)
        self.assertEqual(
            EXPECTED_GPS_HEADER_SHA256, gps.upstream_header_sha256)
        self.assertEqual(
            EXPECTED_GPS_GENERATED_HEADER, gps.generated_header)
        self.assertEqual(
            EXPECTED_GPS_GENERATED_HEADER_SHA256,
            gps.generated_header_sha256,
        )
        self.assertEqual(EXPECTED_GROUNDTRUTH_PLUGIN, groundtruth.plugin)
        self.assertEqual(
            EXPECTED_GROUNDTRUTH_SOURCE, groundtruth.upstream_source)
        self.assertEqual(
            EXPECTED_GROUNDTRUTH_SOURCE_SHA256,
            groundtruth.upstream_source_sha256,
        )
        self.assertEqual(
            EXPECTED_PATCHED_GROUNDTRUTH_SOURCE_SHA256,
            groundtruth.patched_source_sha256,
        )
        self.assertEqual(
            EXPECTED_GROUNDTRUTH_HEADER, groundtruth.upstream_header)
        self.assertEqual(
            EXPECTED_GROUNDTRUTH_HEADER_SHA256,
            groundtruth.upstream_header_sha256,
        )
        self.assertEqual(
            EXPECTED_GROUNDTRUTH_GENERATED_HEADER,
            groundtruth.generated_header,
        )
        self.assertEqual(
            EXPECTED_GROUNDTRUTH_GENERATED_HEADER_SHA256,
            groundtruth.generated_header_sha256,
        )
        self.assertEqual(
            EXPECTED_OVERLAY_SUPPORT_LIBRARY,
            config.overlay_support_library,
        )
        self.assertEqual(
            EXPECTED_OVERLAY_SUPPORT_LIBRARY_SHA256,
            config.overlay_support_library_sha256,
        )
        self.assertEqual(
            EXPECTED_OVERLAY_COMMON_HEADER,
            config.overlay_common_header,
        )
        self.assertEqual(
            EXPECTED_OVERLAY_COMMON_HEADER_SHA256,
            config.overlay_common_header_sha256,
        )

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

            unsafe_overlay = copy.deepcopy(baseline)
            unsafe_overlay["gazebo"]["local_plugin_overlay"][
                "plugin_root"] = "../outside/lib"
            cases.append(unsafe_overlay)

            invalid_hash = copy.deepcopy(baseline)
            invalid_hash["gazebo"]["local_plugin_overlay"]["plugins"][0][
                "upstream_source_sha256"] = "not-a-sha256"
            cases.append(invalid_hash)

            duplicate_overlay = copy.deepcopy(baseline)
            duplicate_overlay["gazebo"]["local_plugin_overlay"][
                "plugins"].append(copy.deepcopy(
                    duplicate_overlay["gazebo"]["local_plugin_overlay"][
                        "plugins"][0]))
            cases.append(duplicate_overlay)

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
        self.assertEqual(2, len(resolved.overlay_plugin_inputs))
        self.assertEqual(
            (
                EXPECTED_GPS_PLUGIN,
                self.checkout.root / EXPECTED_GPS_SOURCE,
                self.checkout.root / EXPECTED_GPS_HEADER,
                self.checkout.root / EXPECTED_GPS_GENERATED_HEADER,
            ),
            (
                resolved.overlay_plugin_inputs[0].spec.plugin,
                resolved.overlay_plugin_inputs[0].upstream_source,
                resolved.overlay_plugin_inputs[0].upstream_header,
                resolved.overlay_plugin_inputs[0].generated_header,
            ),
        )
        self.assertEqual(
            EXPECTED_GROUNDTRUTH_PLUGIN,
            resolved.overlay_plugin_inputs[1].spec.plugin,
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

        with self.assertRaises(RuntimeValidationError):
            validate_px4_checkout(
                self.config,
                self.checkout.root,
                repository_root=self.checkout.root / "nested-sim-repository",
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

    def test_support_library_resolution_allows_not_found_words_in_root(self):
        checkout = FakePx4Checkout(self.root / "external not found px4")
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


@unittest.skipUnless(
    callable(validate_runtime_plugin_overlay),
    "local plugin overlay validator not materialized",
)
class LocalPluginOverlayValidationTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.repository = self.root / "runtime repository"
        self.repository.mkdir()
        self.checkout = FakePx4Checkout(self.root / "external px4")
        config_path = self.root / "runtime.json"
        config_path.write_text(
            json.dumps(self.checkout.payload()), encoding="utf-8")
        self.config = load_runtime_config(config_path)
        self.external = validate_px4_checkout(
            self.config,
            self.checkout.root,
            repository_root=self.repository,
            ldd_runner=lambda _path, _environment: "clean",
        )
        self.overlay_root = self.repository / EXPECTED_LOCAL_PLUGIN_ROOT
        self.overlay_root.mkdir(parents=True)
        self.plugins = tuple(
            self.overlay_root / name
            for name in (EXPECTED_GPS_PLUGIN, EXPECTED_GROUNDTRUTH_PLUGIN)
        )
        for plugin in self.plugins:
            shutil.copy2("/bin/true", plugin)

    def _clean_ldd(self, _path, _environment):
        return "libsensor_msgs.so => %s" % self.external.overlay_support_library

    @staticmethod
    def _clean_dynamic(path):
        return (
            "0x0000000000000001 (NEEDED) Shared library: [libsensor_msgs.so]\n"
            "0x000000000000000e (SONAME) Library soname: "
            "[%s]\n" % Path(path).name
        )

    @staticmethod
    def _clean_header(_path):
        return (
            "Class:                             ELF64\n"
            "Type:                              DYN (Shared object file)\n"
            "Machine:                           Advanced Micro Devices X86-64\n"
        )

    @staticmethod
    def _clean_symbols(_path):
        return (
            "12: 0000000000001000 42 FUNC GLOBAL DEFAULT 12 RegisterPlugin\n"
        )

    def test_overlay_resolves_inside_repository_with_external_support(self):
        observed = {}

        def ldd_runner(_path, environment):
            observed.update(environment)
            return "libsensor_msgs.so => %s/libsensor_msgs.so" % (
                self.external.plugin_root)

        resolved = validate_runtime_plugin_overlay(
            self.config,
            self.repository,
            self.external,
            ldd_runner=ldd_runner,
            header_runner=self._clean_header,
            dynamic_runner=self._clean_dynamic,
            symbol_runner=self._clean_symbols,
        )
        self.assertEqual(self.overlay_root.resolve(), resolved.root)
        self.assertEqual(
            tuple(plugin.resolve() for plugin in self.plugins),
            resolved.plugins,
        )
        self.assertEqual(
            "%s:%s" % (self.overlay_root, self.external.plugin_root),
            observed["LD_LIBRARY_PATH"],
        )

    def test_missing_escaping_unresolved_and_rpath_overlay_fail(self):
        plugin = self.plugins[0]
        plugin.unlink()
        with self.assertRaises(RuntimeValidationError):
            validate_runtime_plugin_overlay(
                self.config,
                self.repository,
                self.external,
                ldd_runner=self._clean_ldd,
                header_runner=self._clean_header,
                dynamic_runner=self._clean_dynamic,
                symbol_runner=self._clean_symbols,
            )

        outside = self.root / EXPECTED_GPS_PLUGIN
        shutil.copy2("/bin/true", outside)
        plugin.symlink_to(outside)
        with self.assertRaises(RuntimeValidationError):
            validate_runtime_plugin_overlay(
                self.config,
                self.repository,
                self.external,
                ldd_runner=self._clean_ldd,
                header_runner=self._clean_header,
                dynamic_runner=self._clean_dynamic,
                symbol_runner=self._clean_symbols,
            )
        plugin.unlink()
        shutil.copy2("/bin/true", plugin)

        with self.assertRaises(RuntimeValidationError):
            validate_runtime_plugin_overlay(
                self.config,
                self.repository,
                self.external,
                ldd_runner=lambda _path, _environment: (
                    "libsensor_msgs.so => not found"),
                header_runner=self._clean_header,
                dynamic_runner=self._clean_dynamic,
                symbol_runner=self._clean_symbols,
            )

        with self.assertRaises(RuntimeValidationError):
            validate_runtime_plugin_overlay(
                self.config,
                self.repository,
                self.external,
                ldd_runner=self._clean_ldd,
                header_runner=self._clean_header,
                dynamic_runner=lambda _path: (
                    "0x000000000000001d (RUNPATH) Library runpath: [/tmp]"),
                symbol_runner=self._clean_symbols,
            )

        invalid_contracts = (
            {
                "ldd_runner": lambda _path, _environment: (
                    "libc.so.6 => /lib/libc.so.6"),
                "header_runner": self._clean_header,
                "dynamic_runner": self._clean_dynamic,
                "symbol_runner": self._clean_symbols,
            },
            {
                "ldd_runner": self._clean_ldd,
                "header_runner": self._clean_header,
                "dynamic_runner": lambda _path: (
                    "0x0 (SONAME) Library soname: [wrong.so]\n"),
                "symbol_runner": self._clean_symbols,
            },
            {
                "ldd_runner": self._clean_ldd,
                "header_runner": self._clean_header,
                "dynamic_runner": self._clean_dynamic,
                "symbol_runner": lambda _path: "",
            },
            {
                "ldd_runner": self._clean_ldd,
                "header_runner": self._clean_header,
                "dynamic_runner": self._clean_dynamic,
                "symbol_runner": lambda _path: (
                    "12: 0000000000001000 42 FUNC GLOBAL DEFAULT 12 "
                    "RegisterPlugin\n"
                    "13: 0000000000000000 0 FUNC GLOBAL DEFAULT UND "
                    "gazebo::physics::World::Reset()\n"),
            },
            {
                "ldd_runner": self._clean_ldd,
                "header_runner": self._clean_header,
                "dynamic_runner": self._clean_dynamic,
                "symbol_runner": lambda _path: (
                    "12: 0000000000001000 8 OBJECT GLOBAL DEFAULT 12 "
                    "RegisterPlugin\n"),
            },
            {
                "ldd_runner": self._clean_ldd,
                "header_runner": lambda _path: (
                    "Class: ELF32\n"
                    "Type: DYN (Shared object file)\n"
                    "Machine: Intel 80386\n"),
                "dynamic_runner": self._clean_dynamic,
                "symbol_runner": self._clean_symbols,
            },
            {
                "ldd_runner": self._clean_ldd,
                "header_runner": lambda _path: (
                    "Class: ELF64\n"
                    "Type: DYN (Position-Independent Executable file)\n"
                    "Machine: Advanced Micro Devices X86-64\n"),
                "dynamic_runner": self._clean_dynamic,
                "symbol_runner": self._clean_symbols,
            },
        )
        for runners in invalid_contracts:
            with self.subTest(runners=tuple(runners)):
                with self.assertRaises(RuntimeValidationError):
                    validate_runtime_plugin_overlay(
                        self.config,
                        self.repository,
                        self.external,
                        **runners,
                    )


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
        external_plugin_root = checkout.root / (
            "build/amovlab_sitl_default/build_gazebo")
        sensor_source = root / "fake_sensor_messages.c"
        sensor_source.write_text(
            "void p450_fake_sensor_message(void) {}\n", encoding="utf-8")
        _run((
            "/usr/bin/cc", "-shared", "-fPIC",
            "-Wl,-soname,libsensor_msgs.so",
            "-o", str(external_plugin_root / "libsensor_msgs.so"),
            str(sensor_source),
        ), root)

        for relative in (
            "scripts/with_p450_env.bash",
            "scripts/with_noetic_env.bash",
            "tools/p450_runtime.py",
        ):
            destination = runtime_root / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / relative, destination)

        local_plugin_root = runtime_root / EXPECTED_LOCAL_PLUGIN_ROOT
        local_plugin_root.mkdir(parents=True)
        overlay_source = root / "fake_groundtruth_overlay.c"
        overlay_source.write_text(
            "extern void p450_fake_sensor_message(void);\n"
            "void RegisterPlugin(void) { p450_fake_sensor_message(); }\n",
            encoding="utf-8",
        )
        for plugin_name in (EXPECTED_GPS_PLUGIN, EXPECTED_GROUNDTRUTH_PLUGIN):
            _run((
                "/usr/bin/cc", "-shared", "-fPIC",
                "-Wl,-soname,%s" % plugin_name,
                "-o", str(local_plugin_root / plugin_name),
                str(overlay_source),
                "-L" + str(external_plugin_root),
                "-Wl,--no-as-needed", "-lsensor_msgs",
            ), root)

        config_path = runtime_root / "config/p450_runtime.json"
        config_path.parent.mkdir(parents=True)
        config_path.write_text(
            json.dumps(checkout.payload()), encoding="utf-8")

        install_root = runtime_root / "install/p450-clean"
        package_root = install_root / "share/prometheus_gazebo"
        for relative in EXPECTED_LOCAL_MODEL_ROOTS:
            (package_root / relative).mkdir(parents=True)
        for name in EXPECTED_REQUIRED_PACKAGES:
            installed_package = install_root / "share" / name
            installed_package.mkdir(parents=True, exist_ok=True)
            (installed_package / "package.xml").write_text(
                FakePx4Checkout._package_xml(name), encoding="utf-8")
        platform_package = install_root / "share/sim_platform_bringup"
        platform_package.mkdir(parents=True, exist_ok=True)
        (platform_package / "package.xml").write_text(
            FakePx4Checkout._package_xml("sim_platform_bringup"),
            encoding="utf-8",
        )
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
            root = Path(temporary)
            px4_root = root / "px4 root"
            ros_home = root / "ros home"
            caller_cwd = root / "caller cwd"
            ros_home.mkdir()
            caller_cwd.mkdir()
            binary = px4_root / "build/amovlab_sitl_default/bin/px4"
            binary.parent.mkdir(parents=True)
            binary.write_text(
                "#!/bin/bash\nprintf '<%s>\\n' \"$@\"\n",
                encoding="utf-8",
            )
            binary.chmod(0o755)
            result = subprocess.run(
                [
                    str(PX4_NODE_WRAPPER),
                    "-w", "sitl_amov_0", "--help", "two words",
                ],
                cwd=str(caller_cwd),
                env={
                    "P450_PX4_ROOT": str(px4_root.resolve()),
                    "ROS_HOME": str(ros_home.resolve()),
                    "PATH": "/usr/bin:/bin",
                },
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
            )
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual(
            "<-w>\n<sitl_amov_0>\n<--help>\n<two words>\n",
            result.stdout,
        )

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
            "/opt/ros/noetic/bin/rospack",
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
        local_plugin_root = runtime_root / EXPECTED_LOCAL_PLUGIN_ROOT
        self.assertEqual(
            "%s:%s" % (local_plugin_root, plugin_root),
            environment["GAZEBO_PLUGIN_PATH"],
        )
        self.assertEqual(
            [str(local_plugin_root), str(plugin_root)],
            environment["LD_LIBRARY_PATH"].split(":")[:2],
        )
        self.assertEqual(
            "%s:%s:%s/share:/opt/ros/noetic/share" % (
                checkout.root,
                checkout.sitl,
                runtime_root / "install/p450-clean",
            ),
            environment["ROS_PACKAGE_PATH"],
        )

    def test_runtime_wrapper_rejects_missing_local_plugin_overlay(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runtime_root, checkout, _probe = self._make_wrapper_fixture(root)
            plugin = (
                runtime_root / EXPECTED_LOCAL_PLUGIN_ROOT /
                EXPECTED_GROUNDTRUTH_PLUGIN
            )
            plugin.unlink()
            result = subprocess.run(
                [
                    str(runtime_root / "scripts/with_p450_env.bash"),
                    "/usr/bin/true",
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
        self.assertNotEqual(0, result.returncode)
        self.assertIn("local plugin overlay", result.stderr)

    def test_runtime_wrapper_maps_explicit_render_capability(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runtime_root, checkout, probe = self._make_wrapper_fixture(root)
            xauthority = root / "render authority"
            xauthority.write_text("test cookie\n", encoding="utf-8")
            result = subprocess.run(
                [
                    str(runtime_root / "scripts/with_p450_env.bash"),
                    "/usr/bin/python3", str(probe),
                ],
                env={
                    "PATH": "/usr/bin:/bin",
                    "DISPLAY": ":99",
                    "XAUTHORITY": "/poison/authority",
                    "P450_PX4_ROOT": str(checkout.root),
                    "P450_GAZEBO_DISPLAY": ":91.0",
                    "P450_GAZEBO_XAUTHORITY": str(xauthority),
                },
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
            )

        self.assertEqual(0, result.returncode, result.stderr)
        environment = json.loads(result.stdout)["environment"]
        self.assertEqual(":91.0", environment.get("DISPLAY"))
        self.assertEqual(str(xauthority.resolve()), environment.get("XAUTHORITY"))
        self.assertNotIn("/poison", "\n".join(environment.values()))

    def test_runtime_wrapper_rejects_invalid_render_capability(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runtime_root, checkout, _probe = self._make_wrapper_fixture(root)
            wrapper = runtime_root / "scripts/with_p450_env.bash"
            xauthority = root / "render authority"
            xauthority.write_text("test cookie\n", encoding="utf-8")
            base_environment = {
                "PATH": "/usr/bin:/bin",
                "P450_PX4_ROOT": str(checkout.root),
            }
            cases = (
                (
                    {"P450_GAZEBO_DISPLAY": ":91"},
                    "P450_GAZEBO_XAUTHORITY",
                ),
                (
                    {"P450_GAZEBO_XAUTHORITY": str(xauthority)},
                    "P450_GAZEBO_DISPLAY",
                ),
                (
                    {
                        "P450_GAZEBO_DISPLAY": "remote.example:0",
                        "P450_GAZEBO_XAUTHORITY": str(xauthority),
                    },
                    "local X display",
                ),
                (
                    {
                        "P450_GAZEBO_DISPLAY": ":91",
                        "P450_GAZEBO_XAUTHORITY": str(root / "missing"),
                    },
                    "P450_GAZEBO_XAUTHORITY",
                ),
            )
            for extra_environment, expected_error in cases:
                with self.subTest(extra_environment=extra_environment):
                    result = subprocess.run(
                        [str(wrapper), "/usr/bin/true"],
                        env={**base_environment, **extra_environment},
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        check=False,
                    )
                    self.assertNotEqual(0, result.returncode)
                    self.assertIn(expected_error, result.stderr)

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
