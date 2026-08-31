import re
import shlex
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
P450 = ROOT / "src/p450"

EXPECTED_VENDOR_FILES = frozenset({
    "common/include/geometry_utils.h",
    "common/include/math_utils.h",
    "common/include/printf_utils.h",
    "communication/include/message_convert.hpp",
    "communication/include/param_manager.hpp",
    "communication/src/param_manager.cpp",
})

EXPECTED_TARGETS = {
    "prometheus_gazebo": {},
    "prometheus_uav_control": {
        "uav_controller": (
            "src/uav_controller.cpp",
            "vendor_upstream/communication/src/param_manager.cpp",
        ),
        "uav_estimator": ("src/uav_estimator.cpp",),
        "uav_control_main": ("src/uav_control_node.cpp",),
        "uav_command_pub": ("utils/uav_command_pub.cpp",),
        "rc_test": ("utils/rc_test.cpp",),
        "joy_node": ("utils/joy_node.cpp",),
    },
    "realsense_ros_gazebo": {
        "realsense_gazebo_plugin": (
            "src/RealSensePlugin.cpp",
            "src/gazebo_ros_realsense.cpp",
        ),
    },
}

EXPECTED_MSG_COMPONENTS = frozenset({
    "actionlib_msgs",
    "geometry_msgs",
    "message_generation",
    "sensor_msgs",
    "std_msgs",
})

REQUIRED_CATKIN_COMPONENTS = {
    "prometheus_uav_control": frozenset({
        "diagnostic_updater",
        "geometry_msgs",
        "mavros",
        "mavros_msgs",
        "nav_msgs",
        "prometheus_msgs",
        "roscpp",
        "sensor_msgs",
        "std_msgs",
        "tf2_geometry_msgs",
        "tf2_ros",
        "visualization_msgs",
    }),
    "realsense_ros_gazebo": frozenset({
        "camera_info_manager",
        "gazebo_dev",
        "gazebo_ros",
        "image_transport",
        "roscpp",
        "sensor_msgs",
    }),
}

REQUIRED_MANIFEST_DEPENDENCIES = {
    "prometheus_uav_control": REQUIRED_CATKIN_COMPONENTS[
        "prometheus_uav_control"
    ] | frozenset({"eigen", "geographiclib", "mavlink"}),
    "realsense_ros_gazebo": REQUIRED_CATKIN_COMPONENTS[
        "realsense_ros_gazebo"
    ] | frozenset({"boost"}),
}

EXPECTED_MSG_MANIFEST_PHASES = {
    "build": EXPECTED_MSG_COMPONENTS,
    "export": EXPECTED_MSG_COMPONENTS - {"message_generation"},
    "exec": ((EXPECTED_MSG_COMPONENTS - {"message_generation"}) |
             {"message_runtime"}),
}

REQUIRED_INSTALL_DIRECTORIES = {
    "prometheus_gazebo": frozenset({
        "config",
        "gazebo_models",
        "gazebo_worlds",
        "launch_basic",
        "launch_fmt",
        "launch_test",
        "launch_uav_with_sensor",
    }),
    "prometheus_uav_control": frozenset({
        "include",
        "launch",
        "launch_controller_test",
        "meshes",
    }),
    "realsense_ros_gazebo": frozenset({
        "include/realsense_gazebo_plugin",
        "worlds",
    }),
    "brick_aerial_perception": frozenset({
        "config",
        "launch",
        "rviz",
        "worlds",
    }),
}

FORBIDDEN_DEPENDENCIES = frozenset({
    "air_ground_pose_bridge",
    "aerial_ground_bridge",
    "aerial_traversability",
    "aubo_description",
    "brick_pick_demo",
    "brick_rgbd_perception",
    "brick_visual_pick",
    "bunker_aubo_description",
    "bunker_aubo_gazebo",
    "bunker_aubo_moveit_config",
    "bunker_description",
    "bunker_navigation",
    "dh_ag95_description",
    "ground_aerial_benchmark",
    "ground_pick_orchestrator",
    "irm_base_placement",
    "paper_benchmark",
    "roboticsgroup_gazebo_plugins",
    "system_baseline_freeze",
    "task_aware_approach",
})


def _cmake_path(package):
    return P450 / package / "CMakeLists.txt"


def _strip_cmake_comments(text):
    lines = []
    for line in text.splitlines():
        quote = None
        kept = []
        for character in line:
            if character in ("'", '"'):
                quote = None if quote == character else character
            if character == "#" and quote is None:
                break
            kept.append(character)
        lines.append("".join(kept))
    return "\n".join(lines)


def _cmake_commands(package):
    """Return (lowercase command, shell-like tokens) for simple CMake files."""
    text = _strip_cmake_comments(_cmake_path(package).read_text(encoding="utf-8"))
    start = re.compile(r"\b([A-Za-z_][A-Za-z0-9_]*)\s*\(")
    commands = []
    cursor = 0
    while True:
        match = start.search(text, cursor)
        if match is None:
            return tuple(commands)
        depth = 1
        quote = None
        index = match.end()
        body_start = index
        while index < len(text) and depth:
            character = text[index]
            if character in ("'", '"'):
                quote = None if quote == character else character
            elif quote is None and character == "(":
                depth += 1
            elif quote is None and character == ")":
                depth -= 1
            index += 1
        if depth:
            raise AssertionError("unterminated CMake command in %s" % package)
        commands.append((
            match.group(1).lower(),
            tuple(shlex.split(text[body_start:index - 1])),
        ))
        cursor = index


def _commands(package, name):
    return tuple(tokens for command, tokens in _cmake_commands(package)
                 if command == name)


def _catkin_components(package):
    for tokens in _commands(package, "find_package"):
        if tokens and tokens[0] == "catkin":
            if "COMPONENTS" not in tokens:
                return frozenset()
            return frozenset(tokens[tokens.index("COMPONENTS") + 1:])
    raise AssertionError("%s has no find_package(catkin ...)" % package)


def _targets(package):
    result = {}
    for command in ("add_library", "add_executable"):
        for tokens in _commands(package, command):
            if tokens:
                result[tokens[0]] = tuple(
                    token for token in tokens[1:]
                    if Path(token).suffix in (".c", ".cc", ".cpp", ".cxx")
                )
    return result


def _manifest_dependencies(package):
    root = ET.parse(str(P450 / package / "package.xml")).getroot()
    phases = {"build": set(), "export": set(), "exec": set()}
    for element in root:
        value = (element.text or "").strip()
        if not value:
            continue
        if element.tag == "depend":
            for dependencies in phases.values():
                dependencies.add(value)
        elif element.tag == "build_depend":
            phases["build"].add(value)
        elif element.tag == "build_export_depend":
            phases["export"].add(value)
        elif element.tag == "exec_depend":
            phases["exec"].add(value)
    return {name: frozenset(values) for name, values in phases.items()}


def _all_manifest_dependencies(package):
    return frozenset().union(*_manifest_dependencies(package).values())


def _install_directory_sources(package):
    result = []
    for tokens in _commands(package, "install"):
        if not tokens or tokens[0] != "DIRECTORY":
            continue
        for token in tokens[1:]:
            if token == "DESTINATION":
                break
            result.append(token.replace("${PROJECT_NAME}", package))
    return tuple(result)


def _catkin_installed_programs(package):
    result = []
    for tokens in _commands(package, "catkin_install_python"):
        if not tokens or tokens[0] != "PROGRAMS":
            continue
        for token in tokens[1:]:
            if token == "DESTINATION":
                break
            result.append(token)
    return frozenset(result)


def _installed_targets(package):
    result = []
    stop = {"ARCHIVE", "DESTINATION", "INCLUDES", "LIBRARY", "RUNTIME"}
    for tokens in _commands(package, "install"):
        if not tokens or tokens[0] != "TARGETS":
            continue
        for token in tokens[1:]:
            if token in stop:
                break
            result.append(token.replace("${PROJECT_NAME}", package))
    return frozenset(result)


def _catkin_exported_libraries(package):
    commands = _commands(package, "catkin_package")
    if len(commands) != 1 or "LIBRARIES" not in commands[0]:
        return frozenset()
    tokens = commands[0]
    start = tokens.index("LIBRARIES") + 1
    stop = {"CATKIN_DEPENDS", "CFG_EXTRAS", "DEPENDS", "INCLUDE_DIRS"}
    result = []
    for token in tokens[start:]:
        if token in stop:
            break
        result.append(token.replace("${PROJECT_NAME}", package))
    return frozenset(result)


class P450BuildContractTest(unittest.TestCase):
    def test_uav_control_uses_only_its_frozen_vendor_helpers(self):
        vendor = P450 / "prometheus_uav_control/vendor_upstream"
        actual = frozenset(
            path.relative_to(vendor).as_posix()
            for path in vendor.rglob("*") if path.is_file()
        )
        self.assertEqual(EXPECTED_VENDOR_FILES, actual)

        text = _cmake_path("prometheus_uav_control").read_text(encoding="utf-8")
        self.assertNotIn("../common", text)
        self.assertNotIn("../communication", text)
        for required in (
            "vendor_upstream/common/include",
            "vendor_upstream/communication/include",
            "vendor_upstream/communication/src/param_manager.cpp",
        ):
            self.assertIn(required, text)

    def test_compiled_targets_and_sources_are_exact_and_present(self):
        for package, expected in EXPECTED_TARGETS.items():
            with self.subTest(package=package):
                self.assertEqual(expected, _targets(package))
            for target, sources in expected.items():
                for source in sources:
                    with self.subTest(package=package, target=target,
                                      source=source):
                        self.assertTrue((P450 / package / source).is_file())

    def test_asset_and_control_packages_have_no_phantom_messages(self):
        for package in ("prometheus_gazebo", "prometheus_uav_control"):
            text = _cmake_path(package).read_text(encoding="utf-8")
            dependencies = _all_manifest_dependencies(package)
            for stale in ("message_generation", "message_runtime"):
                with self.subTest(package=package, stale=stale):
                    self.assertNotIn(stale, text)
                    self.assertNotIn(stale, dependencies)
            self.assertNotIn("generate_messages", {
                command for command, _tokens in _cmake_commands(package)
            })
        self.assertNotIn(
            "prometheus_gazebo_gencpp",
            _cmake_path("prometheus_gazebo").read_text(encoding="utf-8"),
        )
        self.assertNotIn(
            "prometheus_msgs",
            _catkin_components("prometheus_gazebo"),
            "src/p450/prometheus_gazebo/CMakeLists.txt: asset-only package "
            "must not compile against prometheus_msgs",
        )
        self.assertNotIn(
            "prometheus_msgs",
            _all_manifest_dependencies("prometheus_gazebo"),
            "src/p450/prometheus_gazebo/package.xml: asset-only package must "
            "not declare prometheus_msgs",
        )

    def test_message_package_has_only_its_real_generation_dependencies(self):
        package = "prometheus_msgs"
        self.assertEqual(EXPECTED_MSG_COMPONENTS, _catkin_components(package))
        non_catkin = frozenset(
            tokens[0] for tokens in _commands(package, "find_package")
            if tokens and tokens[0] != "catkin"
        )
        self.assertEqual(frozenset(), non_catkin)
        dependencies = _manifest_dependencies(package)
        for phase, expected in EXPECTED_MSG_MANIFEST_PHASES.items():
            with self.subTest(package=package, phase=phase):
                self.assertEqual(
                    expected,
                    dependencies[phase],
                    "src/p450/prometheus_msgs/package.xml: %s dependency "
                    "closure differs; expected=%s actual=%s" % (
                        phase, sorted(expected), sorted(dependencies[phase])),
                )

    def test_compiled_dependencies_are_declared(self):
        for package, required in REQUIRED_CATKIN_COMPONENTS.items():
            missing_cmake = required - _catkin_components(package)
            with self.subTest(package=package, declaration="CMake"):
                self.assertFalse(
                    missing_cmake,
                    "src/p450/%s/CMakeLists.txt: missing direct Catkin "
                    "components %s" % (package, sorted(missing_cmake)),
                )
            manifest = _manifest_dependencies(package)
            for phase in ("build", "export", "exec"):
                required_phase = REQUIRED_MANIFEST_DEPENDENCIES[package]
                missing = required_phase - manifest[phase]
                with self.subTest(package=package, phase=phase):
                    self.assertFalse(
                        missing,
                        "src/p450/%s/package.xml: missing %s dependencies %s"
                        % (package, phase, sorted(missing)),
                    )

        uav_manifest = _manifest_dependencies("prometheus_uav_control")
        self.assertIn(
            "prometheus_msgs",
            uav_manifest["build"],
            "src/p450/prometheus_uav_control/package.xml: missing build edge "
            "to prometheus_msgs",
        )

    def test_aerial_perception_declares_d435_runtime(self):
        runtime = _manifest_dependencies("brick_aerial_perception")["exec"]
        required = {
            "prometheus_gazebo",
            "prometheus_msgs",
            "prometheus_uav_control",
            "realsense_ros_gazebo",
        }
        missing = required - runtime
        self.assertFalse(
            missing,
            "src/p450/brick_aerial_perception/package.xml: missing exec "
            "dependencies %s" % sorted(missing),
        )

    def test_install_directories_exist(self):
        for package in (
            "prometheus_msgs",
            "prometheus_gazebo",
            "prometheus_uav_control",
            "realsense_ros_gazebo",
            "brick_aerial_perception",
        ):
            for relative in _install_directory_sources(package):
                with self.subTest(package=package, relative=relative):
                    self.assertNotIn("${", relative)
                    self.assertTrue(
                        (P450 / package / relative).is_dir(),
                        "src/p450/%s/CMakeLists.txt: install DIRECTORY does "
                        "not exist: %s" % (package, relative),
                    )

        for package, required in REQUIRED_INSTALL_DIRECTORIES.items():
            actual = frozenset(
                relative.rstrip("/")
                for relative in _install_directory_sources(package)
            )
            missing = required - actual
            with self.subTest(package=package, contract="required-resources"):
                self.assertFalse(
                    missing,
                    "src/p450/%s/CMakeLists.txt: missing install directories "
                    "%s" % (package, sorted(missing)),
                )

        self.assertIn(
            "scripts/jinja_gen.py",
            _catkin_installed_programs("prometheus_gazebo"),
            "src/p450/prometheus_gazebo/CMakeLists.txt: jinja_gen.py must be "
            "installed as an executable program",
        )

    def test_runtime_targets_are_installed_and_realsense_is_exported(self):
        self.assertEqual(
            frozenset(EXPECTED_TARGETS["prometheus_uav_control"]),
            _installed_targets("prometheus_uav_control"),
        )
        self.assertEqual(
            frozenset({"realsense_gazebo_plugin"}),
            _installed_targets("realsense_ros_gazebo"),
        )
        self.assertEqual(
            frozenset({"realsense_gazebo_plugin"}),
            _catkin_exported_libraries("realsense_ros_gazebo"),
        )

    def test_p450_slice_has_no_ground_or_paper_dependencies(self):
        for package in (
            "prometheus_msgs",
            "prometheus_gazebo",
            "prometheus_uav_control",
            "realsense_ros_gazebo",
            "brick_aerial_perception",
        ):
            actual = _catkin_components(package) | _all_manifest_dependencies(package)
            with self.subTest(package=package):
                self.assertFalse(actual & FORBIDDEN_DEPENDENCIES)


if __name__ == "__main__":
    unittest.main()
