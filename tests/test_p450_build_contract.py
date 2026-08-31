import json
import os
import pwd
import re
import stat
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
P450 = ROOT / "src/p450"
NOETIC_WRAPPER = ROOT / "scripts/with_noetic_env.bash"

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

BRICK_AERIAL_CATKIN_COMPONENTS = frozenset({
    "cv_bridge",
    "gazebo_msgs",
    "geometry_msgs",
    "mavros_msgs",
    "message_filters",
    "nav_msgs",
    "rospy",
    "sensor_msgs",
    "std_msgs",
    "std_srvs",
    "tf",
    "tf2_ros",
    "visualization_msgs",
})

EXPECTED_CATKIN_COMPONENTS = {
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
    "brick_aerial_perception": BRICK_AERIAL_CATKIN_COMPONENTS,
}

EXPECTED_MANIFEST_PHASES = {
    "prometheus_uav_control": {
        "build": EXPECTED_CATKIN_COMPONENTS["prometheus_uav_control"] |
                 frozenset({
                     "eigen", "geographiclib", "mavlink", "pkg-config",
                 }),
        "export": frozenset({
            "eigen",
            "geographiclib",
            "geometry_msgs",
            "mavlink",
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
        "exec": frozenset({
            "diagnostic_updater",
            "geographiclib",
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
    },
    "realsense_ros_gazebo": {
        "build": EXPECTED_CATKIN_COMPONENTS["realsense_ros_gazebo"] |
                 frozenset({"boost"}),
        "export": frozenset({
            "boost",
            "camera_info_manager",
            "gazebo_dev",
            "image_transport",
            "roscpp",
            "sensor_msgs",
        }),
        "exec": frozenset({
            "camera_info_manager",
            "gazebo",
            "gazebo_ros",
            "image_transport",
            "roscpp",
            "sensor_msgs",
        }),
    },
    "prometheus_gazebo": {
        "build": frozenset(),
        "export": frozenset(),
        "exec": frozenset({
            "gazebo_plugins",
            "gazebo_ros",
            "python3-jinja2",
            "python3-numpy",
            "realsense_ros_gazebo",
        }),
    },
    "brick_aerial_perception": {
        "build": BRICK_AERIAL_CATKIN_COMPONENTS,
        "export": BRICK_AERIAL_CATKIN_COMPONENTS,
        "exec": BRICK_AERIAL_CATKIN_COMPONENTS | frozenset({
            "prometheus_gazebo",
            "prometheus_msgs",
            "prometheus_uav_control",
            "python3-numpy",
            "python3-opencv",
            "python3-yaml",
            "realsense_ros_gazebo",
            "rviz",
        }),
    },
}

EXPECTED_CATKIN_EXPORTS = {
    "prometheus_msgs": frozenset({
        "actionlib_msgs",
        "geometry_msgs",
        "message_runtime",
        "sensor_msgs",
        "std_msgs",
    }),
    "prometheus_uav_control": frozenset({
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
        "image_transport",
        "roscpp",
        "sensor_msgs",
    }),
}

EXPECTED_CATKIN_PACKAGE_FIELDS = {
    "prometheus_msgs": {
        "INCLUDE_DIRS": frozenset(),
        "LIBRARIES": frozenset(),
        "CATKIN_DEPENDS": EXPECTED_CATKIN_EXPORTS["prometheus_msgs"],
        "DEPENDS": frozenset(),
    },
    "prometheus_gazebo": {
        "INCLUDE_DIRS": frozenset(),
        "LIBRARIES": frozenset(),
        "CATKIN_DEPENDS": frozenset(),
        "DEPENDS": frozenset(),
    },
    "prometheus_uav_control": {
        "INCLUDE_DIRS": frozenset({
            "include",
            "vendor_upstream/common/include",
            "vendor_upstream/communication/include",
        }),
        "LIBRARIES": frozenset({"uav_controller", "uav_estimator"}),
        "CATKIN_DEPENDS": EXPECTED_CATKIN_EXPORTS[
            "prometheus_uav_control"
        ],
        "DEPENDS": frozenset({"EIGEN3", "GEOGRAPHICLIB", "mavlink"}),
    },
    "realsense_ros_gazebo": {
        "INCLUDE_DIRS": frozenset({"include"}),
        "LIBRARIES": frozenset({"realsense_gazebo_plugin"}),
        "CATKIN_DEPENDS": EXPECTED_CATKIN_EXPORTS[
            "realsense_ros_gazebo"
        ],
        "DEPENDS": frozenset({"Boost"}),
    },
    "brick_aerial_perception": {
        "INCLUDE_DIRS": frozenset(),
        "LIBRARIES": frozenset(),
        "CATKIN_DEPENDS": frozenset(),
        "DEPENDS": frozenset(),
    },
}

EXPECTED_UAV_INCLUDE_TOKENS = frozenset({
    "include",
    "include/Position_Controller",
    "vendor_upstream/common/include",
    "vendor_upstream/communication/include",
    "${catkin_INCLUDE_DIRS}",
    "${EIGEN3_INCLUDE_DIRS}",
    "${GEOGRAPHICLIB_INCLUDE_DIRS}",
    "${mavlink_INCLUDE_DIRS}",
})

EXPECTED_NON_CATKIN_PACKAGES = {
    "prometheus_msgs": frozenset(),
    "prometheus_gazebo": frozenset(),
    "prometheus_uav_control": frozenset({"Eigen3", "mavlink", "PkgConfig"}),
    "realsense_ros_gazebo": frozenset({"Boost"}),
    "brick_aerial_perception": frozenset(),
}

EXPECTED_MSG_MANIFEST_PHASES = {
    "build": EXPECTED_MSG_COMPONENTS,
    "export": EXPECTED_MSG_COMPONENTS - {"message_generation"},
    "exec": ((EXPECTED_MSG_COMPONENTS - {"message_generation"}) |
             {"message_runtime"}),
}

EXPECTED_INSTALL_DIRECTORIES = {
    "prometheus_msgs": frozenset(),
    "prometheus_gazebo": frozenset({
        "config",
        "gazebo_models",
        "gazebo_worlds",
        "scripts",
    }),
    "prometheus_uav_control": frozenset({
        "include",
        "launch",
        "launch_controller_test",
        "meshes",
        "vendor_upstream/common/include",
        "vendor_upstream/communication/include",
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

EXPECTED_INSTALLED_FILES = {
    "prometheus_msgs": frozenset(),
    "prometheus_gazebo": frozenset({"scripts/jinja_gen.py"}),
    "prometheus_uav_control": frozenset(),
    "realsense_ros_gazebo": frozenset(),
    "brick_aerial_perception": frozenset({
        "README.md",
        "scripts/aerial_brick_pose_node.py",
        "scripts/aerial_viewpoint_mission.py",
        "scripts/world_tf_bridge.py",
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


def _bracket_argument(text, index):
    match = re.match(r"\[(=*)\[", text[index:])
    if match is None:
        return None
    closing = "]%s]" % match.group(1)
    content_start = index + match.end()
    closing_start = text.find(closing, content_start)
    if closing_start < 0:
        raise AssertionError("unterminated CMake bracket argument")
    return content_start, closing_start, closing_start + len(closing)


def _strip_cmake_comments(text):
    result = []
    index = 0
    while index < len(text):
        character = text[index]
        if character == '"':
            start = index
            index += 1
            while index < len(text):
                if text[index] == "\\":
                    index += 2
                    continue
                if text[index] == '"':
                    index += 1
                    break
                index += 1
            result.append(text[start:index])
            continue
        bracket = _bracket_argument(text, index) if character == "[" else None
        if bracket is not None:
            result.append(text[index:bracket[2]])
            index = bracket[2]
            continue
        if character == "#":
            comment_bracket = _bracket_argument(text, index + 1)
            if comment_bracket is not None:
                index = comment_bracket[2]
                continue
            newline = text.find("\n", index)
            if newline < 0:
                break
            result.append("\n")
            index = newline + 1
            continue
        result.append(character)
        index += 1
    return "".join(result)


def _cmake_tokens(text):
    tokens = []
    index = 0
    while index < len(text):
        while index < len(text) and (text[index].isspace() or text[index] == ";"):
            index += 1
        if index >= len(text):
            break
        if text[index] == '"':
            index += 1
            value = []
            while index < len(text):
                if text[index] == "\\" and index + 1 < len(text):
                    value.append(text[index + 1])
                    index += 2
                    continue
                if text[index] == '"':
                    index += 1
                    break
                value.append(text[index])
                index += 1
            tokens.append("".join(value))
            continue
        bracket = _bracket_argument(text, index) if text[index] == "[" else None
        if bracket is not None:
            tokens.append(text[bracket[0]:bracket[1]])
            index = bracket[2]
            continue
        start = index
        while (index < len(text) and not text[index].isspace() and
               text[index] != ";"):
            index += 1
        tokens.append(text[start:index])
    return tuple(tokens)


def _cmake_commands_from_text(raw_text):
    """Parse the command subset used by the frozen P450 CMake files."""
    text = _strip_cmake_comments(raw_text)
    start = re.compile(r"\b([A-Za-z_][A-Za-z0-9_]*)\s*\(")
    commands = []
    cursor = 0
    while True:
        match = start.search(text, cursor)
        if match is None:
            return tuple(commands)
        depth = 1
        index = match.end()
        body_start = index
        while index < len(text) and depth:
            character = text[index]
            if character == '"':
                index += 1
                while index < len(text):
                    if text[index] == "\\":
                        index += 2
                        continue
                    if text[index] == '"':
                        index += 1
                        break
                    index += 1
                continue
            bracket = (_bracket_argument(text, index)
                       if character == "[" else None)
            if bracket is not None:
                index = bracket[2]
                continue
            if character == "\\" and index + 1 < len(text):
                index += 2
                continue
            if character == "(":
                depth += 1
            elif character == ")":
                depth -= 1
            index += 1
        if depth:
            raise AssertionError("unterminated CMake command")
        commands.append((
            match.group(1).lower(),
            _cmake_tokens(text[body_start:index - 1]),
        ))
        cursor = index


def _cmake_commands(package):
    return _cmake_commands_from_text(
        _cmake_path(package).read_text(encoding="utf-8")
    )


def _commands(package, name):
    return tuple(tokens for command, tokens in _cmake_commands(package)
                 if command == name)


def _catkin_components(package):
    for tokens in _commands(package, "find_package"):
        if tokens and tokens[0] == "catkin":
            if "COMPONENTS" not in tokens:
                return frozenset()
            result = []
            stop = {"OPTIONAL_COMPONENTS", "NO_POLICY_SCOPE"}
            for token in tokens[tokens.index("COMPONENTS") + 1:]:
                if token in stop:
                    break
                result.append(token)
            return frozenset(result)
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


def _manifest_buildtools(package):
    root = ET.parse(str(P450 / package / "package.xml")).getroot()
    return frozenset(
        (element.text or "").strip()
        for element in root.findall("buildtool_depend")
        if (element.text or "").strip()
    )


def _install_directory_sources(package):
    result = []
    stop = {
        "COMPONENT", "CONFIGURATIONS", "DESTINATION", "EXCLUDE_FROM_ALL",
        "FILES_MATCHING", "MESSAGE_NEVER", "OPTIONAL", "PATTERN", "REGEX",
        "TYPE", "USE_SOURCE_PERMISSIONS",
    }
    for tokens in _commands(package, "install"):
        if not tokens or tokens[0] != "DIRECTORY":
            continue
        for token in tokens[1:]:
            if token in stop:
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


def _installed_file_sources(package):
    result = set(_catkin_installed_programs(package))
    stop = {
        "COMPONENT", "CONFIGURATIONS", "DESTINATION", "EXCLUDE_FROM_ALL",
        "OPTIONAL", "PERMISSIONS", "RENAME", "TYPE",
    }
    for tokens in _commands(package, "install"):
        if not tokens or tokens[0] not in {"FILES", "PROGRAMS"}:
            continue
        for token in tokens[1:]:
            if token in stop:
                break
            result.add(token.replace("${PROJECT_NAME}", package))
    return frozenset(result)


def _installed_targets_from_commands(commands):
    result = []
    stop = {
        "ARCHIVE", "BUNDLE", "COMPONENT", "CONFIGURATIONS", "DESTINATION",
        "EXCLUDE_FROM_ALL", "EXPORT", "INCLUDES", "LIBRARY",
        "NAMELINK_COMPONENT", "NAMELINK_ONLY", "NAMELINK_SKIP",
        "PRIVATE_HEADER", "PUBLIC_HEADER", "RESOURCE", "RUNTIME",
    }
    for command, tokens in commands:
        if command != "install":
            continue
        if not tokens or tokens[0] != "TARGETS":
            continue
        for token in tokens[1:]:
            if token in stop:
                break
            result.append(token)
    return frozenset(result)


def _installed_targets(package):
    return frozenset(
        token.replace("${PROJECT_NAME}", package)
        for token in _installed_targets_from_commands(_cmake_commands(package))
    )


def _catkin_package_field(package, field):
    commands = _commands(package, "catkin_package")
    if len(commands) != 1 or field not in commands[0]:
        return frozenset()
    tokens = commands[0]
    start = tokens.index(field) + 1
    stop = {
        "CATKIN_DEPENDS", "CFG_EXTRAS", "DEPENDS", "EXPORTED_TARGETS",
        "INCLUDE_DIRS", "LIBRARIES",
    }
    result = []
    for token in tokens[start:]:
        if token in stop:
            break
        result.append(token.replace("${PROJECT_NAME}", package))
    return frozenset(result)


def _catkin_exported_libraries(package):
    return _catkin_package_field(package, "LIBRARIES")


def _has_required_cxx17_contract(package):
    settings = {
        tokens[0]: tuple(tokens[1:])
        for tokens in _commands(package, "set")
        if tokens
    }
    if (settings.get("CMAKE_CXX_STANDARD") == ("17",) and
            settings.get("CMAKE_CXX_STANDARD_REQUIRED") == ("ON",)):
        return True

    expected_targets = frozenset(EXPECTED_TARGETS[package])
    feature_targets = frozenset(
        tokens[0]
        for tokens in _commands(package, "target_compile_features")
        if tokens and "cxx_std_17" in tokens[1:]
    )
    property_targets = set()
    for tokens in _commands(package, "set_target_properties"):
        if "PROPERTIES" not in tokens:
            continue
        split = tokens.index("PROPERTIES")
        properties = dict(zip(tokens[split + 1::2], tokens[split + 2::2]))
        if (properties.get("CXX_STANDARD") == "17" and
                properties.get("CXX_STANDARD_REQUIRED") == "ON"):
            property_targets.update(tokens[:split])
    return expected_targets <= (feature_targets | property_targets)


class CMakeContractParserTest(unittest.TestCase):
    def test_comments_quotes_brackets_and_install_export_are_structural(self):
        commands = _cmake_commands_from_text(r'''
            # fake_command(ignored)
            #[=[ another_fake(ignored) ]=]
            set(TEXT "value # ( \"quoted\" )")
            set(BRACKET [=[fake_inside(command) # )]=])
            install(TARGETS real_target EXPORT export_set
                    RUNTIME DESTINATION bin)
        ''')
        self.assertEqual(
            ("set", "set", "install"),
            tuple(command for command, _tokens in commands),
        )
        self.assertEqual(
            ("TEXT", 'value # ( "quoted" )'),
            commands[0][1],
        )
        self.assertEqual(
            ("BRACKET", "fake_inside(command) # )"),
            commands[1][1],
        )
        self.assertEqual(
            frozenset({"real_target"}),
            _installed_targets_from_commands(commands),
        )


class P450BuildContractTest(unittest.TestCase):
    def test_uav_control_uses_only_its_frozen_vendor_helpers(self):
        package = "prometheus_uav_control"
        vendor = P450 / "prometheus_uav_control/vendor_upstream"
        actual = frozenset(
            path.relative_to(vendor).as_posix()
            for path in vendor.rglob("*") if path.is_file()
        )
        self.assertEqual(EXPECTED_VENDOR_FILES, actual)

        include_tokens = frozenset(
            token
            for tokens in _commands(package, "include_directories")
            for token in tokens
        )
        self.assertEqual(
            EXPECTED_UAV_INCLUDE_TOKENS,
            include_tokens,
            "src/p450/prometheus_uav_control/CMakeLists.txt: include surface "
            "must remain local plus declared dependency variables",
        )
        self.assertEqual(
            (),
            _commands(package, "target_include_directories"),
            "src/p450/prometheus_uav_control/CMakeLists.txt: target-specific "
            "include paths would bypass the frozen package-local surface",
        )
        self.assertIn(
            "vendor_upstream/communication/src/param_manager.cpp",
            _targets(package)["uav_controller"],
        )

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
            dependencies = _all_manifest_dependencies(package)
            components = _catkin_components(package)
            command_names = {
                command for command, _tokens in _cmake_commands(package)
            }
            target_dependencies = frozenset(
                token
                for tokens in _commands(package, "add_dependencies")
                for token in tokens[1:]
            )
            for stale in ("message_generation", "message_runtime"):
                with self.subTest(package=package, stale=stale):
                    self.assertNotIn(stale, components)
                    self.assertNotIn(stale, dependencies)
            self.assertFalse(
                command_names & {
                    "add_action_files",
                    "add_message_files",
                    "add_service_files",
                    "generate_messages",
                },
                "src/p450/%s/CMakeLists.txt: non-interface package declares "
                "message-generation commands" % package,
            )
            for interface_directory in ("action", "msg", "srv"):
                self.assertFalse(
                    (P450 / package / interface_directory).exists(),
                    "src/p450/%s: stale local interface directory %s" % (
                        package, interface_directory),
                )
            phantom_targets = {
                token for token in target_dependencies
                if ("gencpp" in token or
                    re.search(r"(^|_)generate_messages(_|$)", token) or
                    ("EXPORTED_TARGETS" in token and
                     token != "${catkin_EXPORTED_TARGETS}"))
            }
            self.assertFalse(
                phantom_targets,
                "src/p450/%s/CMakeLists.txt: phantom generated-message "
                "target dependencies %s" % (
                    package, sorted(phantom_targets)),
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

    def test_package_dependency_closures_are_exact(self):
        for package, expected in EXPECTED_CATKIN_COMPONENTS.items():
            actual_cmake = _catkin_components(package)
            with self.subTest(package=package, declaration="CMake"):
                self.assertEqual(
                    expected,
                    actual_cmake,
                    "src/p450/%s/CMakeLists.txt: direct Catkin component "
                    "closure differs; expected=%s actual=%s" % (
                        package, sorted(expected), sorted(actual_cmake)),
                )
            manifest = _manifest_dependencies(package)
            for phase in ("build", "export", "exec"):
                expected_phase = EXPECTED_MANIFEST_PHASES[package][phase]
                with self.subTest(package=package, phase=phase):
                    self.assertEqual(
                        expected_phase,
                        manifest[phase],
                        "src/p450/%s/package.xml: %s dependency closure "
                        "differs; expected=%s actual=%s" % (
                            package, phase, sorted(expected_phase),
                            sorted(manifest[phase])),
                    )

        uav_manifest = _manifest_dependencies("prometheus_uav_control")
        self.assertIn(
            "prometheus_msgs",
            uav_manifest["build"],
            "src/p450/prometheus_uav_control/package.xml: missing build edge "
            "to prometheus_msgs",
        )

    def test_build_tool_and_system_dependency_surfaces_are_exact(self):
        for package, expected in EXPECTED_NON_CATKIN_PACKAGES.items():
            actual = frozenset(
                tokens[0]
                for tokens in _commands(package, "find_package")
                if tokens and tokens[0] != "catkin"
            )
            with self.subTest(package=package, dependency_kind="system"):
                self.assertEqual(
                    expected,
                    actual,
                    "src/p450/%s/CMakeLists.txt: non-Catkin find_package "
                    "surface differs; expected=%s actual=%s" % (
                        package, sorted(expected), sorted(actual)),
                )
            with self.subTest(package=package, dependency_kind="buildtool"):
                self.assertEqual(
                    frozenset({"catkin"}),
                    _manifest_buildtools(package),
                    "src/p450/%s/package.xml: build tool closure must be "
                    "exactly catkin" % package,
                )

        self.assertEqual(
            (("GEOGRAPHICLIB", "REQUIRED", "geographiclib"),),
            _commands("prometheus_uav_control", "pkg_check_modules"),
            "src/p450/prometheus_uav_control/CMakeLists.txt: GeographicLib "
            "must resolve through its available pkg-config module",
        )

    def test_catkin_package_exports_match_public_build_surface(self):
        for package, fields in EXPECTED_CATKIN_PACKAGE_FIELDS.items():
            for field, expected in fields.items():
                actual = _catkin_package_field(package, field)
                with self.subTest(package=package, field=field):
                    self.assertEqual(
                        expected,
                        actual,
                        "src/p450/%s/CMakeLists.txt: catkin_package %s "
                        "differs; expected=%s actual=%s" % (
                            package, field, sorted(expected), sorted(actual)),
                    )

    def test_gazebo_assets_declare_runtime_consumers(self):
        package = "prometheus_gazebo"
        self.assertEqual(frozenset(), _catkin_components(package))
        self.assertEqual(
            EXPECTED_MANIFEST_PHASES[package],
            _manifest_dependencies(package),
            "src/p450/prometheus_gazebo/package.xml: asset-only dependency "
            "closure must be exact",
        )

    def test_uav_control_requires_cxx17(self):
        self.assertTrue(
            _has_required_cxx17_contract("prometheus_uav_control"),
            "src/p450/prometheus_uav_control/CMakeLists.txt: every compiled "
            "target must require C++17",
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

        for package, expected in EXPECTED_INSTALL_DIRECTORIES.items():
            actual = frozenset(
                relative.rstrip("/")
                for relative in _install_directory_sources(package)
            )
            with self.subTest(package=package, contract="exact-resources"):
                self.assertEqual(
                    expected,
                    actual,
                    "src/p450/%s/CMakeLists.txt: installed directory surface "
                    "differs; expected=%s actual=%s" % (
                        package, sorted(expected), sorted(actual)),
                )

        for package, expected in EXPECTED_INSTALLED_FILES.items():
            actual = _installed_file_sources(package)
            with self.subTest(package=package, contract="installed-files"):
                self.assertEqual(
                    expected,
                    actual,
                    "src/p450/%s/CMakeLists.txt: installed file/program "
                    "surface differs; expected=%s actual=%s" % (
                        package, sorted(expected), sorted(actual)),
                )
            for relative in actual:
                with self.subTest(package=package, file=relative):
                    self.assertTrue(
                        (P450 / package / relative).is_file(),
                        "src/p450/%s/CMakeLists.txt: installed file does not "
                        "exist: %s" % (package, relative),
                    )

        for package, fields in EXPECTED_CATKIN_PACKAGE_FIELDS.items():
            installed_sources = tuple(
                (P450 / package / relative.rstrip("/")).resolve()
                for relative in _install_directory_sources(package)
            )
            for relative in fields["INCLUDE_DIRS"]:
                include_root = P450 / package / relative
                with self.subTest(package=package, public_include=relative):
                    self.assertTrue(include_root.is_dir())
                for header in include_root.rglob("*"):
                    if not header.is_file():
                        continue
                    self.assertTrue(
                        any(source == header.resolve() or
                            source in header.resolve().parents
                            for source in installed_sources),
                        "src/p450/%s/CMakeLists.txt: public header is not "
                        "covered by install(DIRECTORY): %s" % (
                            package, header.relative_to(P450 / package)),
                    )

        self.assertIn(
            "scripts/jinja_gen.py",
            _catkin_installed_programs("prometheus_gazebo"),
            "src/p450/prometheus_gazebo/CMakeLists.txt: jinja_gen.py must be "
            "installed as an executable program",
        )

    def test_runtime_targets_are_installed_and_realsense_is_exported(self):
        for package in EXPECTED_INSTALL_DIRECTORIES:
            installed = _installed_targets(package)
            built = frozenset(_targets(package))
            with self.subTest(package=package, contract="no-ghost-target"):
                self.assertFalse(
                    installed - built,
                    "src/p450/%s/CMakeLists.txt: install(TARGETS) contains "
                    "targets that are not built: %s" % (
                        package, sorted(installed - built)),
                )
        for package, expected in EXPECTED_TARGETS.items():
            with self.subTest(package=package):
                self.assertEqual(
                    frozenset(expected),
                    _installed_targets(package),
                    "src/p450/%s/CMakeLists.txt: installed target surface "
                    "must exactly match built targets" % package,
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


class NoeticEnvironmentWrapperContractTest(unittest.TestCase):
    def _require_wrapper(self):
        self.assertTrue(
            NOETIC_WRAPPER.is_file(),
            "scripts/with_noetic_env.bash must exist before the clean "
            "Catkin build",
        )

    def _run_wrapper(self, command):
        self._require_wrapper()
        poison = "P450_WRAPPER_POISON_91A7"
        ambient = {
            "HOME": "/%s/home" % poison,
            "USER": poison,
            "LOGNAME": poison,
            "SHELL": "/%s/bash" % poison,
            "LANG": "C",
            "LC_ALL": "C",
            "PATH": "/%s/bin:/usr/bin:/bin" % poison,
            "CMAKE_PREFIX_PATH": "/%s/cmake" % poison,
            "ROS_PACKAGE_PATH": "/%s/ros" % poison,
            "ROS_MASTER_URI": "http://%s:11311" % poison,
            "ROS_IP": poison,
            "ROS_HOSTNAME": poison,
            "PYTHONPATH": "/%s/python" % poison,
            "PYTHONHOME": "/%s/python-home" % poison,
            "LD_LIBRARY_PATH": "/%s/lib" % poison,
            "GAZEBO_MODEL_PATH": "/%s/models" % poison,
            "GAZEBO_PLUGIN_PATH": "/%s/plugins" % poison,
            "GAZEBO_RESOURCE_PATH": "/%s/resources" % poison,
            "DISPLAY": ":91",
            "CATKIN_PROFILE": poison,
            "AMENT_PREFIX_PATH": "/%s/ament" % poison,
            "COLCON_PREFIX_PATH": "/%s/colcon" % poison,
            "VIRTUAL_ENV": "/%s/venv" % poison,
            "CONDA_PREFIX": "/%s/conda" % poison,
            "P450_UNRELATED_SENTINEL": poison,
        }
        return subprocess.run(
            [str(NOETIC_WRAPPER)] + list(command),
            cwd=str(ROOT),
            env=ambient,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )

    def test_wrapper_is_executable_and_has_one_clean_setup_boundary(self):
        self._require_wrapper()
        mode = NOETIC_WRAPPER.lstat().st_mode
        self.assertTrue(stat.S_ISREG(mode))
        self.assertFalse(NOETIC_WRAPPER.is_symlink())
        self.assertTrue(mode & 0o111)

        text = NOETIC_WRAPPER.read_text(encoding="utf-8")
        self.assertEqual(1, text.count("source /opt/ros/noetic/setup.bash"))
        for contract in (
            "/usr/bin/env -i",
            "/bin/bash --noprofile --norc",
            'p450_command=("$@")',
            "set --",
            'exec "${p450_command[@]}"',
        ):
            self.assertIn(contract, text)
        self.assertNotIn("/home/lu", text)
        self.assertNotIn(str(ROOT), text)

    def test_wrapper_scrubs_ambient_workspaces_and_uses_local_state(self):
        result = self._run_wrapper((
            "/usr/bin/python3",
            "-c",
            "import json, os; print(json.dumps(dict(os.environ), "
            "sort_keys=True))",
        ))
        self.assertEqual(0, result.returncode, result.stderr)
        environment = json.loads(result.stdout)
        login = pwd.getpwuid(os.getuid())

        expected = {
            "HOME": login.pw_dir,
            "USER": login.pw_name,
            "LOGNAME": login.pw_name,
            "SHELL": "/bin/bash",
            "LANG": "C.UTF-8",
            "LC_ALL": "C.UTF-8",
            "PATH": "/opt/ros/noetic/bin:/usr/bin:/bin:/usr/sbin:/sbin",
            "ROS_HOME": str(ROOT / "logs/ros"),
            "ROS_LOG_DIR": str(ROOT / "logs/ros/log"),
            "GAZEBO_LOG_PATH": str(ROOT / "logs/gazebo"),
            "XDG_CONFIG_HOME": str(ROOT / "logs/xdg/config"),
            "XDG_CACHE_HOME": str(ROOT / "logs/xdg/cache"),
            "CMAKE_PREFIX_PATH": "/opt/ros/noetic",
            "ROS_PACKAGE_PATH": "/opt/ros/noetic/share",
            "ROS_DISTRO": "noetic",
        }
        for name, value in expected.items():
            with self.subTest(name=name):
                self.assertEqual(value, environment.get(name))

        poison = "P450_WRAPPER_POISON_91A7"
        self.assertFalse(
            any(poison in value for value in environment.values()),
            "ambient workspace value survived env -i",
        )
        for name in (
            "AMENT_PREFIX_PATH",
            "CATKIN_PROFILE",
            "COLCON_PREFIX_PATH",
            "CONDA_PREFIX",
            "DISPLAY",
            "GAZEBO_MODEL_PATH",
            "GAZEBO_PLUGIN_PATH",
            "GAZEBO_RESOURCE_PATH",
            "P450_UNRELATED_SENTINEL",
            "PYTHONHOME",
            "ROS_HOSTNAME",
            "ROS_IP",
            "ROS_MASTER_URI",
            "VIRTUAL_ENV",
        ):
            with self.subTest(scrubbed=name):
                self.assertNotIn(name, environment)

        for path in (
            ROOT / "logs/ros",
            ROOT / "logs/ros/log",
            ROOT / "logs/gazebo",
            ROOT / "logs/xdg/config",
            ROOT / "logs/xdg/cache",
        ):
            with self.subTest(directory=path.relative_to(ROOT).as_posix()):
                self.assertTrue(path.is_dir())

    def test_wrapper_preserves_arguments_and_command_exit_status(self):
        expected_arguments = ["--help", "--extend", "--local", "two words"]
        result = self._run_wrapper((
            "/usr/bin/python3",
            "-c",
            "import json, sys; print(json.dumps(sys.argv[1:])); "
            "raise SystemExit(37)",
        ) + tuple(expected_arguments))
        self.assertEqual(37, result.returncode, result.stderr)
        self.assertEqual(expected_arguments, json.loads(result.stdout))


if __name__ == "__main__":
    unittest.main()
