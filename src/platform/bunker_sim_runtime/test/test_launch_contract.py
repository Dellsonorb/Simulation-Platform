import shlex
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
PACKAGE = ROOT / "src/platform/bunker_sim_runtime"
RUNTIME_LAUNCH = PACKAGE / "launch/bunker_runtime.launch"
STANDALONE_LAUNCH = PACKAGE / "launch/bunker_standalone.launch"
WORLD = PACKAGE / "worlds/bunker_standalone.world"
TF_FIXTURE = PACKAGE / "test/tf_prefix_fixture.urdf"
TF_ROSTEST = PACKAGE / "test/tf_prefix_contract.test"

RUNTIME_ARGS = {
    "x": "0.0", "y": "0.0", "z": "0.36",
    "roll": "0.0", "pitch": "0.0", "yaw": "0.0",
}
STANDALONE_ARGS = {"gui": "false"}
EXPECTED_NODES = {
    "velocity_guard": ("bunker_sim_runtime", "velocity_guard.py"),
    "spawn_bunker": ("gazebo_ros", "spawn_model"),
    "robot_state_publisher": (
        "robot_state_publisher", "robot_state_publisher"),
    "world_to_odom": ("tf2_ros", "static_transform_publisher"),
}
EXPECTED_SPAWN_PREFIX = (
    "$(find bunker_sim_runtime)/scripts/spawn_bunker_preflight.py "
    "$(arg x) $(arg y) $(arg z) $(arg roll) $(arg pitch) $(arg yaw) --"
)
EXPECTED_SPAWN_ARGS = (
    "-urdf -param robot_description -model bunker -b "
    "-x $(arg x) -y $(arg y) -z $(arg z) "
    "-R $(arg roll) -P $(arg pitch) -Y $(arg yaw)"
)
FORBIDDEN_TOKENS = (
    "rviz", "navigation", "move_base", "controller", "aubo", "ag95",
    "d435", "attachment", "orchestration", "brick", "benchmark",
)


def _parse(path):
    return ET.parse(str(path)).getroot()


def _direct_arguments(root):
    return {item.get("name"): item.get("default")
            for item in root.findall("arg")}


def _signature(element):
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        tuple(_signature(child) for child in list(element)),
    )


class LaunchContractTest(unittest.TestCase):
    def test_worldless_runtime_surface_and_identity_are_exact(self):
        root = _parse(RUNTIME_LAUNCH)
        self.assertEqual("launch", root.tag)
        self.assertEqual(RUNTIME_ARGS, _direct_arguments(root))
        self.assertEqual(1, len(root.findall("group")))
        self.assertEqual([], root.findall("include"))
        self.assertEqual([], root.findall("node"))
        group = root.find("group")
        self.assertEqual({"ns": "ground"}, group.attrib)
        self.assertNotIn("use_sim_time", {
            param.get("name") for param in root.iter("param")})

        parameters = {item.get("name"): item for item in group.findall("param")}
        self.assertEqual({"tf_prefix", "robot_description"}, set(parameters))
        self.assertEqual(
            {"name": "tf_prefix", "type": "str", "value": "ground"},
            parameters["tf_prefix"].attrib,
        )
        self.assertEqual(
            "$(find bunker_sim_runtime)/scripts/render_bunker_runtime.py "
            "$(find bunker_description)/urdf/bunker.urdf.xacro",
            parameters["robot_description"].get("command"),
        )
        self.assertEqual(
            {"name", "command"}, set(parameters["robot_description"].attrib))

    def test_runtime_has_only_the_four_owned_nodes(self):
        group = _parse(RUNTIME_LAUNCH).find("group")
        nodes = {item.get("name"): item for item in group.findall("node")}
        self.assertEqual(EXPECTED_NODES, {
            name: (item.get("pkg"), item.get("type"))
            for name, item in nodes.items()
        })
        self.assertEqual(4, len(nodes))
        self.assertEqual("true", nodes["velocity_guard"].get("required"))
        self.assertEqual("true", nodes["spawn_bunker"].get("required"))
        self.assertEqual("screen", nodes["velocity_guard"].get("output"))
        self.assertEqual("screen", nodes["spawn_bunker"].get("output"))
        self.assertEqual([], nodes["robot_state_publisher"].findall("param"))
        self.assertEqual([], list(nodes["robot_state_publisher"]))
        self.assertEqual(
            "0 0 0 0 0 0 1 world ground/odom",
            nodes["world_to_odom"].get("args"),
        )

    def test_spawn_prefix_and_bonded_command_are_exact(self):
        spawn = next(
            item for item in _parse(RUNTIME_LAUNCH).iter("node")
            if item.get("name") == "spawn_bunker")
        self.assertEqual(EXPECTED_SPAWN_PREFIX, spawn.get("launch-prefix"))
        self.assertEqual(EXPECTED_SPAWN_ARGS, spawn.get("args"))
        tokens = shlex.split(spawn.get("args"))
        self.assertEqual(1, tokens.count("-b"))
        self.assertEqual(["-model", "bunker"], tokens[3:5])
        self.assertEqual(1, sum(
            item.get("pkg") == "gazebo_ros" and
            item.get("type") == "spawn_model"
            for item in _parse(RUNTIME_LAUNCH).iter("node")))

    def test_standalone_wrapper_has_only_gui_and_two_exact_includes(self):
        root = _parse(STANDALONE_LAUNCH)
        self.assertEqual(STANDALONE_ARGS, _direct_arguments(root))
        self.assertEqual([], root.findall("node"))
        includes = root.findall("include")
        self.assertEqual(2, len(includes))
        gazebo = next(
            item for item in includes
            if item.get("file") ==
            "$(find gazebo_ros)/launch/empty_world.launch")
        runtime = next(
            item for item in includes
            if item.get("file") ==
            "$(find bunker_sim_runtime)/launch/bunker_runtime.launch")
        self.assertEqual([], list(runtime))
        self.assertEqual(set(), runtime.attrib.keys() - {"file"})
        forwarded = {
            item.get("name"): item.get("value") for item in gazebo.findall("arg")}
        self.assertEqual({
            "world_name": (
                "$(find bunker_sim_runtime)/worlds/bunker_standalone.world"),
            "gui": "$(arg gui)",
            "paused": "false",
            "debug": "false",
            "verbose": "false",
            "use_sim_time": "true",
            "server_required": "true",
        }, forwarded)
        self.assertEqual(1, sum(
            item.get("file") ==
            "$(find bunker_sim_runtime)/launch/bunker_runtime.launch"
            for item in includes))

    def test_world_is_the_minimal_scan_scene(self):
        root = _parse(WORLD)
        self.assertEqual("sdf", root.tag)
        self.assertEqual("1.6", root.get("version"))
        self.assertEqual(1, len(root.findall("world")))
        world = root.find("world")
        self.assertEqual("bunker_standalone", world.get("name"))
        self.assertEqual(["include", "include", "model"], [
            item.tag for item in list(world)])
        self.assertEqual(
            ["model://ground_plane", "model://sun"],
            [item.findtext("uri") for item in world.findall("include")],
        )
        expected = ET.fromstring(
            '<model name="scan_obstacle">'
            '<static>true</static><pose>2.0 0.0 0.5 0 0 0</pose>'
            '<link name="link"><collision name="collision">'
            '<geometry><box><size>0.5 1.0 1.0</size></box></geometry>'
            '</collision><visual name="visual">'
            '<geometry><box><size>0.5 1.0 1.0</size></box></geometry>'
            '</visual></link></model>')
        self.assertEqual(_signature(expected), _signature(world.find("model")))

    def test_runtime_sources_exclude_unrelated_systems(self):
        text = "\n".join(path.read_text(encoding="utf-8").lower()
                         for path in (RUNTIME_LAUNCH, STANDALONE_LAUNCH, WORLD))
        for token in FORBIDDEN_TOKENS:
            with self.subTest(token=token):
                self.assertNotIn(token, text)

    def test_tf_prefix_fixture_and_rostest_are_staged_exactly(self):
        fixture = _parse(TF_FIXTURE)
        self.assertEqual("robot", fixture.tag)
        self.assertEqual(
            ["base_link", "lidar_2d_link"],
            [item.get("name") for item in fixture.findall("link")],
        )
        self.assertEqual(1, len(fixture.findall("joint")))
        joint = fixture.find("joint")
        self.assertEqual("tf_prefix_fixture_joint", joint.get("name"))
        self.assertEqual("fixed", joint.get("type"))
        self.assertEqual("-0.30 0.0 0.25", joint.find("origin").get("xyz"))
        self.assertEqual("0 0 0", joint.find("origin").get("rpy"))
        self.assertEqual("base_link", joint.find("parent").get("link"))
        self.assertEqual("lidar_2d_link", joint.find("child").get("link"))

        launch = _parse(TF_ROSTEST)
        group = launch.find("group")
        self.assertEqual("ground", group.get("ns"))
        params = {item.get("name"): item for item in group.findall("param")}
        self.assertEqual({"tf_prefix", "robot_description"}, set(params))
        self.assertEqual("ground", params["tf_prefix"].get("value"))
        self.assertEqual(
            "$(find bunker_sim_runtime)/test/tf_prefix_fixture.urdf",
            params["robot_description"].get("textfile"),
        )
        nodes = group.findall("node")
        self.assertEqual(1, len(nodes))
        self.assertEqual(
            ("robot_state_publisher", "robot_state_publisher",
             "robot_state_publisher"),
            (nodes[0].get("pkg"), nodes[0].get("type"),
             nodes[0].get("name")),
        )
        tests = launch.findall("test")
        self.assertEqual(1, len(tests))
        self.assertEqual("bunker_sim_runtime", tests[0].get("pkg"))
        self.assertEqual("test_tf_prefix_contract.py", tests[0].get("type"))


if __name__ == "__main__":
    unittest.main()
