import os
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


FRAME_PREFIX = "ground/"
XACRO_EXECUTABLE = "/opt/ros/noetic/bin/xacro"
ROS_PYTHON_PATH = "/opt/ros/noetic/lib/python3/dist-packages"
LOCAL_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,127}$")
INVALID_NAME_RUN_RE = re.compile(r"[^A-Za-z0-9_]+")
BASE_COLLISION_ORIGIN = "0.018058912 0.001357451 -0.160420741"
BASE_COLLISION_SIZE = "1.026335219 0.782744936 0.395154782"
INCOMPATIBLE_FIXED_JOINT_TAGS = (
    "axis", "limit", "dynamics", "calibration", "mimic",
    "safety_controller",
)
EXPECTED_CONTROLLED_JOINTS = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    "left_outer_knuckle_joint",
}
EXPECTED_PLUGIN_LIBRARIES = {
    "libbunker_planar_move_plugin.so",
    "libgazebo_ros_laser.so",
    "libgazebo_ros_control.so",
    "libroboticsgroup_gazebo_mimic_joint_plugin.so",
    "libgazebo_ros_camera.so",
    "libgazebo_ros_openni_kinect.so",
}
EXPECTED_SENSORS = {
    "bunker_lidar_2d", "ground_d435_color", "ground_d435_depth",
}


class RenderError(ValueError):
    pass


def canonical_local_name(value):
    if not isinstance(value, str) or not value:
        raise RenderError("robot name is empty")
    mapped = INVALID_NAME_RUN_RE.sub("_", value)
    if LOCAL_NAME_RE.fullmatch(mapped) is None:
        raise RenderError("robot name cannot be canonicalized: %r" % value)
    return mapped


def _declaration_map(elements, label, prefix=""):
    mapping = {}
    outputs = {}
    for element in elements:
        source = element.get("name")
        if not isinstance(source, str) or source in mapping:
            raise RenderError("duplicate or missing %s name" % label)
        target = prefix + canonical_local_name(source)
        if target in outputs:
            raise RenderError(
                "%s name collision: %s and %s" %
                (label, outputs[target], source))
        mapping[source] = target
        outputs[target] = source
    return mapping


def _map_required(mapping, value, label):
    if value not in mapping:
        raise RenderError("%s references unknown name %r" % (label, value))
    return mapping[value]


def _replace_bunker_collisions(links, link_sources):
    for link in links:
        source = link_sources[link]
        if source == "base_link" or source.startswith("wheel"):
            for collision in tuple(link.findall("collision")):
                link.remove(collision)
    base_link = next(
        link for link in links if link_sources[link] == "base_link")
    base_link.append(ET.fromstring(
        '<collision name="base_link_collision">'
        '<origin xyz="%s" rpy="0 0 0"/>'
        '<geometry><box size="%s"/></geometry>'
        '</collision>' % (BASE_COLLISION_ORIGIN, BASE_COLLISION_SIZE)))


def transform_robot_tree(root):
    if root.tag != "robot" or root.get("name") != "bunker_aubo":
        raise RenderError("unexpected robot root")
    links = root.findall("link")
    joints = root.findall("joint")
    if not links or not joints:
        raise RenderError("robot must contain links and joints")

    link_sources = {link: link.get("name") for link in links}
    joint_sources = {joint: joint.get("name") for joint in joints}
    link_map = _declaration_map(links, "link", FRAME_PREFIX)
    joint_map = _declaration_map(joints, "joint")
    sdf_joints = root.findall("./gazebo/joint")
    sdf_joint_sources = {joint: joint.get("name") for joint in sdf_joints}
    sdf_joint_map = _declaration_map(sdf_joints, "Gazebo joint")

    _replace_bunker_collisions(links, link_sources)
    for link in links:
        link.set("name", link_map[link_sources[link]])

    for joint in joints:
        source = joint_sources[joint]
        joint.set("name", joint_map[source])
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise RenderError("URDF joint lacks parent or child")
        parent.set(
            "link", _map_required(link_map, parent.get("link"), "joint parent"))
        child.set(
            "link", _map_required(link_map, child.get("link"), "joint child"))
        mimic = joint.find("mimic")
        if mimic is not None:
            mimic.set(
                "joint", _map_required(
                    joint_map, mimic.get("joint"), "joint mimic"))
        if source.startswith("wheel"):
            joint.set("type", "fixed")
            for tag in INCOMPATIBLE_FIXED_JOINT_TAGS:
                for element in tuple(joint.findall(tag)):
                    joint.remove(element)

    for transmission_joint in root.findall("./transmission/joint"):
        transmission_joint.set(
            "name", _map_required(
                joint_map, transmission_joint.get("name"),
                "transmission joint"))

    for gazebo in root.findall("gazebo"):
        reference = gazebo.get("reference")
        if reference is not None:
            if reference in link_map:
                gazebo.set("reference", link_map[reference])
            elif reference in joint_map:
                gazebo.set("reference", joint_map[reference])
            elif reference in sdf_joint_map:
                gazebo.set("reference", sdf_joint_map[reference])
            else:
                raise RenderError(
                    "Gazebo reference is unknown: %s" % reference)
        for sdf_joint in gazebo.findall("joint"):
            sdf_joint.set("name", sdf_joint_map[sdf_joint_sources[sdf_joint]])
            for tag in ("parent", "child"):
                element = sdf_joint.find(tag)
                if element is None or element.text is None:
                    raise RenderError("Gazebo joint lacks %s" % tag)
                source = element.text.strip()
                element.text = _map_required(
                    link_map, source, "Gazebo joint %s" % tag)
        for plugin in gazebo.findall("plugin"):
            for tag in ("joint", "mimicJoint"):
                element = plugin.find(tag)
                if element is None or element.text is None:
                    continue
                source = element.text.strip()
                element.text = _map_required(
                    joint_map, source, "plugin %s" % tag)
    return root


def _require_plugin(root, filename):
    matches = [
        plugin for plugin in root.findall(".//plugin")
        if plugin.get("filename") == filename]
    if len(matches) != 1:
        raise RenderError("expected one plugin %s" % filename)
    return matches[0]


def validate_runtime_tree(root):
    if root.tag != "robot" or root.get("name") != "bunker_aubo":
        raise RenderError("unexpected runtime robot root")
    links = root.findall("link")
    joints = root.findall("joint")
    link_names = [link.get("name") for link in links]
    joint_names = [joint.get("name") for joint in joints]
    if (not links or not joints or len(link_names) != len(set(link_names)) or
            len(joint_names) != len(set(joint_names))):
        raise RenderError("runtime declarations are missing or duplicated")
    if any(not isinstance(name, str) or not name.startswith(FRAME_PREFIX)
           for name in link_names):
        raise RenderError("runtime link lacks the ground frame prefix")
    if any(not isinstance(name, str) or LOCAL_NAME_RE.fullmatch(name) is None
           for name in joint_names):
        raise RenderError("runtime joint name is invalid")

    link_set = set(link_names)
    child_links = set()
    for joint in joints:
        parent = joint.find("parent")
        child = joint.find("child")
        if (parent is None or child is None or
                parent.get("link") not in link_set or
                child.get("link") not in link_set):
            raise RenderError("runtime joint reference is invalid")
        child_links.add(child.get("link"))
    if link_set - child_links != {"ground/base_link"}:
        raise RenderError("runtime must have one ground/base_link root")

    required_links = {
        "ground/base_link", "ground/aubo_i5_base_link", "ground/ee_link",
        "ground/ag95_base_link", "ground/gripper_tcp_link",
        "ground/d435_link", "ground/d435_color_optical_frame",
        "ground/d435_depth_optical_frame", "ground/lidar_2d_link",
    }
    if not required_links.issubset(link_set):
        raise RenderError("runtime robot links are incomplete")
    if not EXPECTED_CONTROLLED_JOINTS.issubset(set(joint_names)):
        raise RenderError("runtime controlled joints are incomplete")

    transmissions = root.findall("transmission")
    transmission_joints = {
        transmission.find("joint").get("name")
        for transmission in transmissions
        if transmission.find("joint") is not None
    }
    if (len(transmissions) != len(EXPECTED_CONTROLLED_JOINTS) or
            transmission_joints != EXPECTED_CONTROLLED_JOINTS):
        raise RenderError("runtime transmissions differ")

    plugins = root.findall(".//plugin")
    plugin_libraries = {plugin.get("filename") for plugin in plugins}
    if (len(plugins) != len(EXPECTED_PLUGIN_LIBRARIES) or
            plugin_libraries != EXPECTED_PLUGIN_LIBRARIES):
        raise RenderError("runtime plugin set differs")
    sensors = root.findall(".//sensor")
    if ({sensor.get("name") for sensor in sensors} != EXPECTED_SENSORS or
            len(sensors) != len(EXPECTED_SENSORS)):
        raise RenderError("runtime sensor set differs")

    base = root.find("./link[@name='ground/base_link']")
    base_collisions = base.findall("collision") if base is not None else []
    if len(base_collisions) != 1:
        raise RenderError("runtime base collision differs")
    base_collision = base_collisions[0]
    if (base_collision.get("name") != "base_link_collision" or
            base_collision.find("origin") is None or
            base_collision.find("origin").get("xyz") != BASE_COLLISION_ORIGIN or
            base_collision.find("geometry/box") is None or
            base_collision.find("geometry/box").get("size") !=
            BASE_COLLISION_SIZE):
        raise RenderError("runtime base collision differs")

    planar = _require_plugin(root, "libbunker_planar_move_plugin.so")
    if (
        planar.findtext("robotNamespace") != "/ground" or
        planar.findtext("commandTopic") != "cmd_vel_safe" or
        planar.findtext("odometryTopic") != "odom" or
        planar.findtext("odometryFrame") != "ground/odom" or
        planar.findtext("robotBaseFrame") != "ground/base_link"
    ):
        raise RenderError("runtime planar interface differs")
    laser = _require_plugin(root, "libgazebo_ros_laser.so")
    if (laser.findtext("robotNamespace") != "/ground" or
            laser.findtext("topicName") != "scan" or
            laser.findtext("frameName") != "ground/lidar_2d_link"):
        raise RenderError("runtime laser interface differs")
    control = _require_plugin(root, "libgazebo_ros_control.so")
    if (control.findtext("robotNamespace") != "/ground" or
            control.findtext("robotParam") != "/ground/robot_description"):
        raise RenderError("runtime control interface differs")
    color = _require_plugin(root, "libgazebo_ros_camera.so")
    depth = _require_plugin(root, "libgazebo_ros_openni_kinect.so")
    if (color.findtext("robotNamespace") != "/ground" or
            color.findtext("cameraName") != "d435/color" or
            color.findtext("imageTopicName") != "image_raw" or
            color.findtext("cameraInfoTopicName") !=
            "camera_info" or
            color.findtext("frameName") !=
            "ground/d435_color_optical_frame" or
            depth.findtext("robotNamespace") != "/ground" or
            depth.findtext("cameraName") != "d435/depth" or
            depth.findtext("depthImageTopicName") !=
            "image_raw" or
            depth.findtext("depthImageCameraInfoTopicName") !=
            "camera_info" or
            depth.findtext("pointCloudTopicName") != "points" or
            depth.findtext("frameName") !=
            "ground/d435_depth_optical_frame"):
        raise RenderError("runtime D435 interface differs")

    lowered = ET.tostring(root, encoding="unicode").lower()
    for token in (
            "brick", "handoff", "attachment", "benchmark", "provenance",
            "lifecycle"):
        if token in lowered:
            raise RenderError("task-specific token in runtime: %s" % token)
    return root


def _run_xacro(path):
    source = Path(path).absolute()
    if not source.is_file():
        raise RenderError("xacro file is missing: %s" % source)
    environment = dict(os.environ)
    python_paths = [
        item for item in environment.get("PYTHONPATH", "").split(os.pathsep)
        if item]
    if ROS_PYTHON_PATH not in python_paths:
        python_paths.append(ROS_PYTHON_PATH)
    environment["PYTHONPATH"] = os.pathsep.join(python_paths)
    try:
        result = subprocess.run(
            [XACRO_EXECUTABLE, str(source)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            check=False, timeout=20.0, env=environment)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise RenderError("xacro execution failed: %s" % error)
    if result.returncode != 0:
        detail = result.stderr.decode("utf-8", "replace").strip()
        raise RenderError(
            "xacro returned %d: %s" % (result.returncode, detail))
    return result.stdout


def _indent_tree(element, level=0):
    prefix = "\n" + "  " * level
    child_prefix = "\n" + "  " * (level + 1)
    children = list(element)
    if children:
        if element.text is None or not element.text.strip():
            element.text = child_prefix
        for child in children:
            _indent_tree(child, level + 1)
        if children[-1].tail is None or not children[-1].tail.strip():
            children[-1].tail = prefix
    if level and (element.tail is None or not element.tail.strip()):
        element.tail = prefix


def render_ground_robot(path, validate=True):
    try:
        root = ET.fromstring(_run_xacro(path))
    except ET.ParseError as error:
        raise RenderError("expanded xacro is invalid XML: %s" % error)
    transform_robot_tree(root)
    if validate:
        validate_runtime_tree(root)
    _indent_tree(root)
    payload = ET.tostring(
        root, encoding="utf-8", xml_declaration=True,
        short_empty_elements=True).decode("utf-8")
    return payload.rstrip("\n") + "\n"
