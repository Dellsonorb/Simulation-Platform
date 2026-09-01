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
            else:
                raise RenderError(
                    "Gazebo reference is unknown: %s" % reference)
        for sdf_joint in gazebo.findall("joint"):
            sdf_joint.set(
                "name", canonical_local_name(sdf_joint.get("name")))
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


def render_ground_robot(path):
    try:
        root = ET.fromstring(_run_xacro(path))
    except ET.ParseError as error:
        raise RenderError("expanded xacro is invalid XML: %s" % error)
    transform_robot_tree(root)
    _indent_tree(root)
    payload = ET.tostring(
        root, encoding="utf-8", xml_declaration=True,
        short_empty_elements=True).decode("utf-8")
    return payload.rstrip("\n") + "\n"
