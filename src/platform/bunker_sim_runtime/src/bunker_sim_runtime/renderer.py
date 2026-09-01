import os
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


XACRO_EXECUTABLE = "/opt/ros/noetic/bin/xacro"
ROS_PYTHON_PATH = "/opt/ros/noetic/lib/python3/dist-packages"
MESH_URI_PREFIX = "package://bunker_description/"
LOCAL_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,63}$")
INVALID_NAME_RUN_RE = re.compile(r"[^A-Za-z0-9_]+")
EXPECTED_BOX_ORIGIN = "0.018058912 0.001357451 -0.160420741"
EXPECTED_BOX_SIZE = "1.026335219 0.782744936 0.395154782"
INCOMPATIBLE_JOINT_TAGS = (
    "axis", "limit", "dynamics", "calibration", "mimic",
    "safety_controller")
COLLISION_XML = """<collision name="base_link_collision">
  <origin xyz="0.018058912 0.001357451 -0.160420741" rpy="0 0 0" />
  <geometry><box size="1.026335219 0.782744936 0.395154782" /></geometry>
</collision>"""
BASE_FRICTION_XML = """<gazebo reference="base_link">
  <mu1>0.0</mu1>
  <mu2>0.0</mu2>
</gazebo>"""
LIDAR_JOINT_XML = """<joint name="lidar_2d_joint" type="fixed">
  <origin xyz="-0.30 0.0 0.25" rpy="0 0 0" />
  <parent link="base_link" />
  <child link="lidar_2d_link" />
</joint>"""
LIDAR_GAZEBO_XML = """<gazebo reference="lidar_2d_link">
  <sensor name="bunker_lidar_2d" type="ray">
    <always_on>true</always_on>
    <update_rate>15.0</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.141592653589793</min_angle>
          <max_angle>3.141592653589793</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.12</min>
        <max>8.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0</mean>
        <stddev>0.002</stddev>
      </noise>
    </ray>
    <plugin name="bunker_laser" filename="libgazebo_ros_laser.so">
      <robotNamespace>/ground</robotNamespace>
      <topicName>scan</topicName>
      <frameName>lidar_2d_link</frameName>
    </plugin>
  </sensor>
</gazebo>"""
PLANAR_GAZEBO_XML = """<gazebo>
  <plugin name="bunker_planar_move" filename="libbunker_planar_move_plugin.so">
    <robotNamespace>/ground</robotNamespace>
    <commandTopic>cmd_vel_safe</commandTopic>
    <odometryTopic>odom</odometryTopic>
    <odometryFrame>odom</odometryFrame>
    <robotBaseFrame>base_link</robotBaseFrame>
    <odometryRate>50.0</odometryRate>
    <cmdTimeout>0.5</cmdTimeout>
  </plugin>
</gazebo>"""


class RenderContractError(ValueError):
    pass


def _validated_source(source_path):
    source = Path(source_path).absolute()
    if source.name != "bunker.urdf.xacro" or source.parent.name != "urdf":
        raise RenderContractError(
            "input must be urdf/bunker.urdf.xacro")
    package_root = source.parent.parent
    if not source.is_file() or not package_root.is_dir():
        raise RenderContractError("BUNKER xacro or package directory is missing")
    return source, package_root


def _validate_mesh_references(root, package_root):
    for mesh in root.findall(".//mesh"):
        uri = mesh.get("filename")
        if type(uri) is not str or not uri.startswith(MESH_URI_PREFIX):
            raise RenderContractError(
                "unsupported source mesh URI: %r" % uri)
        relative_text = uri[len(MESH_URI_PREFIX):]
        relative = Path(relative_text)
        if (relative.is_absolute() or not relative.parts or
                any(part in ("", "..") for part in relative.parts)):
            raise RenderContractError("unsafe source mesh URI: %s" % uri)
        if not (package_root / relative).is_file():
            raise RenderContractError("missing source mesh: %s" % uri)


def run_xacro(path, executable=XACRO_EXECUTABLE):
    if (type(executable) is not str or not os.path.isfile(executable) or
            not os.access(executable, os.X_OK)):
        raise RenderContractError("xacro executable is unavailable")
    child_environment = dict(os.environ)
    python_paths = [
        item for item in child_environment.get("PYTHONPATH", "").split(
            os.pathsep) if item]
    if ROS_PYTHON_PATH not in python_paths:
        python_paths.append(ROS_PYTHON_PATH)
    child_environment["PYTHONPATH"] = os.pathsep.join(python_paths)
    try:
        result = subprocess.run(
            [executable, str(path)], stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, check=False, timeout=10.0,
            env=child_environment)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise RenderContractError("xacro execution failed: %s" % error)
    if result.returncode != 0:
        detail = result.stderr.decode("utf-8", "replace").strip()
        raise RenderContractError(
            "xacro returned %d: %s" % (result.returncode, detail))
    return result.stdout


def canonical_local_name(value):
    if type(value) is not str or not value:
        raise RenderContractError("empty source name")
    mapped = INVALID_NAME_RUN_RE.sub("_", value)
    if LOCAL_NAME_RE.fullmatch(mapped) is None:
        raise RenderContractError("name cannot be canonicalized: %r" % value)
    return mapped


def _declaration_map(elements, label):
    mapping = {}
    outputs = {}
    for element in elements:
        source = element.get("name")
        if type(source) is not str or source in mapping:
            raise RenderContractError("duplicate or missing %s name" % label)
        target = canonical_local_name(source)
        if target in outputs:
            raise RenderContractError(
                "%s name collision: %s and %s" %
                (label, outputs[target], source))
        mapping[source] = target
        outputs[target] = source
    return mapping


def validate_source_tree(root, source_path):
    _, package_root = _validated_source(source_path)
    if root.tag != "robot" or root.get("name") != "bunker_description":
        raise RenderContractError("unexpected source robot root")
    links = root.findall("link")
    joints = root.findall("joint")
    if not links or not joints or not any(
            link.get("name") == "base_link" for link in links):
        raise RenderContractError("source must contain links, joints, and base_link")
    if root.findall(".//plugin"):
        raise RenderContractError("source plugins conflict with runtime plugins")
    _declaration_map(links, "link")
    _declaration_map(joints, "joint")
    _validate_mesh_references(root, package_root)
    return root


def remap_urdf_names(root):
    links = root.findall("link")
    joints = root.findall("joint")
    link_map = _declaration_map(links, "link")
    joint_map = _declaration_map(joints, "joint")
    if set(link_map.values()) & set(joint_map.values()):
        raise RenderContractError("mapped link and joint names collide")
    for link in links:
        link.set("name", link_map[link.get("name")])
    for joint in joints:
        joint.set("name", joint_map[joint.get("name")])
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise RenderContractError("joint lacks parent or child")
        for reference in (parent, child):
            source = reference.get("link")
            if source not in link_map:
                raise RenderContractError("joint references unknown link")
            reference.set("link", link_map[source])
        mimic = joint.find("mimic")
        if mimic is not None:
            source = mimic.get("joint")
            if source not in joint_map:
                raise RenderContractError("mimic references unknown joint")
            mimic.set("joint", joint_map[source])
    for gazebo in root.findall("gazebo"):
        if "reference" in gazebo.attrib:
            source = gazebo.get("reference")
            if source not in link_map:
                raise RenderContractError("gazebo references unknown link")
            gazebo.set("reference", link_map[source])
    return root


def make_base_model_joints_fixed(root):
    joints = root.findall("joint")
    for joint in joints:
        joint.set("type", "fixed")
        for tag in INCOMPATIBLE_JOINT_TAGS:
            for child in tuple(joint.findall(tag)):
                joint.remove(child)
    return root


def replace_collisions(root):
    links = root.findall("link")
    for link in links:
        for collision in tuple(link.findall("collision")):
            link.remove(collision)
    base_link = next(
        (link for link in links if link.get("name") == "base_link"), None)
    if base_link is None:
        raise RenderContractError("base_link is missing")
    base_link.append(ET.fromstring(COLLISION_XML))
    return root


def add_base_friction(root):
    if any(gazebo.get("reference") == "base_link"
           for gazebo in root.findall("gazebo")):
        raise RenderContractError("base_link Gazebo properties already exist")
    root.append(ET.fromstring(BASE_FRICTION_XML))
    return root


def add_lidar(root):
    if any(element.get("name") in {"lidar_2d_link", "lidar_2d_joint"}
           for element in root.findall("link") + root.findall("joint")):
        raise RenderContractError("LiDAR entity already exists")
    root.append(ET.Element("link", {"name": "lidar_2d_link"}))
    root.append(ET.fromstring(LIDAR_JOINT_XML))
    root.append(ET.fromstring(LIDAR_GAZEBO_XML))
    return root


def add_planar_plugin(root):
    if root.find(".//plugin[@name='bunker_laser']") is None:
        raise RenderContractError("laser plugin is missing")
    if root.find(".//plugin[@name='bunker_planar_move']") is not None:
        raise RenderContractError("planar move plugin already exists")
    root.append(ET.fromstring(PLANAR_GAZEBO_XML))
    return root


def _signature(element):
    return (
        element.tag, tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        tuple(_signature(child) for child in list(element)),
    )


def _require_signature(actual, expected_xml, label):
    expected = ET.fromstring(expected_xml)
    if actual is None or _signature(actual) != _signature(expected):
        raise RenderContractError("%s subtree differs" % label)


def validate_runtime_tree(root):
    if root.tag != "robot" or root.get("name") != "bunker_description":
        raise RenderContractError("runtime robot root changed")
    links = root.findall("link")
    joints = root.findall("joint")
    collisions = root.findall(".//collision")
    sensors = root.findall(".//sensor")
    plugins = root.findall(".//plugin")
    if not links or not joints or len(collisions) != 1:
        raise RenderContractError("runtime robot tree or collision is missing")
    for label, elements in (
            ("link", links), ("joint", joints),
            ("collision", collisions), ("sensor", sensors),
            ("plugin", plugins)):
        names = [element.get("name") for element in elements]
        if any(type(name) is not str or LOCAL_NAME_RE.fullmatch(name) is None
               for name in names):
            raise RenderContractError("invalid runtime %s name" % label)
        if len(names) != len(set(names)):
            raise RenderContractError("duplicate runtime %s name" % label)
    link_names = {link.get("name") for link in links}
    child_names = set()
    for joint in joints:
        if joint.get("type") != "fixed":
            raise RenderContractError("all runtime joints must be fixed")
        if any(joint.find(tag) is not None for tag in INCOMPATIBLE_JOINT_TAGS):
            raise RenderContractError(
                "fixed joint retains incompatible child")
        parent = joint.find("parent")
        child = joint.find("child")
        if (parent is None or child is None or
                parent.get("link") not in link_names or
                child.get("link") not in link_names):
            raise RenderContractError("runtime joint reference is invalid")
        child_names.add(child.get("link"))
    if link_names - child_names != {"base_link"}:
        raise RenderContractError("runtime must have one base_link root")
    base_link = next(
        link for link in links if link.get("name") == "base_link")
    _require_signature(
        base_link.find("collision"), COLLISION_XML, "base collision")
    lidar_joint = next(
        (joint for joint in joints
         if joint.get("name") == "lidar_2d_joint"), None)
    _require_signature(lidar_joint, LIDAR_JOINT_XML, "LiDAR joint")
    gazebos = root.findall("gazebo")
    base_gazebo = next(
        (gazebo for gazebo in gazebos
         if gazebo.get("reference") == "base_link"), None)
    lidar_gazebo = next(
        (gazebo for gazebo in gazebos
         if gazebo.get("reference") == "lidar_2d_link"), None)
    planar_gazebo = next(
        (gazebo for gazebo in gazebos if not gazebo.attrib), None)
    _require_signature(lidar_gazebo, LIDAR_GAZEBO_XML, "LiDAR Gazebo")
    _require_signature(base_gazebo, BASE_FRICTION_XML, "base friction Gazebo")
    _require_signature(planar_gazebo, PLANAR_GAZEBO_XML, "planar Gazebo")
    if {plugin.get("name"): plugin.get("filename") for plugin in plugins} != {
            "bunker_planar_move": "libbunker_planar_move_plugin.so",
            "bunker_laser": "libgazebo_ros_laser.so"}:
        raise RenderContractError("runtime plugin library set differs")
    if root.findall(".//publishTF") or root.findall(".//tf_prefix"):
        raise RenderContractError("unsupported TF plugin tag")
    return root


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


def render_runtime_urdf(source_path):
    source, _ = _validated_source(source_path)
    try:
        root = ET.fromstring(run_xacro(source))
    except ET.ParseError as error:
        raise RenderContractError(
            "expanded URDF is invalid XML: %s" % error)
    validate_source_tree(root, source)
    remap_urdf_names(root)
    make_base_model_joints_fixed(root)
    replace_collisions(root)
    add_base_friction(root)
    add_lidar(root)
    add_planar_plugin(root)
    validate_runtime_tree(root)
    _indent_tree(root)
    payload = ET.tostring(
        root, encoding="utf-8", xml_declaration=True,
        short_empty_elements=True).decode("utf-8")
    return payload.rstrip("\n") + "\n"
