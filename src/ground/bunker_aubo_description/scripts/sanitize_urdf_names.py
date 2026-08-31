#!/usr/bin/env python
from __future__ import print_function

import re
import subprocess
import sys
import xml.etree.ElementTree as ET


NAME_ATTRIBUTES = {'name', 'link', 'reference', 'joint'}
NAME_TEXT_TAGS = {'parent', 'child', 'joint', 'mimicJoint'}
LOCK_AG95_OPEN_ARGUMENT = 'lock_ag95_open:='
AG95_EFFORT_COUPLED_ARGUMENT = 'ag95_effort_coupled:='
MIMIC_PLUGIN = 'libroboticsgroup_upatras_gazebo_mimic_joint_plugin.so'


def sanitize(value):
    return re.sub(r'[^A-Za-z0-9_/]', '_', value)


def remove_ag95_loop_and_mimic(root):
    """Remove the redundant SDF constraints from the upstream AG95 model."""
    gazebo_elements = root.findall('gazebo')
    loop_names = {
        'left_inner_knuckle_to_finger_joint',
        'right_inner_knuckle_to_finger_joint'}
    for gazebo in gazebo_elements:
        for joint in list(gazebo.findall('joint')):
            if joint.attrib.get('name') in loop_names:
                gazebo.remove(joint)
        for plugin in list(gazebo.findall('plugin')):
            if plugin.get('filename') == MIMIC_PLUGIN:
                gazebo.remove(plugin)


def lock_ag95_open(root):
    """Build the directed reality-check model with AG95 held physically open.

    The 115 mm Brick is rejected before any gripper actuator command because
    it exceeds the measured 95.2 mm jaw opening.  For that fail-closed audit,
    hold the five passive linkage joints at their real q=0 open geometry and
    remove redundant Gazebo loop/mimic constraints.  The actuated joint and
    its unmodified 0..0.93 rad hardware limit remain present; this opt-in mode
    must not be used to claim a successful grasp.
    """
    remove_ag95_loop_and_mimic(root)
    passive_joints = {
        'right_outer_knuckle_joint', 'left_finger_joint',
        'right_finger_joint', 'left_inner_knuckle_joint',
        'right_inner_knuckle_joint'}
    for joint in root.findall('joint'):
        if joint.attrib.get('name') not in passive_joints:
            continue
        joint.attrib['type'] = 'fixed'
        for tag in ('axis', 'limit', 'dynamics', 'calibration',
                    'safety_controller', 'mimic'):
            for child in list(joint.findall(tag)):
                joint.remove(child)


def configure_ag95_reduced_effort_linkage(root):
    """Use a stable, effort-driven two-jaw compatibility linkage.

    The upstream URDF+SDF combination over-constrains four auxiliary links
    with mimic joints and two loop joints.  Once the loop constraints are
    removed, those lightweight auxiliary joints diverge under ODE.  Keep the
    actual controller-driven left outer knuckle and the effort-coupled right
    outer knuckle dynamic; lock only the four unactuated parallelogram helper
    joints at their real q=0 geometry.  Both original collision pad chains,
    the master 0..0.93 limit, and the 95.2 mm opening remain unchanged.
    """
    remove_ag95_loop_and_mimic(root)
    fixed_helpers = {
        'left_finger_joint', 'right_finger_joint',
        'left_inner_knuckle_joint', 'right_inner_knuckle_joint'}
    for joint in root.findall('joint'):
        if joint.attrib.get('name') not in fixed_helpers:
            continue
        joint.attrib['type'] = 'fixed'
        for tag in ('axis', 'limit', 'dynamics', 'calibration',
                    'safety_controller', 'mimic'):
            for child in list(joint.findall(tag)):
                joint.remove(child)
    # Gazebo otherwise lumps the fixed helpers and their pad children into
    # outer-knuckle links, which removes the named pad collision/contact
    # entities required by the guarded attachment audit.
    preserve_joints = fixed_helpers | {
        'left_inner_finger_pad_joint', 'right_inner_finger_pad_joint'}
    for name in sorted(preserve_joints):
        gazebo = ET.SubElement(root, 'gazebo', {'reference': name})
        ET.SubElement(gazebo, 'preserveFixedJoint').text = 'true'


def main():
    if len(sys.argv) >= 2:
        xacro_arguments = []
        lock_open = False
        effort_coupled = False
        for argument in sys.argv[2:]:
            if argument.startswith(LOCK_AG95_OPEN_ARGUMENT):
                value = argument[len(LOCK_AG95_OPEN_ARGUMENT):].lower()
                if value not in ('true', 'false'):
                    raise SystemExit('lock_ag95_open must be true/false')
                lock_open = value == 'true'
            elif argument.startswith(AG95_EFFORT_COUPLED_ARGUMENT):
                value = argument[len(AG95_EFFORT_COUPLED_ARGUMENT):].lower()
                if value not in ('true', 'false'):
                    raise SystemExit('ag95_effort_coupled must be true/false')
                effort_coupled = value == 'true'
                xacro_arguments.append(argument)
            else:
                xacro_arguments.append(argument)
        if lock_open and effort_coupled:
            raise SystemExit(
                'lock_ag95_open and ag95_effort_coupled are mutually exclusive')
        source = subprocess.check_output(
            ['xacro', '--inorder', sys.argv[1]] + xacro_arguments)
    elif len(sys.argv) == 1:
        source = sys.stdin.read()
        lock_open = False
        effort_coupled = False
    else:
        raise SystemExit(
            'usage: sanitize_urdf_names.py [robot.xacro [xacro_arg:=value ...]]')
    root = ET.fromstring(source)
    for element in root.iter():
        for attribute in NAME_ATTRIBUTES:
            if attribute in element.attrib:
                element.attrib[attribute] = sanitize(element.attrib[attribute])
        if element.tag in NAME_TEXT_TAGS and element.text:
            element.text = sanitize(element.text.strip())
    # The BUNKER is intentionally stationary in V0.1. Its exported wheel
    # joints have zero effort/velocity yet are declared revolute, leaving
    # MoveIt waiting forever for states that no controller publishes.
    for joint in root.findall('joint'):
        if joint.attrib.get('name', '').startswith('wheel'):
            joint.attrib['type'] = 'fixed'
            for tag in ('axis', 'limit', 'dynamics', 'calibration',
                        'safety_controller'):
                for child in joint.findall(tag):
                    joint.remove(child)
    if lock_open:
        lock_ag95_open(root)
    elif effort_coupled:
        configure_ag95_reduced_effort_linkage(root)
    print(ET.tostring(root, encoding='utf-8'))


if __name__ == '__main__':
    main()
