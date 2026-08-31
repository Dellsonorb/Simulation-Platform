#!/usr/bin/env python3
"""M5-only source entrypoint for the current multi-view mission node."""

import importlib.util
from pathlib import Path

import rospy


def _load_current_node():
    source = Path(__file__).resolve().with_name("aerial_viewpoint_mission.py")
    spec = importlib.util.spec_from_file_location(
        "brick_aerial_perception_current_viewpoint_node", str(source)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.AerialViewpointMissionNode


if __name__ == "__main__":
    rospy.init_node("m1_aerial_viewpoint_mission")
    _load_current_node()()
    rospy.spin()
