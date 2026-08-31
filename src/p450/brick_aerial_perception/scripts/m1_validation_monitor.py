#!/usr/bin/env python3
"""Pair four fixed P450 views with four brick poses and score world estimates."""

import json
import hashlib
import math
import os
import subprocess
import threading

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from brick_aerial_perception.geometry import yaw_error_mod_pi
from brick_aerial_perception.validation import summarize_results


def _source_tree_sha256(package_root):
    digest = hashlib.sha256()
    entries = ("CMakeLists.txt", "package.xml", "config", "launch", "rviz",
               "scripts", "src", "test", "worlds")
    paths = []
    for entry in entries:
        path = os.path.join(package_root, entry)
        if os.path.isfile(path):
            paths.append(path)
        elif os.path.isdir(path):
            for root, directories, filenames in os.walk(path):
                directories[:] = [item for item in directories if item != "__pycache__"]
                for filename in filenames:
                    if not filename.endswith((".pyc", ".pyo")):
                        paths.append(os.path.join(root, filename))
    for path in sorted(paths):
        relative = os.path.relpath(path, package_root).replace(os.sep, "/")
        digest.update(relative.encode("utf-8") + b"\0")
        with open(path, "rb") as stream:
            digest.update(stream.read())
    return digest.hexdigest()


def _git_value(package_root, arguments):
    try:
        return subprocess.check_output(
            ["git", "-C", package_root] + list(arguments),
            stderr=subprocess.DEVNULL, universal_newlines=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


class M1ValidationMonitor:
    def __init__(self):
        self.uav_id = int(rospy.get_param("~uav_id", 1))
        self.scenarios = rospy.get_param("~validation_scenarios")
        self.scenario_by_view = {item["viewpoint"]: item for item in self.scenarios}
        if len(self.scenario_by_view) != len(self.scenarios):
            raise ValueError("validation viewpoints must be unique")
        self.output_json = rospy.get_param("~output_json", "/tmp/m1_validation.json")
        self.package_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        self.max_position_error = float(rospy.get_param("~max_position_error", 0.12))
        self.max_yaw_error = float(rospy.get_param("~max_yaw_error", 0.20))
        self.lock = threading.RLock()
        self.active_view = None
        self.sample_started = rospy.Time(0)
        self.samples = []
        self.results = []
        self.positioned_views = set()
        self.latest_quality = {}
        self.latest_uav_pose = None
        self.rgb_observation = None
        self.depth_observation = None
        self.color_info = None
        self.depth_info = None
        self.written = False

        prefix = "/uav{}/camera/".format(self.uav_id)
        rospy.Subscriber("/m1/viewpoint_state", String, self._state_cb, queue_size=10)
        rospy.Subscriber("/m1_aerial_brick_pose/brick_pose", PoseStamped,
                         self._pose_cb, queue_size=20)
        rospy.Subscriber("/m1_aerial_brick_pose/quality_status", String,
                         self._quality_cb, queue_size=20)
        rospy.Subscriber("/uav{}/prometheus/ground_truth".format(self.uav_id),
                         Odometry, self._uav_cb, queue_size=20)
        rospy.Subscriber(prefix + "color/image_raw", Image, self._rgb_cb, queue_size=1)
        rospy.Subscriber(prefix + "depth/image_raw", Image, self._depth_cb, queue_size=1)
        rospy.Subscriber(prefix + "color/camera_info", CameraInfo,
                         self._color_info_cb, queue_size=1)
        rospy.Subscriber(prefix + "depth/camera_info", CameraInfo,
                         self._depth_info_cb, queue_size=1)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)
        self.set_model_state = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState, persistent=True
        )
        rospy.on_shutdown(self._write_partial)

    @staticmethod
    def _yaw(quaternion):
        return euler_from_quaternion((quaternion.x, quaternion.y,
                                      quaternion.z, quaternion.w))[2]

    def _set_brick(self, scenario):
        state = ModelState()
        state.model_name = "m1_brick"
        state.reference_frame = "world"
        state.pose.position.x = float(scenario["brick_position"][0])
        state.pose.position.y = float(scenario["brick_position"][1])
        state.pose.position.z = float(scenario["brick_position"][2])
        quaternion = quaternion_from_euler(0.0, 0.0, float(scenario["brick_yaw"]))
        state.pose.orientation.x = quaternion[0]
        state.pose.orientation.y = quaternion[1]
        state.pose.orientation.z = quaternion[2]
        state.pose.orientation.w = quaternion[3]
        response = self.set_model_state(state)
        if not response.success:
            raise RuntimeError("SetModelState failed: " + response.status_message)
        rospy.loginfo("[m1_validation] %s brick=(%.3f, %.3f, %.3f, %.3f)",
                      scenario["id"], state.pose.position.x, state.pose.position.y,
                      state.pose.position.z, float(scenario["brick_yaw"]))

    def _state_cb(self, message):
        state = message.data
        moving_view = state.split(":", 1)[1] if state.startswith("MOVING:") else None
        sampling_view = state.split(":", 1)[1] if state.startswith("SAMPLING:") else None
        if moving_view in self.scenario_by_view and moving_view not in self.positioned_views:
            try:
                self._set_brick(self.scenario_by_view[moving_view])
                self.positioned_views.add(moving_view)
            except (rospy.ServiceException, RuntimeError) as error:
                rospy.logerr("[m1_validation] brick positioning failed: %s", error)
        with self.lock:
            if self.active_view is not None and sampling_view != self.active_view:
                self._finalize_active()
            if sampling_view is not None and sampling_view != self.active_view:
                self.active_view = sampling_view
                self.sample_started = rospy.Time.now()
                self.samples = []
            if state == "COMPLETE" or state.startswith("FAILED:"):
                self._write_report(state)

    def _pose_cb(self, message):
        with self.lock:
            if (self.active_view is None or message.header.frame_id.lstrip("/") != "world"
                    or message.header.stamp < self.sample_started):
                return
            pose = message.pose
            sample = {
                "stamp": message.header.stamp.to_sec(),
                "position": [pose.position.x, pose.position.y, pose.position.z],
                "yaw": self._yaw(pose.orientation),
                "quality": dict(self.latest_quality),
                "uav_world_pose": self.latest_uav_pose,
            }
            self.samples.append(sample)

    def _quality_cb(self, message):
        try:
            quality = json.loads(message.data)
        except ValueError:
            return
        with self.lock:
            self.latest_quality = quality

    def _uav_cb(self, message):
        pose = message.pose.pose
        with self.lock:
            self.latest_uav_pose = {
                "position": [pose.position.x, pose.position.y, pose.position.z],
                "yaw": self._yaw(pose.orientation),
            }

    def _rgb_cb(self, message):
        self.rgb_observation = {
            "topic": "/uav{}/camera/color/image_raw".format(self.uav_id),
            "encoding": message.encoding, "width": message.width,
            "height": message.height, "frame_id": message.header.frame_id,
        }

    def _depth_cb(self, message):
        self.depth_observation = {
            "topic": "/uav{}/camera/depth/image_raw".format(self.uav_id),
            "encoding": message.encoding, "width": message.width,
            "height": message.height, "frame_id": message.header.frame_id,
        }

    @staticmethod
    def _info(message, topic):
        return {"topic": topic, "frame_id": message.header.frame_id,
                "width": message.width, "height": message.height,
                "K": list(message.K)}

    def _color_info_cb(self, message):
        self.color_info = self._info(
            message, "/uav{}/camera/color/camera_info".format(self.uav_id)
        )

    def _depth_info_cb(self, message):
        self.depth_info = self._info(
            message, "/uav{}/camera/depth/camera_info".format(self.uav_id)
        )

    def _finalize_active(self):
        view = self.active_view
        scenario = self.scenario_by_view.get(view)
        if scenario is None:
            self.active_view = None
            self.samples = []
            return
        if not self.samples:
            self.results.append({"scenario_id": scenario["id"], "viewpoint": view,
                                 "success": False, "reason": "no_valid_pose"})
        else:
            positions = np.asarray([item["position"] for item in self.samples],
                                   dtype=np.float64)
            position = np.median(positions, axis=0)
            doubled = 2.0 * np.asarray([item["yaw"] for item in self.samples])
            yaw = 0.5 * math.atan2(float(np.mean(np.sin(doubled))),
                                   float(np.mean(np.cos(doubled))))
            gt_position = np.asarray(scenario["brick_position"], dtype=np.float64)
            delta = position - gt_position
            position_error = float(np.linalg.norm(delta))
            xy_error = float(np.linalg.norm(delta[:2]))
            z_error = abs(float(delta[2]))
            yaw_error = yaw_error_mod_pi(yaw, float(scenario["brick_yaw"]))
            success = bool(position_error <= self.max_position_error
                           and yaw_error <= self.max_yaw_error)
            self.results.append({
                "scenario_id": scenario["id"], "viewpoint": view,
                "success": success,
                "reason": "within_limits" if success else "pose_error_limit",
                "sample_count": len(self.samples),
                "estimated_pose": {"position": position.tolist(), "yaw": yaw},
                "ground_truth_pose": {"position": list(scenario["brick_position"]),
                                      "yaw": float(scenario["brick_yaw"])},
                "position_error": position_error, "xy_error": xy_error,
                "z_error": z_error, "yaw_error": yaw_error,
                "last_quality": self.samples[-1]["quality"],
                "uav_world_pose": self.samples[-1]["uav_world_pose"],
            })
        self.active_view = None
        self.samples = []

    def _report_payload(self, terminal_state):
        reported = {item["viewpoint"] for item in self.results}
        for scenario in self.scenarios:
            if scenario["viewpoint"] not in reported:
                self.results.append({
                    "scenario_id": scenario["id"], "viewpoint": scenario["viewpoint"],
                    "success": False, "reason": "viewpoint_not_sampled",
                })
        return {
            "terminal_state": terminal_state,
            "world_frame": "world",
            "pose_topic": "/m1_aerial_brick_pose/brick_pose",
            "optical_frame": "uav{}/camera_color_optical_frame".format(self.uav_id),
            "rgb": self.rgb_observation, "depth": self.depth_observation,
            "color_camera_info": self.color_info, "depth_camera_info": self.depth_info,
            "thresholds": {"max_position_error_m": self.max_position_error,
                           "max_yaw_error_rad": self.max_yaw_error},
            "provenance": {
                "source_tree_sha256": _source_tree_sha256(self.package_root),
                "git_head": _git_value(self.package_root, ["rev-parse", "HEAD"]),
                "git_branch": _git_value(
                    self.package_root, ["rev-parse", "--abbrev-ref", "HEAD"]
                ),
                "git_status": _git_value(
                    self.package_root, ["status", "--short", "--",
                                        self.package_root]
                ),
            },
            "results": self.results,
            "summary": summarize_results(self.results, len(self.scenarios)),
        }

    def _write_report(self, terminal_state):
        if self.written:
            return
        payload = self._report_payload(terminal_state)
        parent = os.path.dirname(os.path.abspath(self.output_json))
        if not os.path.isdir(parent):
            os.makedirs(parent)
        temporary = self.output_json + ".tmp"
        with open(temporary, "w") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, self.output_json)
        self.written = True
        rospy.loginfo("[m1_validation] wrote %s (%d/%d)", self.output_json,
                      payload["summary"]["success_count"], len(self.scenarios))

    def _write_partial(self):
        with self.lock:
            if not self.written and self.results:
                self._write_report("INTERRUPTED")


if __name__ == "__main__":
    rospy.init_node("m1_validation_monitor")
    M1ValidationMonitor()
    rospy.spin()
