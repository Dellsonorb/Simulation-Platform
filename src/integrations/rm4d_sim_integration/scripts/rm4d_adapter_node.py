#!/usr/bin/env python3
"""ROS-only boundary for the frozen external RM4D BasePlacementAPI."""

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from rm4d_sim_integration.core import IntegrationCore, load_external_api
from rm4d_sim_integration.geometry import (
    IntegrationError,
    PoseValues,
    quaternion_from_yaw,
    yaw_from_quaternion,
)
from rm4d_sim_integration.markers import marker_specs
from rm4d_sim_integration.srv import (
    PlanBasePlacement,
    PlanBasePlacementResponse,
)


class RM4DAdapterNode:
    def __init__(self):
        self._map_frame = rospy.get_param("~map_frame", "map")
        self._ground_base_frame = rospy.get_param(
            "~ground_base_frame", "ground/base_link")
        self._tf_timeout = rospy.Duration(
            float(rospy.get_param("~tf_timeout_sec", 1.0)))

        root = rospy.get_param("~rm4d_root")
        map_path = rospy.get_param("~rm4d_map")
        config_path = rospy.get_param("~rm4d_config", "") or None
        self._api = load_external_api(root, map_path, config_path)
        self._core = IntegrationCore(self._api)
        rospy.on_shutdown(self._api.close)

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._grasp_publisher = rospy.Publisher("/rm4d/grasp_tcp", PoseStamped, queue_size=1, latch=True)
        self._candidate_publisher = rospy.Publisher("/rm4d/candidates", PoseArray, queue_size=1, latch=True)
        self._marker_publisher = rospy.Publisher("/rm4d/candidate_markers", MarkerArray, queue_size=1, latch=True)
        self._service = rospy.Service(
            "/rm4d/plan_base_placement", PlanBasePlacement, self._plan)

    def _response(self, success, status, message):
        response = PlanBasePlacementResponse()
        response.success = success
        response.status = status
        response.candidates.header.frame_id = self._map_frame
        response.candidates.header.stamp = rospy.Time.now()
        response.message = message
        return response

    def _current_bunker_pose(self):
        transform = self._tf_buffer.lookup_transform(
            self._map_frame,
            self._ground_base_frame,
            rospy.Time(0),
            self._tf_timeout,
        ).transform
        rotation = transform.rotation
        return (
            transform.translation.x,
            transform.translation.y,
            yaw_from_quaternion(
                (rotation.x, rotation.y, rotation.z, rotation.w)),
        )

    @staticmethod
    def _pose_message(candidate):
        pose = Pose()
        pose.position.x = candidate.x
        pose.position.y = candidate.y
        pose.orientation = Quaternion(*quaternion_from_yaw(candidate.yaw))
        return pose

    def _marker_message(self, spec, stamp):
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = stamp
        marker.ns = "rm4d_candidates"
        marker.id = spec.marker_id
        marker.action = Marker.ADD
        marker.color = ColorRGBA(*spec.color)
        marker.pose.orientation.w = 1.0
        x, y, yaw = spec.pose

        if spec.kind == "arrow":
            marker.type = Marker.ARROW
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.06
            marker.pose.orientation = Quaternion(*quaternion_from_yaw(yaw))
            marker.scale.x = 0.45
            marker.scale.y = 0.08
            marker.scale.z = 0.12
        elif spec.kind == "footprint":
            marker.type = Marker.LINE_STRIP
            marker.points = [Point(*point) for point in spec.points]
            marker.scale.x = 0.025
        elif spec.kind == "text":
            marker.type = Marker.TEXT_VIEW_FACING
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.35
            marker.scale.z = 0.14
            marker.text = spec.text
        return marker

    def _publish(self, grasp_tcp, candidates):
        stamp = rospy.Time.now()
        exact = PoseStamped()
        exact.header.frame_id = self._map_frame
        exact.header.stamp = stamp
        exact.pose = grasp_tcp.pose
        self._grasp_publisher.publish(exact)

        candidate_array = PoseArray()
        candidate_array.header.frame_id = self._map_frame
        candidate_array.header.stamp = stamp
        candidate_array.poses = [
            self._pose_message(candidate) for candidate in candidates]
        self._candidate_publisher.publish(candidate_array)

        delete_all = Marker()
        delete_all.header.frame_id = self._map_frame
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        markers = [delete_all]
        markers.extend(
            self._marker_message(spec, stamp)
            for spec in marker_specs(candidates))
        self._marker_publisher.publish(MarkerArray(markers=markers))
        return candidate_array

    def _plan(self, request):
        try:
            if request.grasp_tcp.header.frame_id != self._map_frame:
                raise IntegrationError("grasp_tcp frame must be map")
            source = request.grasp_tcp.pose
            exact_pose = PoseValues(
                position=(
                    source.position.x,
                    source.position.y,
                    source.position.z,
                ),
                orientation=(
                    source.orientation.x,
                    source.orientation.y,
                    source.orientation.z,
                    source.orientation.w,
                ),
            )
            result = self._core.plan(
                frame_id=self._map_frame,
                exact_pose=exact_pose,
                current_bunker_pose=self._current_bunker_pose(),
                grasp_id=request.grasp_id,
                top_k=int(request.top_k),
            )
            response = self._response(
                True, result.status,
                "RM4D returned %d candidate(s)" % len(result.candidates))
            response.candidates = self._publish(
                request.grasp_tcp, result.candidates)
            response.candidate_ids = [
                candidate.candidate_id for candidate in result.candidates]
            response.scores = [
                candidate.score for candidate in result.candidates]
            return response
        except (IntegrationError, tf2_ros.TransformException) as error:
            rospy.logwarn("RM4D request failed: %s", error)
            return self._response(False, "error", str(error))
        except Exception as error:
            rospy.logerr("RM4D external API failure: %s", error)
            return self._response(False, "error", str(error))


def main():
    rospy.init_node("rm4d_adapter")
    try:
        RM4DAdapterNode()
    except Exception as error:
        rospy.logfatal("cannot start RM4D adapter: %s", error)
        return
    rospy.spin()


if __name__ == "__main__":
    main()
