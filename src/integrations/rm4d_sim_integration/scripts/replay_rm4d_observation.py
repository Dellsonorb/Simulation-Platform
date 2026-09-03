#!/usr/bin/env python3
"""Replay one sensor-derived P450 observation through RM4D."""

import argparse
import json
import os
from pathlib import Path

from air_ground_pick_demo.grasp import generate_top_down_grasp
from rm4d_sim_integration.core import IntegrationCore, load_external_api
from rm4d_sim_integration.geometry import (
    CandidateValues,
    IntegrationError,
    PoseValues,
    yaw_from_quaternion,
)


def _arguments():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Replay a P450 RGB-D target pose through frozen RM4D")
    parser.add_argument("--mode", choices=("offline", "online"), required=True)
    parser.add_argument(
        "--replay", default=str(package / "config/p450_rgbd_replay.yaml"))
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--rm4d-root", default=os.environ.get("RM4D_ROOT"))
    parser.add_argument("--rm4d-map", default=os.environ.get("RM4D_MAP"))
    parser.add_argument("--rm4d-config", default=os.environ.get("RM4D_CONFIG"))
    parser.add_argument(
        "--service", default="/rm4d/plan_base_placement")
    return parser.parse_args()


def _load_replay(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        replay = json.load(stream)
    if replay.get("source_topic") != "/air_observer/target_pose":
        raise IntegrationError("replay source must be the P450 observer")
    if replay.get("frame_id") != "map":
        raise IntegrationError("replay frame must be map")
    return replay


def _exact_grasp(replay):
    poses = generate_top_down_grasp(
        replay["target"],
        replay["target_size"],
        replay["pregrasp_height"],
        replay["lift_height"],
        replay["finger_pad_lower_edge_offset"],
        replay["contact_overlap"],
        replay["surface_clearance"],
    )
    return PoseValues(poses.grasp.position, poses.grasp.orientation)


def _offline(arguments, replay, exact_pose):
    if not arguments.rm4d_root or not arguments.rm4d_map:
        raise IntegrationError(
            "offline mode requires --rm4d-root and --rm4d-map")
    api = load_external_api(
        arguments.rm4d_root, arguments.rm4d_map, arguments.rm4d_config)
    try:
        return IntegrationCore(api).plan(
            replay["frame_id"],
            exact_pose,
            tuple(replay["current_bunker_pose"]),
            replay["grasp_id"],
            arguments.top_k,
        )
    finally:
        api.close()


def _online(arguments, replay, exact_pose):
    import rospy
    from geometry_msgs.msg import PoseStamped, Quaternion
    from rm4d_sim_integration.srv import (
        PlanBasePlacement,
        PlanBasePlacementRequest,
    )

    rospy.init_node("replay_rm4d_observation", anonymous=True)
    rospy.wait_for_service(arguments.service)
    request = PlanBasePlacementRequest()
    request.grasp_tcp = PoseStamped()
    request.grasp_tcp.header.frame_id = replay["frame_id"]
    request.grasp_tcp.header.stamp = rospy.Time.now()
    request.grasp_tcp.pose.position.x = exact_pose.position[0]
    request.grasp_tcp.pose.position.y = exact_pose.position[1]
    request.grasp_tcp.pose.position.z = exact_pose.position[2]
    request.grasp_tcp.pose.orientation = Quaternion(*exact_pose.orientation)
    request.grasp_id = replay["grasp_id"]
    request.top_k = arguments.top_k
    response = rospy.ServiceProxy(arguments.service, PlanBasePlacement)(request)
    if not (
            len(response.candidates.poses) == len(response.candidate_ids) ==
            len(response.scores)):
        raise IntegrationError("online response candidate arrays differ")
    candidates = tuple(
        CandidateValues(
            candidate_id,
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion((
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )),
            score,
        )
        for pose, candidate_id, score in zip(
            response.candidates.poses,
            response.candidate_ids,
            response.scores,
        )
    )
    return type("OnlineResult", (), {
        "status": response.status,
        "candidates": candidates,
    })()


def _summary(result, replay, exact_pose):
    return {
        "source_topic": replay["source_topic"],
        "frame_id": replay["frame_id"],
        "exact_grasp_tcp": {
            "position_xyz": list(exact_pose.position),
            "quaternion_xyzw": list(exact_pose.orientation),
        },
        "status": result.status,
        "candidates": [
            {
                "rank": rank,
                "candidate_id": candidate.candidate_id,
                "bunker_pose": [candidate.x, candidate.y, candidate.yaw],
                "score": candidate.score,
            }
            for rank, candidate in enumerate(result.candidates, start=1)
        ],
    }


def main():
    arguments = _arguments()
    if arguments.top_k <= 0:
        raise SystemExit("--top-k must be positive")
    replay = _load_replay(arguments.replay)
    exact_pose = _exact_grasp(replay)
    result = _offline(arguments, replay, exact_pose) \
        if arguments.mode == "offline" else \
        _online(arguments, replay, exact_pose)
    print(json.dumps(_summary(result, replay, exact_pose), indent=2))


if __name__ == "__main__":
    main()
