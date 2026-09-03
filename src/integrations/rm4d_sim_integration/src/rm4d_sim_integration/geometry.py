"""Pure map-frame conversions for the frozen RM4D BasePlacementAPI."""

from dataclasses import dataclass
import math


RM4D_LOCAL_Y_REGULARIZATION_RAD = 1e-6


class IntegrationError(RuntimeError):
    """Raised when the ROS-facing value cannot satisfy the frozen contract."""


@dataclass(frozen=True)
class PoseValues:
    position: tuple
    orientation: tuple


@dataclass(frozen=True)
class CandidateValues:
    candidate_id: str
    x: float
    y: float
    yaw: float
    score: float

    @property
    def pose(self):
        return self.x, self.y, self.yaw


def _finite_tuple(values, length, label):
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as error:
        raise IntegrationError("%s must contain finite values" % label) from error
    if len(result) != length or not all(math.isfinite(value)
                                        for value in result):
        raise IntegrationError("%s must contain finite values" % label)
    return result


def _normalized_quaternion(values):
    quaternion = _finite_tuple(values, 4, "quaternion")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1e-12:
        raise IntegrationError("quaternion has zero norm")
    return tuple(value / norm for value in quaternion)


def _multiply_quaternions(first, second):
    x1, y1, z1, w1 = _normalized_quaternion(first)
    x2, y2, z2, w2 = _normalized_quaternion(second)
    return _normalized_quaternion((
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ))


def quaternion_angle(first, second):
    first_q = _normalized_quaternion(first)
    second_q = _normalized_quaternion(second)
    dot = abs(sum(a * b for a, b in zip(first_q, second_q)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def quaternion_from_yaw(yaw):
    yaw_value, = _finite_tuple((yaw,), 1, "yaw")
    half_yaw = 0.5 * yaw_value
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def yaw_from_quaternion(quaternion):
    x, y, z, w = _normalized_quaternion(quaternion)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def regularize_for_rm4d(exact_pose):
    """Return the fixed query-only local-Y regularization of an exact TCP."""
    position = _finite_tuple(exact_pose.position, 3, "position")
    orientation = _normalized_quaternion(exact_pose.orientation)
    half_angle = 0.5 * RM4D_LOCAL_Y_REGULARIZATION_RAD
    local_y = (0.0, math.sin(half_angle), 0.0, math.cos(half_angle))
    return PoseValues(position, _multiply_quaternions(orientation, local_y))


def build_rm4d_request(
        frame_id, exact_pose, current_bunker_pose, grasp_id):
    if frame_id != "map":
        raise IntegrationError("RM4D integration frame must be map")
    if not isinstance(grasp_id, str) or not grasp_id.strip():
        raise IntegrationError("grasp_id must be a non-empty string")
    current = _finite_tuple(current_bunker_pose, 3, "current_bunker_pose")
    query_pose = regularize_for_rm4d(exact_pose)
    return {
        "frame_id": "map",
        "grasp_id": grasp_id,
        "position_xyz": list(query_pose.position),
        "quaternion_xyzw": list(query_pose.orientation),
        "current_bunker_pose": {
            "x": current[0], "y": current[1], "yaw": current[2]},
    }


def ordered_candidates(result):
    try:
        source = result["candidates"]
    except (KeyError, TypeError) as error:
        raise IntegrationError("RM4D result has no candidates") from error
    if not isinstance(source, list):
        raise IntegrationError("RM4D candidates must be a list")
    converted = []
    for item in source:
        try:
            candidate_id = item["candidate_id"]
            x, y, yaw, score = _finite_tuple(
                (item["bunker_x"], item["bunker_y"],
                 item["bunker_yaw"], item["final_score"]),
                4, "candidate")
        except (KeyError, TypeError) as error:
            raise IntegrationError("RM4D candidate is incomplete") from error
        if not isinstance(candidate_id, str) or not candidate_id:
            raise IntegrationError("RM4D candidate_id is invalid")
        converted.append(CandidateValues(
            candidate_id=candidate_id, x=x, y=y, yaw=yaw, score=score))
    return tuple(converted)
