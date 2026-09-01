import math


MODEL_NAME = "ground_robot"
HOME_JOINT_POSITIONS = (
    ("shoulder_pan_joint", 0.0),
    ("shoulder_lift_joint", -0.5),
    ("elbow_joint", 1.0),
    ("wrist_1_joint", 0.0),
    ("wrist_2_joint", 1.0),
    ("wrist_3_joint", 0.0),
    ("left_outer_knuckle_joint", 0.0),
)


class StartupError(RuntimeError):
    pass


def joint_feedback_is_complete(message, expected_positions):
    if len(message.name) != len(message.position):
        return False
    positions = dict(zip(message.name, message.position))
    expected_names = {name for name, _value in expected_positions}
    return expected_names.issubset(positions) and all(
        math.isfinite(positions[name]) for name in expected_names)


def arm_home_trajectory(positions):
    positions = tuple(positions)
    folded = tuple(
        (name, 0.0 if name == "shoulder_lift_joint" else value)
        for name, value in positions)
    return ((4.0, folded), (8.0, positions))


def initialize_ground_robot(
        gateway, robot_xml, initial_pose, wait_for_model,
        enable_controllers, drive_home, timeout=30.0):
    if not isinstance(robot_xml, str) or not robot_xml.strip():
        raise StartupError("ground robot description is empty")
    if timeout <= 0.0:
        raise StartupError("ground model timeout must be positive")

    spawn_attempt = gateway.begin_spawn(MODEL_NAME, robot_xml, initial_pose)
    if not wait_for_model(MODEL_NAME, timeout):
        raise StartupError("ground robot did not appear in Gazebo")
    enable_controllers()
    if not drive_home(HOME_JOINT_POSITIONS, timeout):
        raise StartupError("ground robot controller home motion failed")
    try:
        return bool(spawn_attempt.result(timeout))
    except Exception as error:
        raise StartupError(
            "Gazebo ground robot spawn request failed: %s" % error) from error
