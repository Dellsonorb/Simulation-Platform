import decimal
import math
import os
from pathlib import Path
import re


MODEL_NAME = "bunker"
ROS_NAMESPACE = "/ground"
TF_PREFIX = "ground"
DEFAULT_POSE = ("0.0", "0.0", "0.36", "0.0", "0.0", "0.0")
LOCAL_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,63}$")
ENTITY_NAME_RE = re.compile(r"^[a-z][a-z0-9_]{0,63}$")
DECIMAL_RE = re.compile(
    r"^[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?$")
SPAWN_EXECUTABLE = "/opt/ros/noetic/lib/gazebo_ros/spawn_model"
SPAWN_LOG_RE = re.compile(r"^ground-spawn_bunker-[1-9][0-9]*\.log$")


class RuntimeContractError(ValueError):
    pass


def validate_local_name(value):
    if type(value) is not str or LOCAL_NAME_RE.fullmatch(value) is None:
        raise RuntimeContractError("invalid local name: %r" % (value,))
    return value


def validate_entity_name(value):
    if type(value) is not str or ENTITY_NAME_RE.fullmatch(value) is None:
        raise RuntimeContractError("invalid entity name: %r" % (value,))
    return value


def parse_pose(values):
    if (not isinstance(values, (list, tuple)) or
            isinstance(values, (str, bytes)) or len(values) != 6):
        raise RuntimeContractError("pose requires six decimal scalars")
    parsed = []
    for value in values:
        if type(value) is not str or DECIMAL_RE.fullmatch(value) is None:
            raise RuntimeContractError("invalid pose scalar: %r" % (value,))
        number = decimal.Decimal(value)
        result = float(number)
        if not number.is_finite() or not math.isfinite(result):
            raise RuntimeContractError("nonfinite pose scalar: %r" % value)
        parsed.append(result)
    return tuple(parsed)


def _canonical_mode_0700_directory(value):
    try:
        path = Path(value)
    except TypeError:
        raise RuntimeContractError("ROS_LOG_DIR is not canonical")
    if not path.is_absolute() or path.is_symlink():
        raise RuntimeContractError("ROS_LOG_DIR is not canonical")
    canonical = path.resolve(strict=True)
    if canonical != path or not canonical.is_dir():
        raise RuntimeContractError("ROS_LOG_DIR is not canonical")
    if (canonical.stat().st_mode & 0o777) != 0o700:
        raise RuntimeContractError("ROS_LOG_DIR is not mode 0700")
    return canonical


def validate_spawn_command(command, pose_tokens, ros_log_dir):
    parse_pose(pose_tokens)
    if (not isinstance(command, (list, tuple)) or
            isinstance(command, (str, bytes)) or len(command) != 21):
        raise RuntimeContractError("spawn command requires 21 tokens")
    if any(type(value) is not str for value in command):
        raise RuntimeContractError("spawn command tokens must be strings")
    executable = os.path.realpath(command[0])
    if (command[0] != SPAWN_EXECUTABLE or executable != SPAWN_EXECUTABLE or
            not os.path.isfile(executable) or
            not os.access(executable, os.X_OK)):
        raise RuntimeContractError("invalid spawn executable")
    expected = (
        SPAWN_EXECUTABLE,
        "-urdf", "-param", "robot_description", "-model", MODEL_NAME, "-b",
        "-x", pose_tokens[0], "-y", pose_tokens[1], "-z", pose_tokens[2],
        "-R", pose_tokens[3], "-P", pose_tokens[4], "-Y", pose_tokens[5],
    )
    if tuple(command[:19]) != expected:
        raise RuntimeContractError("spawn command contract changed")
    if command[19] != "__name:=spawn_bunker":
        raise RuntimeContractError("spawn node remap changed")
    if not command[20].startswith("__log:="):
        raise RuntimeContractError("spawn log remap is missing")
    log_root = _canonical_mode_0700_directory(ros_log_dir)
    log_path = Path(command[20][len("__log:="):])
    if not log_path.is_absolute():
        raise RuntimeContractError("spawn log path is invalid")
    if SPAWN_LOG_RE.fullmatch(log_path.name) is None:
        raise RuntimeContractError("spawn log basename changed")
    parent = log_path.parent.resolve(strict=True)
    if parent != log_path.parent:
        raise RuntimeContractError("spawn log path is noncanonical")
    try:
        parent.relative_to(log_root)
    except ValueError:
        raise RuntimeContractError("spawn log escapes ROS_LOG_DIR")
    return tuple(command)
