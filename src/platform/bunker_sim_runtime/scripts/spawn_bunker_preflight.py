#!/usr/bin/env python3
import os
import sys

from bunker_sim_runtime.contracts import (
    RuntimeContractError,
    SPAWN_EXECUTABLE,
    parse_pose,
    validate_spawn_command,
)
from bunker_sim_runtime.sim_time import (
    SimTimeContractError,
    require_boolean_sim_time,
)


def _fail(code, message, stderr):
    stderr.write("bunker-spawn-preflight: %s\n" % message)
    return code


def main(argv=None, environ=None, stderr=None,
         sim_time_reader=require_boolean_sim_time, execv=os.execv,
         isfile=os.path.isfile, access=os.access):
    arguments = list(sys.argv[1:] if argv is None else argv)
    environment = os.environ if environ is None else environ
    errors = sys.stderr if stderr is None else stderr
    if len(arguments) < 8 or arguments[6] != "--":
        return _fail(
            64, "expected six pose scalars, --, and spawn command", errors)
    pose_tokens = arguments[:6]
    command = arguments[7:]
    try:
        parse_pose(pose_tokens)
    except RuntimeContractError as error:
        return _fail(64, str(error), errors)
    if (command and command[0] == SPAWN_EXECUTABLE and
            (not isfile(SPAWN_EXECUTABLE) or
             not access(SPAWN_EXECUTABLE, os.X_OK))):
        return _fail(
            66, "frozen spawn executable is unavailable", errors)
    try:
        ros_log_dir = environment["ROS_LOG_DIR"]
        master_uri = environment["ROS_MASTER_URI"]
    except KeyError as error:
        return _fail(
            65, "missing environment variable %s" % error.args[0], errors)
    try:
        validated = validate_spawn_command(
            command, pose_tokens, ros_log_dir)
    except RuntimeContractError as error:
        return _fail(64, str(error), errors)
    try:
        sim_time_reader(master_uri, "/ground/spawn_bunker")
    except SimTimeContractError as error:
        return _fail(65, str(error), errors)
    try:
        execv(validated[0], validated)
    except OSError as error:
        return _fail(
            66, "could not exec spawn_model: %s" % error, errors)
    return _fail(70, "execv returned unexpectedly", errors)


if __name__ == "__main__":
    raise SystemExit(main())
