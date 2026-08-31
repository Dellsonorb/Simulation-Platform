#!/usr/bin/env bash
set -eo pipefail

M1_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
M1_WS_ROOT="$(realpath "${M1_SCRIPT_DIR}/../../..")"
M1_PROMETHEUS_ROOT="${M1_PROMETHEUS_ROOT:-${M1_WS_ROOT}}"
M1_PX4_ROOT="${M1_PX4_ROOT:-${M1_WS_ROOT}/.external/px4}"
M1_PX4_BUILD="${M1_PX4_BUILD:-${M1_PX4_ROOT}/build/amovlab_sitl_default}"

# These setup paths intentionally come from the configurable local ROS/PX4
# installations, so ShellCheck cannot resolve them statically.
# shellcheck disable=SC1090,SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${M1_PROMETHEUS_ROOT}/devel/setup.bash" --extend
# shellcheck disable=SC1090
source "${M1_WS_ROOT}/devel/setup.bash" --extend
# shellcheck disable=SC1090
source "${M1_PX4_ROOT}/Tools/setup_gazebo.bash" \
  "${M1_PX4_ROOT}" "${M1_PX4_BUILD}"

export ROS_PACKAGE_PATH="${M1_WS_ROOT}/Modules/brick_aerial_perception:${M1_WS_ROOT}/Modules/FAST_LIO:${M1_WS_ROOT}/Modules/tutorial_demo:${M1_WS_ROOT}/Simulator/gazebo_simulator:${M1_WS_ROOT}/Modules/uav_control:${M1_PX4_ROOT}:${M1_PX4_ROOT}/Tools/sitl_gazebo:${ROS_PACKAGE_PATH}"
export GAZEBO_MODEL_PATH="${M1_WS_ROOT}/Simulator/gazebo_simulator/gazebo_models/uav_models:${M1_WS_ROOT}/Simulator/gazebo_simulator/gazebo_models/ugv_models:${M1_WS_ROOT}/Simulator/gazebo_simulator/gazebo_models/sensor_models:${M1_WS_ROOT}/Simulator/gazebo_simulator/gazebo_models/scene_models:${GAZEBO_MODEL_PATH}"

exec roslaunch brick_aerial_perception m1_aerial_perception.launch "$@"
