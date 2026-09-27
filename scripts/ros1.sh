#!/usr/bin/env bash
# Run a command with only system Noetic and this workspace overlaid.
set -e
autocar_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES
unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export ROS_HOME="$autocar_root/.ros"
# Keep command options such as --help out of catkin's setup argument parser.
source /opt/ros/noetic/setup.bash --
if [[ -f "$autocar_root/devel/setup.bash" ]]; then
    source "$autocar_root/devel/setup.bash" --extend
elif [[ -f "$autocar_root/install/setup.bash" ]]; then
    source "$autocar_root/install/setup.bash" --extend
fi
cd "$autocar_root"
if [[ $# -eq 0 ]]; then
    exec bash --noprofile --norc
fi
exec "$@"
