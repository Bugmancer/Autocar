#!/usr/bin/env bash
# Run a command with only system Noetic and this workspace overlaid.
set -e
autocar_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES
unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export ROS_HOME="$autocar_root/.ros"
# Some copied or unpacked catkin workspaces lose the executable bit on this
# generated helper. setup.sh invokes it directly, so repair the local mode
# before sourcing the workspace environment.
for setup_util in "$autocar_root/devel/_setup_util.py" \
                  "$autocar_root/install/_setup_util.py"; do
    if [[ -f "$setup_util" && ! -x "$setup_util" ]]; then
        chmod u+x "$setup_util" 2>/dev/null || true
    fi
done
# Catkin copies Python entry points and may leave compiled node targets
# non-executable when the workspace was unpacked with restrictive modes.
# Repair only known node entry points; libraries and pkg-config files stay
# unchanged.
for node_root in "$autocar_root/devel/lib" "$autocar_root/install/lib"; do
    if [[ -d "$node_root" ]]; then
        while IFS= read -r -d '' node_file; do
            chmod u+x "$node_file" 2>/dev/null || true
        done < <(find "$node_root" -type f \( -name '*.py' -o -name '*_node' \) -print0)
    fi
done
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
