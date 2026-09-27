#!/usr/bin/env bash
# Authenticate on the caller's terminal before roslaunch detaches its children.
set -euo pipefail
autocar_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
takeover_args=(--takeover-video-address)
if [[ "${1:-}" == "--keep-video-address" ]]; then
    takeover_args=()
    shift
fi
if [[ $# -eq 0 ]]; then
    set -- roslaunch livox_ros_driver2 msg_MID360.launch
fi
bash "$autocar_root/scripts/ros1.sh" /usr/bin/python3 \
    "$autocar_root/src/livox_ros_driver2/scripts/start_mid360.py" \
    --config "$autocar_root/src/livox_ros_driver2/config/MID360_config.json" \
    "${takeover_args[@]}" --prepare-only
exec bash "$autocar_root/scripts/ros1.sh" "$@"
