#!/usr/bin/env bash
# 在 roslaunch 分离子进程前，先通过调用者终端完成网络助手的 sudo 认证。
set -euo pipefail
autocar_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# 默认允许把视频网卡上的指定静态地址移给雷达网卡；退出时不会自动恢复视频链路。
takeover_args=(--takeover-video-address)
if [[ "${1:-}" == "--keep-video-address" ]]; then
    takeover_args=()
    shift
fi
if [[ $# -eq 0 ]]; then
    set -- roslaunch livox_ros_driver2 msg_MID360.launch
fi
# set -e 保证网络准备失败后立即退出，不执行后面的 ROS 启动命令。
bash "$autocar_root/scripts/ros1.sh" /usr/bin/python3 \
    "$autocar_root/src/livox_ros_driver2/scripts/start_mid360.py" \
    --config "$autocar_root/src/livox_ros_driver2/config/MID360_config.json" \
    "${takeover_args[@]}" --prepare-only
exec bash "$autocar_root/scripts/ros1.sh" "$@"
