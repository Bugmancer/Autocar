#!/usr/bin/env bash
# 清理继承的工作区环境，只叠加系统 Noetic 与当前工作区后执行指定命令。
set -e
autocar_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES
unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export ROS_HOME="$autocar_root/.ros"
# 复制或解压可能丢失 catkin 生成脚本的执行位；加载环境前修复当前工作区文件权限。
for setup_util in "$autocar_root/devel/_setup_util.py" \
                  "$autocar_root/install/_setup_util.py"; do
    if [[ -f "$setup_util" && ! -x "$setup_util" ]]; then
        chmod u+x "$setup_util" 2>/dev/null || true
    fi
done
# 同样修复 devel/install 内已知 Python 入口和节点程序的执行位。
# 仅修改匹配的文件权限，失败时继续交由后续启动报告错误。
for node_root in "$autocar_root/devel/lib" "$autocar_root/install/lib"; do
    if [[ -d "$node_root" ]]; then
        while IFS= read -r -d '' node_file; do
            chmod u+x "$node_file" 2>/dev/null || true
        done < <(find "$node_root" -type f \( -name '*.py' -o -name '*_node' \) -print0)
    fi
done
# 用 -- 隔离调用命令的参数，避免 --help 等被 catkin 环境脚本解析。
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
