#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-galactic}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "错误: 未找到 ROS 2 环境文件: $ROS_SETUP" >&2
    exit 1
fi

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
if [[ -f "$WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
fi
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
    for candidate in "$WS/.cyclonedds.xml" "$HOME/.cyclonedds.xml"; do
        if [[ -f "$candidate" ]]; then
            export CYCLONEDDS_URI="file://${candidate}"
            break
        fi
    done
fi

if ! ros2 pkg prefix robot_status_web >/dev/null 2>&1; then
    echo "错误: 未找到已安装的 robot_status_web。请先在工作区根目录执行:" >&2
    echo "  source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --packages-up-to robot_status_web" >&2
    exit 1
fi

launch_args=()
if [[ -n "${ROBOT_STATUS_CONFIG:-}" ]]; then
    launch_args+=("config_file:=${ROBOT_STATUS_CONFIG}")
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
    echo "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
else
    echo "提示: 未设置 CYCLONEDDS_URI，将使用 CycloneDDS 默认发现配置。"
fi
exec ros2 launch robot_status_web robot_status_web.launch.py "${launch_args[@]}"
