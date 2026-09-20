#!/usr/bin/env bash
set -euo pipefail

# 默认启动单机实测的 striker。也可在命令行传入一个或多个 namespace：
#   bash start_vio.sh cyberdog_1 cyberdog_3
if (( $# > 0 )); then
    ROBOT_NAMESPACES=("$@")
else
    ROBOT_NAMESPACES=(cyberdog_1)
fi

# CycloneDDS 配置文件候选路径（docker 内 root 用户 HOME 常为 /root，需显式列出）
CYCLONEDDS_XML_PATHS=(
    "/home/lee/.cyclonedds.xml"
    "$HOME/.cyclonedds.xml"
)

# lifecycle / service 超时（秒）
LIFECYCLE_TIMEOUT=30
SERVICE_TIMEOUT=30
TOPIC_TIMEOUT=10

# ROS2 环境（从 PC 侧远程控制机器狗节点）
# set -u 下 source ROS setup 会因未定义变量报错，需临时关闭
setup_ros() {
    local setup_file=""
    if [[ -f /opt/ros/galactic/setup.bash ]]; then
        setup_file=/opt/ros/galactic/setup.bash
    elif [[ -f /opt/ros2/galactic/setup.bash ]]; then
        setup_file=/opt/ros2/galactic/setup.bash
    fi
    if [[ -n "$setup_file" ]]; then
        set +u
        # shellcheck disable=SC1090
        source "$setup_file"
        set -u
    fi
}

setup_cyclonedds() {
    if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
        return
    fi
    local path
    for path in "${CYCLONEDDS_XML_PATHS[@]}"; do
        if [[ -f "$path" ]]; then
            export CYCLONEDDS_URI="file://${path}"
            return
        fi
    done
    echo "错误: 未找到 CycloneDDS 配置文件。" >&2
    echo "  请设置 CYCLONEDDS_URI，或在以下路径之一放置 .cyclonedds.xml：" >&2
    printf '  - %s\n' "${CYCLONEDDS_XML_PATHS[@]}" >&2
    exit 1
}

refresh_ros2_daemon() {
    echo "==> 刷新 ros2 daemon（使 DDS 配置生效）..."
    ros2 daemon stop >/dev/null 2>&1 || true
    ros2 daemon start
}

wait_for_lifecycle_service() {
    local node="$1"
    local service="${node}/change_state"
    local waited=0
    local max_wait=15

    while (( waited < max_wait )); do
        if timeout 3 ros2 service list 2>/dev/null | grep -qx "$service"; then
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    echo "错误: 找不到 lifecycle 服务 ${service}" >&2
    echo "  当前 CYCLONEDDS_URI=${CYCLONEDDS_URI}" >&2
    echo "  请检查多机通信 Peers 配置，并确认机器人 namespace 正确。" >&2
    return 1
}

lifecycle_set() {
    local node="$1"
    local transition="$2"
    wait_for_lifecycle_service "$node"
    set +e
    timeout "$LIFECYCLE_TIMEOUT" ros2 lifecycle set "$node" "$transition"
    local rc=$?
    set -e
    if (( rc != 0 )); then
        if [[ $rc -eq 124 ]]; then
            echo "错误: lifecycle set ${node} ${transition} 超时（${LIFECYCLE_TIMEOUT}s）" >&2
            echo "  通常是 CYCLONEDDS_URI 未生效或 DDS 无法与机器人通信。" >&2
        fi
        return "$rc"
    fi
}

lifecycle_state() {
    local node="$1"
    timeout 5 ros2 lifecycle get "$node" 2>/dev/null | awk '{print $1}'
}

reset_lifecycle() {
    local node="$1"
    local state
    state="$(lifecycle_state "$node" || true)"
    case "$state" in
        active)
            lifecycle_set "$node" deactivate
            lifecycle_set "$node" cleanup
            ;;
        inactive)
            lifecycle_set "$node" cleanup
            ;;
        unconfigured)
            ;;
        *)
            echo "错误: 无法读取 ${node} lifecycle 状态（当前=${state:-unknown}）" >&2
            return 1
            ;;
    esac
}

require_topic_message() {
    local topic="$1"
    local label="$2"
    local first_line
    first_line="$(timeout "$TOPIC_TIMEOUT" ros2 topic echo "$topic" 2>/dev/null | head -n 1 || true)"
    if [[ -z "$first_line" ]]; then
        echo "错误: ${label} 在 ${TOPIC_TIMEOUT}s 内没有消息：${topic}" >&2
        return 1
    fi
    echo "    ${label} 数据正常：${topic}"
}

service_call_setbool() {
    local service="$1"
    local data="$2"
    set +e
    timeout "$SERVICE_TIMEOUT" ros2 service call \
        "$service" std_srvs/srv/SetBool "{data: ${data}}"
    local rc=$?
    set -e
    if (( rc != 0 )); then
        if [[ $rc -eq 124 ]]; then
            echo "错误: service call ${service} 超时（${SERVICE_TIMEOUT}s）" >&2
        fi
        return "$rc"
    fi
}

setup_ros
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
setup_cyclonedds
echo "==> CYCLONEDDS_URI=${CYCLONEDDS_URI}"
refresh_ros2_daemon

start_camera() {
    local ns="$1"
    echo "==> [$ns] 启动 RealSense 相机..."
    reset_lifecycle "/${ns}/camera/camera"
    echo "    [1/3] camera configure..."
    lifecycle_set "/${ns}/camera/camera" configure
    echo "    [1/3] camera configure 完成"
    echo "    [2/3] camera activate..."
    lifecycle_set "/${ns}/camera/camera" activate
    echo "    [2/3] camera activate 完成"
    echo "    [3/3] 打开 realsense_frame_service..."
    service_call_setbool "/${ns}/camera/realsense_frame_service" true
    echo "    [3/3] realsense_frame_service 已开启"
}

start_vins() {
    local ns="$1"
    echo "==> [$ns] 启动 Mivins (vinslocalization)..."
    reset_lifecycle "/${ns}/vinslocalization"
    echo "    [1/2] vinslocalization configure..."
    lifecycle_set "/${ns}/vinslocalization" configure
    echo "    [1/2] vinslocalization configure 完成"
    echo "    [2/2] vinslocalization activate..."
    lifecycle_set "/${ns}/vinslocalization" activate
    echo "    [2/2] vinslocalization activate 完成"
}

for ns in "${ROBOT_NAMESPACES[@]}"; do
    echo "========== ${ns} =========="
    start_camera "$ns"
    start_vins "$ns"
    echo "==> [$ns] 校验实机数据流..."
    require_topic_message "/${ns}/camera/infra1/image_rect_raw" "相机"
    require_topic_message "/${ns}/odom_slam" "VIO"
    require_topic_message "/${ns}/motion_status" "运动状态"
done

echo "==> 全部机器人 VIO 启动完成"
