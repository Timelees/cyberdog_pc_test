#!/usr/bin/env bash
set -euo pipefail

# 机器人命名空间列表（按需追加，如 cyberdog_3）
ROBOT_NAMESPACES=(
    cyberdog_1
    cyberdog_2
)

# CycloneDDS 配置文件候选路径（docker 内 root 用户 HOME 常为 /root，需显式列出）
CYCLONEDDS_XML_PATHS=(
    "/home/lee/.cyclonedds.xml"
    "$HOME/.cyclonedds.xml"
)

# lifecycle / service 超时（秒）
LIFECYCLE_TIMEOUT=30
SERVICE_TIMEOUT=30

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
    if ! timeout "$LIFECYCLE_TIMEOUT" ros2 lifecycle set "$node" "$transition"; then
        local rc=$?
        if [[ $rc -eq 124 ]]; then
            echo "错误: lifecycle set ${node} ${transition} 超时（${LIFECYCLE_TIMEOUT}s）" >&2
            echo "  通常是 CYCLONEDDS_URI 未生效或 DDS 无法与机器人通信。" >&2
        fi
        return "$rc"
    fi
}

service_call_setbool() {
    local service="$1"
    local data="$2"
    if ! timeout "$SERVICE_TIMEOUT" ros2 service call "$service" std_srvs/srv/SetBool "{data: ${data}}"; then
        local rc=$?
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
done

echo "==> 全部机器人 VIO 启动完成"
