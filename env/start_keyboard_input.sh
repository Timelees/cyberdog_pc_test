#!/usr/bin/env bash
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/galactic/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
  for candidate in "$WS/.cyclonedds.xml" "$HOME/.cyclonedds.xml"; do
    if [[ -f "$candidate" ]]; then
      export CYCLONEDDS_URI="file://${candidate}"
      break
    fi
  done
fi
namespace="${ROBOT_NAMESPACE:-cyberdog_1}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} namespace=${namespace}"
[[ -n "${CYCLONEDDS_URI:-}" ]] && echo "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
exec ros2 run keyboard_input keyboard_input_node --ros-args -p "namespace:=${namespace}" "$@"
