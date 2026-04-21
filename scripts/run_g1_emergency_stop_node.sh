#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source_without_nounset() {
    local file="$1"
    set +u
    # shellcheck disable=SC1090
    source "${file}"
    set -u
}

resolve_ros_setup() {
    if [[ -n "${ROS_SETUP_PATH:-}" && -f "${ROS_SETUP_PATH}" ]]; then
        echo "${ROS_SETUP_PATH}"
        return
    fi

    local distro
    for distro in jazzy humble foxy; do
        if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
            echo "/opt/ros/${distro}/setup.bash"
            return
        fi
    done

    return 1
}

ROS_SETUP="$(resolve_ros_setup || true)"
if [[ -z "${ROS_SETUP}" ]]; then
    echo "[ERROR] 未找到 ROS2 setup.bash，请设置 ROS_SETUP_PATH 或安装 ROS2。" >&2
    exit 1
fi

# 先加载 ROS2 基础环境，再加载仓库内的网络与工作区 overlay。
source_without_nounset "${ROS_SETUP}"

if [[ -f "${ROOT_DIR}/setup.sh" ]]; then
    source_without_nounset "${ROOT_DIR}/setup.sh"
fi

if [[ -f "${ROOT_DIR}/install/setup.sh" ]]; then
    source_without_nounset "${ROOT_DIR}/install/setup.sh"
fi

if [[ -f "${ROOT_DIR}/example/install/setup.sh" ]]; then
    source_without_nounset "${ROOT_DIR}/example/install/setup.sh"
fi

exec ros2 run unitree_ros2_example g1_emergency_stop_node
