#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SERVICE_NAME="g1-emergency-stop.service"
REPO_DIR="${DEFAULT_REPO_DIR}"
RUN_USER="${SUDO_USER:-${USER}}"
ROS_DISTRO=""

usage() {
    cat <<'EOF'
用法:
  ./scripts/install_g1_emergency_stop_service.sh [选项]

选项:
  --repo-dir <path>       仓库根目录，默认当前脚本上一级目录
  --service-name <name>   systemd 服务名，默认 g1-emergency-stop.service
  --user <name>           运行服务的系统用户，默认当前用户
  --ros-distro <name>     指定 ROS2 发行版，如 jazzy/humble/foxy
  -h, --help              显示帮助
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --repo-dir)
            REPO_DIR="$2"
            shift 2
            ;;
        --service-name)
            SERVICE_NAME="$2"
            shift 2
            ;;
        --user)
            RUN_USER="$2"
            shift 2
            ;;
        --ros-distro)
            ROS_DISTRO="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[ERROR] 未知参数: $1" >&2
            usage
            exit 2
            ;;
    esac
done

if [[ ! -d "${REPO_DIR}" ]]; then
    echo "[ERROR] 仓库目录不存在: ${REPO_DIR}" >&2
    exit 1
fi

RUN_SCRIPT="${REPO_DIR}/scripts/run_g1_emergency_stop_node.sh"
if [[ ! -x "${RUN_SCRIPT}" ]]; then
    echo "[ERROR] 运行脚本不存在或不可执行: ${RUN_SCRIPT}" >&2
    echo "        请先执行: chmod +x ${RUN_SCRIPT}" >&2
    exit 1
fi

ESTOP_BIN="${REPO_DIR}/install/unitree_ros2_example/lib/unitree_ros2_example/g1_emergency_stop_node"
if [[ ! -x "${ESTOP_BIN}" ]]; then
    echo "[ERROR] 未找到可执行文件: ${ESTOP_BIN}" >&2
    echo "        请先在仓库根目录执行 colcon build。" >&2
    exit 1
fi

if ! id -u "${RUN_USER}" >/dev/null 2>&1; then
    echo "[ERROR] 用户不存在: ${RUN_USER}" >&2
    exit 1
fi

RUN_HOME="$(getent passwd "${RUN_USER}" | cut -d: -f6)"
RUN_GROUP="$(id -gn "${RUN_USER}")"

if [[ -z "${ROS_DISTRO}" ]]; then
    for distro in jazzy humble foxy; do
        if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
            ROS_DISTRO="${distro}"
            break
        fi
    done
fi

if [[ -z "${ROS_DISTRO}" ]]; then
    echo "[ERROR] 未检测到 ROS2 发行版，请使用 --ros-distro 显式指定。" >&2
    exit 1
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "[ERROR] ROS setup 文件不存在: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
    exit 1
fi

if [[ ${EUID} -eq 0 ]]; then
    SUDO_CMD=""
else
    if ! command -v sudo >/dev/null 2>&1; then
        echo "[ERROR] 需要 root 权限安装 systemd 服务，请安装 sudo 或切换 root。" >&2
        exit 1
    fi
    SUDO_CMD="sudo"
fi

SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
TMP_SERVICE="$(mktemp)"
trap 'rm -f "${TMP_SERVICE}"' EXIT

cat >"${TMP_SERVICE}" <<EOF
[Unit]
Description=Unitree G1 Emergency Stop ROS2 Node
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${RUN_USER}
Group=${RUN_GROUP}
WorkingDirectory=${REPO_DIR}
Environment=HOME=${RUN_HOME}
Environment=ROS_SETUP_PATH=/opt/ros/${ROS_DISTRO}/setup.bash
ExecStart=/usr/bin/env bash ${RUN_SCRIPT}
Restart=always
RestartSec=2
KillSignal=SIGINT
TimeoutStopSec=15
NoNewPrivileges=true
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

${SUDO_CMD} install -m 644 "${TMP_SERVICE}" "${SERVICE_PATH}"
${SUDO_CMD} systemctl daemon-reload
${SUDO_CMD} systemctl enable --now "${SERVICE_NAME}"

echo "[OK] 已安装并启动: ${SERVICE_NAME}"
echo "[INFO] 查看状态: sudo systemctl status ${SERVICE_NAME} --no-pager"
echo "[INFO] 查看日志: sudo journalctl -u ${SERVICE_NAME} -f"
