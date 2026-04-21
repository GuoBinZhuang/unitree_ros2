#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${1:-g1-emergency-stop.service}"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"

if [[ ${EUID} -eq 0 ]]; then
    SUDO_CMD=""
else
    if ! command -v sudo >/dev/null 2>&1; then
        echo "[ERROR] 需要 root 权限卸载 systemd 服务，请安装 sudo 或切换 root。" >&2
        exit 1
    fi
    SUDO_CMD="sudo"
fi

if ${SUDO_CMD} systemctl list-unit-files | grep -q "^${SERVICE_NAME}"; then
    ${SUDO_CMD} systemctl disable --now "${SERVICE_NAME}" || true
fi

if [[ -f "${SERVICE_PATH}" ]]; then
    ${SUDO_CMD} rm -f "${SERVICE_PATH}"
fi

${SUDO_CMD} systemctl daemon-reload

# 清理失败状态，避免 status 里残留 failed 标记。
${SUDO_CMD} systemctl reset-failed "${SERVICE_NAME}" || true

echo "[OK] 已卸载: ${SERVICE_NAME}"
