#!/usr/bin/env zsh
set -euo pipefail

ROOT_DIR="/home/guobing/My_Repositories/unitree_ros2"
FOXGLOVE_DIR="$ROOT_DIR/foxglove"
G1_URDF_PATH="${G1_URDF_PATH:-$FOXGLOVE_DIR/g1_description/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf}"
LOWSTATE_TOPIC="${LOWSTATE_TOPIC:-/lowstate}"
JOINT_STATE_TOPIC="${JOINT_STATE_TOPIC:-/joint_states}"
LIDAR_FRAME_ID="${LIDAR_FRAME_ID:-livox_frame}"
LIDAR_TF_X="${LIDAR_TF_X:-0}"
LIDAR_TF_Y="${LIDAR_TF_Y:-0}"
LIDAR_TF_Z="${LIDAR_TF_Z:-0}"
# 当前 G1 Mid-360 点云 frame 与 URDF 安装姿态存在上下翻转，需要默认补一个 roll=pi。
LIDAR_TF_ROLL="${LIDAR_TF_ROLL:-3.14159265359}"
LIDAR_TF_PITCH="${LIDAR_TF_PITCH:-0}"
LIDAR_TF_YAW="${LIDAR_TF_YAW:-0}"
BRIDGE_PORT="${FOXGLOVE_BRIDGE_PORT:-8765}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$ROOT_DIR/log/foxglove}"

typeset -a BG_PIDS=()

safe_source() {
  local source_path="$1"
  set +u
  source "$source_path"
  set -u
}

cd "$ROOT_DIR"

safe_source "$ROOT_DIR/setup.sh"
safe_source /opt/ros/jazzy/setup.zsh
safe_source "$ROOT_DIR/install/setup.zsh"
safe_source "$ROOT_DIR/example/install/setup.zsh"

mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

cleanup() {
  local status=$?
  trap - EXIT INT TERM

  for pid in "${BG_PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait "${BG_PIDS[@]:-}" 2>/dev/null || true
  exit "$status"
}

trap cleanup EXIT INT TERM

start_bg_checked() {
  "$@" &
  local pid=$!
  BG_PIDS+=($pid)
  sleep 1
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "后台进程启动失败: $*" >&2
    wait "$pid"
    exit 1
  fi
}

ros2 daemon stop || true
ros2 daemon start

if ! ros2 pkg prefix unitree_ros2_example >/dev/null 2>&1; then
  echo "ROS 环境未正确加载到 unitree_ros2_example 包，请先确认 install 已构建并重新 source。" >&2
  exit 1
fi

# 用 lowstate 驱动完整 JointState，并让 Mid-360 使用 URDF 里的真实 link，而不是手工伪造 frame。
start_bg_checked \
  ros2 run unitree_ros2_example g1_lowstate_to_joint_states \
    --ros-args \
    -p lowstate_topic:="$LOWSTATE_TOPIC" \
    -p joint_state_topic:="$JOINT_STATE_TOPIC" \
    -p urdf_path:="$G1_URDF_PATH"

start_bg_checked \
  ros2 run robot_state_publisher robot_state_publisher "$G1_URDF_PATH"

if [[ "$LIDAR_FRAME_ID" != "mid360_link" ]]; then
  start_bg_checked \
    ros2 run tf2_ros static_transform_publisher \
      --x "$LIDAR_TF_X" \
      --y "$LIDAR_TF_Y" \
      --z "$LIDAR_TF_Z" \
      --roll "$LIDAR_TF_ROLL" \
      --pitch "$LIDAR_TF_PITCH" \
      --yaw "$LIDAR_TF_YAW" \
      --frame-id mid360_link \
      --child-frame-id "$LIDAR_FRAME_ID"
fi

# 保持白名单保守；当前 Foxglove 对 /sportmodestate 的 schema/encoding 兼容性不好，先不桥接。
TOPIC_WHITELIST="['^/(lowstate|lf/lowstate|secondary_imu|lf/secondary_imu|wirelesscontroller|utlidar/imu_livox_mid360|utlidar/cloud_livox_mid360|grid_clouds|safe_clouds|warning_clouds|pre_collision_clouds|collision_clouds|pre_safe_clouds|dog_imu_raw|dog_odom|frontvideostream|joint_states|tf|tf_static)$']"

ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:="$BRIDGE_PORT" \
  topic_whitelist:="$TOPIC_WHITELIST" \
  service_whitelist:="['^$']" \
  client_topic_whitelist:="['^$']" \
  param_whitelist:="['^$']" \
  include_hidden:=false
