#!/usr/bin/env zsh
set -euo pipefail

ROOT_DIR="/home/guobing/My_Repositories/unitree_ros2"
FOXGLOVE_DIR="$ROOT_DIR/foxglove"
UNITREE_SDK2_ROOT="${UNITREE_SDK2_ROOT:-/home/guobing/unitree_sdk2}"
UNITREE_SDK2_THIRDPARTY_LIB_DIR="${UNITREE_SDK2_THIRDPARTY_LIB_DIR:-$UNITREE_SDK2_ROOT/thirdparty/lib/$(uname -m)}"
G1_URDF_PATH="${G1_URDF_PATH:-$FOXGLOVE_DIR/g1_description/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf}"
LOWSTATE_TOPIC="${LOWSTATE_TOPIC:-/lowstate}"
JOINT_STATE_TOPIC="${JOINT_STATE_TOPIC:-/joint_states}"
ENABLE_INSPIRE_HAND_BRIDGE="${ENABLE_INSPIRE_HAND_BRIDGE:-true}"
HAND_JOINT_STATE_TOPIC="${HAND_JOINT_STATE_TOPIC:-$JOINT_STATE_TOPIC}"
INSPIRE_HAND_NETWORK_INTERFACE="${INSPIRE_HAND_NETWORK_INTERFACE:-}"
INSPIRE_HAND_LEFT_TOPIC="${INSPIRE_HAND_LEFT_TOPIC:-rt/inspire_hand/state/l}"
INSPIRE_HAND_RIGHT_TOPIC="${INSPIRE_HAND_RIGHT_TOPIC:-rt/inspire_hand/state/r}"
INSPIRE_HAND_INVERT_THUMBS="${INSPIRE_HAND_INVERT_THUMBS:-true}"
ENABLE_INSPIRE_TOUCH_BRIDGE="${ENABLE_INSPIRE_TOUCH_BRIDGE:-true}"
INSPIRE_HAND_LEFT_TOUCH_TOPIC="${INSPIRE_HAND_LEFT_TOUCH_TOPIC:-rt/inspire_hand/touch/l}"
INSPIRE_HAND_RIGHT_TOUCH_TOPIC="${INSPIRE_HAND_RIGHT_TOUCH_TOPIC:-rt/inspire_hand/touch/r}"
INSPIRE_TOUCH_LEFT_IMAGE_TOPIC="${INSPIRE_TOUCH_LEFT_IMAGE_TOPIC:-/inspire_hand/touch/left/image}"
INSPIRE_TOUCH_RIGHT_IMAGE_TOPIC="${INSPIRE_TOUCH_RIGHT_IMAGE_TOPIC:-/inspire_hand/touch/right/image}"
INSPIRE_TOUCH_LEFT_SUMMARY_NAMESPACE="${INSPIRE_TOUCH_LEFT_SUMMARY_NAMESPACE:-/inspire_hand/touch/left}"
INSPIRE_TOUCH_RIGHT_SUMMARY_NAMESPACE="${INSPIRE_TOUCH_RIGHT_SUMMARY_NAMESPACE:-/inspire_hand/touch/right}"
ENABLE_REALSENSE_D435I_BRIDGE="${ENABLE_REALSENSE_D435I_BRIDGE:-true}"
REALSENSE_TOPIC_PREFIX="${REALSENSE_TOPIC_PREFIX:-/camera}"
ENABLE_REALSENSE_STATIC_TF="${ENABLE_REALSENSE_STATIC_TF:-false}"
REALSENSE_PARENT_FRAME="${REALSENSE_PARENT_FRAME:-d435_link}"
REALSENSE_BASE_FRAME="${REALSENSE_BASE_FRAME:-camera_link}"
REALSENSE_TF_X="${REALSENSE_TF_X:-0}"
REALSENSE_TF_Y="${REALSENSE_TF_Y:-0}"
REALSENSE_TF_Z="${REALSENSE_TF_Z:-0}"
REALSENSE_TF_ROLL="${REALSENSE_TF_ROLL:-0}"
REALSENSE_TF_PITCH="${REALSENSE_TF_PITCH:-0}"
REALSENSE_TF_YAW="${REALSENSE_TF_YAW:-0}"
ENABLE_TELEIMAGER_IMAGE_BRIDGE="${ENABLE_TELEIMAGER_IMAGE_BRIDGE:-false}"
TELEIMAGER_HOST="${TELEIMAGER_HOST:-192.168.123.164}"
TELEIMAGER_REQUEST_PORT="${TELEIMAGER_REQUEST_PORT:-60000}"
TELEIMAGER_TOPIC_PREFIX="${TELEIMAGER_TOPIC_PREFIX:-/teleimager}"
TELEIMAGER_CAMERAS="${TELEIMAGER_CAMERAS:-head,left_wrist,right_wrist}"
TELEIMAGER_PUBLISH_RAW="${TELEIMAGER_PUBLISH_RAW:-false}"
TELEIMAGER_RATE="${TELEIMAGER_RATE:-30}"
TELEIMAGER_PYTHON="${TELEIMAGER_PYTHON:-/usr/bin/python3}"
TELEIMAGER_CLIENT_PYTHON="${TELEIMAGER_CLIENT_PYTHON:-/home/guobing/anaconda3/bin/python}"
ENABLE_G1_BODY_FRAME_ALIAS="${ENABLE_G1_BODY_FRAME_ALIAS:-true}"
G1_BODY_PARENT_FRAME="${G1_BODY_PARENT_FRAME:-pelvis}"
G1_BODY_FRAME="${G1_BODY_FRAME:-body}"
G1_BODY_TF_X="${G1_BODY_TF_X:-0}"
G1_BODY_TF_Y="${G1_BODY_TF_Y:-0}"
G1_BODY_TF_Z="${G1_BODY_TF_Z:-0}"
G1_BODY_TF_ROLL="${G1_BODY_TF_ROLL:-0}"
G1_BODY_TF_PITCH="${G1_BODY_TF_PITCH:-0}"
G1_BODY_TF_YAW="${G1_BODY_TF_YAW:-0}"
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

normalize_topic_prefix() {
  local fallback="${2:-/camera}"
  local prefix="${1:-$fallback}"

  [[ -z "$prefix" ]] && prefix="$fallback"
  [[ "$prefix" != /* ]] && prefix="/$prefix"
  prefix="${prefix%/}"
  [[ -z "$prefix" ]] && prefix="$fallback"

  printf '%s\n' "$prefix"
}

build_topic_whitelist() {
  local whitelist="["
  local pattern

  for pattern in "$@"; do
    [[ "$whitelist" != "[" ]] && whitelist+=","
    whitelist+="'$pattern'"
  done

  whitelist+="]"
  printf '%s\n' "$whitelist"
}

cd "$ROOT_DIR"

REALSENSE_TOPIC_PREFIX="$(normalize_topic_prefix "$REALSENSE_TOPIC_PREFIX")"
TELEIMAGER_TOPIC_PREFIX="$(normalize_topic_prefix "$TELEIMAGER_TOPIC_PREFIX" "/teleimager")"

if [[ ! -f "$G1_URDF_PATH" ]]; then
  echo "未找到 G1 URDF: $G1_URDF_PATH" >&2
  exit 1
fi

safe_source "$ROOT_DIR/setup.sh"
safe_source /opt/ros/jazzy/setup.zsh
safe_source "$ROOT_DIR/install/setup.zsh"
safe_source "$ROOT_DIR/example/install/setup.zsh"

if [[ -d "$UNITREE_SDK2_THIRDPARTY_LIB_DIR" ]]; then
  export LD_LIBRARY_PATH="$UNITREE_SDK2_THIRDPARTY_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

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

if [[ "$ENABLE_INSPIRE_HAND_BRIDGE:l" == "true" ]]; then
  if ! ros2 pkg prefix unitree_ros2_example >/dev/null 2>&1; then
    echo "未找到 unitree_ros2_example 包，无法启动 Inspire Hand JointState 桥接。" >&2
    exit 1
  fi

  typeset -a HAND_BRIDGE_ARGS=(
    unitree_ros2_example
    g1_inspire_hand_to_joint_states
    --ros-args
    -p "joint_state_topic:=$HAND_JOINT_STATE_TOPIC"
    -p "left_hand_topic:=$INSPIRE_HAND_LEFT_TOPIC"
    -p "right_hand_topic:=$INSPIRE_HAND_RIGHT_TOPIC"
    -p "invert_thumbs:=$INSPIRE_HAND_INVERT_THUMBS"
  )

  if [[ -n "$INSPIRE_HAND_NETWORK_INTERFACE" ]]; then
    HAND_BRIDGE_ARGS+=(-p "network_interface:=$INSPIRE_HAND_NETWORK_INTERFACE")
  fi

  start_bg_checked ros2 run "${HAND_BRIDGE_ARGS[@]}"
fi

if [[ "$ENABLE_INSPIRE_TOUCH_BRIDGE:l" == "true" ]]; then
  typeset -a TOUCH_BRIDGE_ARGS=(
    unitree_ros2_example
    g1_inspire_hand_touch_to_visualization
    --ros-args
    -p "left_touch_topic:=$INSPIRE_HAND_LEFT_TOUCH_TOPIC"
    -p "right_touch_topic:=$INSPIRE_HAND_RIGHT_TOUCH_TOPIC"
    -p "left_image_topic:=$INSPIRE_TOUCH_LEFT_IMAGE_TOPIC"
    -p "right_image_topic:=$INSPIRE_TOUCH_RIGHT_IMAGE_TOPIC"
    -p "left_summary_namespace:=$INSPIRE_TOUCH_LEFT_SUMMARY_NAMESPACE"
    -p "right_summary_namespace:=$INSPIRE_TOUCH_RIGHT_SUMMARY_NAMESPACE"
  )

  if [[ -n "$INSPIRE_HAND_NETWORK_INTERFACE" ]]; then
    TOUCH_BRIDGE_ARGS+=(-p "network_interface:=$INSPIRE_HAND_NETWORK_INTERFACE")
  fi

  start_bg_checked ros2 run "${TOUCH_BRIDGE_ARGS[@]}"
fi

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

if [[ "$ENABLE_REALSENSE_STATIC_TF:l" == "true" ]]; then
  start_bg_checked \
    ros2 run tf2_ros static_transform_publisher \
      --x "$REALSENSE_TF_X" \
      --y "$REALSENSE_TF_Y" \
      --z "$REALSENSE_TF_Z" \
      --roll "$REALSENSE_TF_ROLL" \
      --pitch "$REALSENSE_TF_PITCH" \
      --yaw "$REALSENSE_TF_YAW" \
      --frame-id "$REALSENSE_PARENT_FRAME" \
      --child-frame-id "$REALSENSE_BASE_FRAME"
fi

if [[ "$ENABLE_G1_BODY_FRAME_ALIAS:l" == "true" && "$G1_BODY_FRAME" != "$G1_BODY_PARENT_FRAME" ]]; then
  start_bg_checked \
    ros2 run tf2_ros static_transform_publisher \
      --x "$G1_BODY_TF_X" \
      --y "$G1_BODY_TF_Y" \
      --z "$G1_BODY_TF_Z" \
      --roll "$G1_BODY_TF_ROLL" \
      --pitch "$G1_BODY_TF_PITCH" \
      --yaw "$G1_BODY_TF_YAW" \
      --frame-id "$G1_BODY_PARENT_FRAME" \
      --child-frame-id "$G1_BODY_FRAME"
fi

# 保持白名单保守；当前 Foxglove 对 /sportmodestate 的 schema/encoding 兼容性不好，先不桥接。
typeset -a TOPIC_PATTERNS=(
  '^/(lowstate|lf/lowstate|secondary_imu|lf/secondary_imu|wirelesscontroller|utlidar/imu_livox_mid360|utlidar/cloud_livox_mid360|grid_clouds|safe_clouds|warning_clouds|pre_collision_clouds|collision_clouds|pre_safe_clouds|dog_imu_raw|dog_odom|frontvideostream|joint_states|tf|tf_static|inspire_hand/touch/(left|right)/(image|little|ring|middle|index|thumb|palm))$'
)

if [[ "$ENABLE_REALSENSE_D435I_BRIDGE:l" == "true" ]]; then
  TOPIC_PATTERNS+=("^${REALSENSE_TOPIC_PREFIX}/(color/(image_raw|camera_info)|depth/(camera_info|image_rect_raw|color/points)|aligned_depth_to_color/(image_raw|camera_info)|imu)$")
fi

if [[ "$ENABLE_TELEIMAGER_IMAGE_BRIDGE:l" == "true" ]]; then
  typeset -a TELEIMAGER_PYTHON_CMD
  TELEIMAGER_PYTHON_CMD=("${(@z)TELEIMAGER_PYTHON}")

  if ! "${TELEIMAGER_PYTHON_CMD[@]}" -c "import rclpy; import sensor_msgs; import cv2; import numpy" >/dev/null; then
    echo "Teleimager ROS 图像发布 Python 环境不可用: $TELEIMAGER_PYTHON" >&2
    echo "需要能导入 ROS2(rclpy/sensor_msgs)；Jazzy 的 rclpy 绑定系统 Python 3.12。" >&2
    echo "Jazzy 的 rclpy 绑定系统 Python 3.12；不要使用 Anaconda Python 3.13 启动这个 ROS2 节点。" >&2
    exit 1
  fi

  typeset -a TELEIMAGER_CLIENT_PYTHON_CMD
  TELEIMAGER_CLIENT_PYTHON_CMD=("${(@z)TELEIMAGER_CLIENT_PYTHON}")

  if ! "${TELEIMAGER_CLIENT_PYTHON_CMD[@]}" -c "import zmq" >/dev/null; then
    echo "Teleimager ZMQ 客户端 Python 环境不可用: $TELEIMAGER_CLIENT_PYTHON" >&2
    echo "需要能导入 pyzmq；可通过 TELEIMAGER_CLIENT_PYTHON 指向已安装 pyzmq 的 Python。" >&2
    exit 1
  fi

  typeset -a TELEIMAGER_BRIDGE_ARGS=(
    "$FOXGLOVE_DIR/teleimager_to_ros_image.py"
    --host "$TELEIMAGER_HOST"
    --request-port "$TELEIMAGER_REQUEST_PORT"
    --topic-prefix "$TELEIMAGER_TOPIC_PREFIX"
    --cameras "$TELEIMAGER_CAMERAS"
    --rate "$TELEIMAGER_RATE"
    --client-python "$TELEIMAGER_CLIENT_PYTHON"
  )

  if [[ "$TELEIMAGER_PUBLISH_RAW:l" == "true" ]]; then
    TELEIMAGER_BRIDGE_ARGS+=(--raw)
  fi

  start_bg_checked "${TELEIMAGER_PYTHON_CMD[@]}" "${TELEIMAGER_BRIDGE_ARGS[@]}"
  TOPIC_PATTERNS+=("^${TELEIMAGER_TOPIC_PREFIX}/(head|left_wrist|right_wrist)/(compressed|image)$")
fi

TOPIC_WHITELIST="$(build_topic_whitelist "${TOPIC_PATTERNS[@]}")"

ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:="$BRIDGE_PORT" \
  topic_whitelist:="$TOPIC_WHITELIST" \
  service_whitelist:="['^$']" \
  client_topic_whitelist:="['^$']" \
  param_whitelist:="['^$']" \
  include_hidden:=false
