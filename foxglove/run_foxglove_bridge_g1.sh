#!/usr/bin/env zsh
set -eo pipefail

ROOT_DIR="/home/guobing/My_Repositories/unitree_ros2"

cd "$ROOT_DIR"

source "$ROOT_DIR/setup.sh"
source /opt/ros/jazzy/setup.zsh
source "$ROOT_DIR/install/setup.zsh"
source "$ROOT_DIR/example/install/setup.zsh"

ros2 daemon stop || true
ros2 daemon start

# 仅桥接 G1 可视化常用且稳定的话题，避免触发 bridge 的 rosgraph 异常。
TOPIC_WHITELIST="['^/(lowstate|lf/lowstate|secondary_imu|lf/secondary_imu|wirelesscontroller|utlidar/imu_livox_mid360|utlidar/cloud_livox_mid360|grid_clouds|safe_clouds|warning_clouds|pre_collision_clouds|collision_clouds|pre_safe_clouds|dog_imu_raw|dog_odom|frontvideostream)$']"

exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
	port:=8765 \
	topic_whitelist:="$TOPIC_WHITELIST" \
	service_whitelist:="['^$']" \
	client_topic_whitelist:="['^$']" \
	param_whitelist:="['^$']" \
	include_hidden:=false
