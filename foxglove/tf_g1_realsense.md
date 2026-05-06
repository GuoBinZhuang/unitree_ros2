# G1 + RealSense + Foxglove TF 说明

本文整理 `foxglove/run_foxglove_bridge_g1.sh`、G1 URDF 和 RealSense ROS2 wrapper 共同构成的 TF 树，用于排查 Foxglove 中的 `Missing transform` 报错。D435i 安装姿态保持本地官方 URDF 原值；宇树服务控制台 2026-01-23 说明中的 Z 值与本地 URDF 相差 0.01 m，当前认为可接受，不修改官方模型资产。

## 当前 TF 来源

### 1. G1 机器人模型

`run_foxglove_bridge_g1.sh` 会启动：

```bash
ros2 run unitree_ros2_example g1_lowstate_to_joint_states
ros2 run robot_state_publisher robot_state_publisher "$G1_URDF_PATH"
```

这两者共同提供 G1 的主要 TF：

```text
pelvis
├── pelvis_contour_link
├── imu_in_pelvis
├── left/right leg links
├── waist_yaw_link -> waist_roll_link -> torso_link
│   ├── head_link
│   ├── imu_in_torso
│   ├── d435_link
│   ├── mid360_link
│   └── arm/hand links
```

关键固定关节来自 `g1_description/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf`：

```text
torso_link -> d435_link
  xyz = 0.0576235 0.01753 0.42987
  rpy = 0 0.8307767239493009 0

torso_link -> mid360_link
  xyz = 0.0002835 0.00003 0.41618
  rpy = 0 0.04014257279586953 0
```

`d435_link` 的 pitch 为 `0.8307767239493009 rad`，约等于 `47.6°`，与宇树服务控制台说明的 `47.5°`一致到可接受精度；roll/yaw 均为 0。服务控制台说明中的 Z 值为 `0.41987`，本地 URDF 为 `0.42987`，差 0.01 m，当前保留本地 URDF。

### 2. Mid-360 点云 frame 兼容层

脚本默认点云话题使用 `LIDAR_FRAME_ID=livox_frame`。如果它不是 `mid360_link`，脚本会发布：

```text
mid360_link -> livox_frame
```

默认 `roll=pi`，用于补偿当前 Mid-360 点云 frame 和 URDF 安装姿态之间的上下翻转。

### 3. body frame 兼容层

部分机器人端消息会把 `frame_id` 写成 `body`，但 G1 URDF 根帧是 `pelvis`。脚本默认发布一个同位姿别名：

```text
pelvis -> body
```

相关环境变量：

```bash
ENABLE_G1_BODY_FRAME_ALIAS=true
G1_BODY_PARENT_FRAME=pelvis
G1_BODY_FRAME=body
```

如果机器人端已经发布了 `pelvis -> body` 或 `body -> pelvis`，需要关闭这里的别名，避免重复 TF：

```bash
ENABLE_G1_BODY_FRAME_ALIAS=false
```

### 4. RealSense 相机内部 TF

RealSense ROS2 wrapper 默认 `publish_tf:=true`，会发布相机内部静态 TF。官方说明中，`base_frame_id` 是相机根帧后缀，默认 `"link"`；根帧由 `[camera_name]_[base_frame_id]` 组成，因此默认根帧通常是：

```text
camera_link
```

内部链路通常类似：

```text
camera_link
├── camera_color_frame -> camera_color_optical_frame
├── camera_depth_frame -> camera_depth_optical_frame
├── camera_accel_frame / camera_gyro_frame
```

RealSense wrapper 只知道相机内部结构，不知道它装在 G1 哪个 link 上。G1 侧必须提供：

```text
d435_link -> camera_link
```

脚本提供了可选静态 TF：

```bash
ENABLE_REALSENSE_STATIC_TF=false
REALSENSE_PARENT_FRAME=d435_link
REALSENSE_BASE_FRAME=camera_link
REALSENSE_TF_X=0
REALSENSE_TF_Y=0
REALSENSE_TF_Z=0
REALSENSE_TF_ROLL=0
REALSENSE_TF_PITCH=0
REALSENSE_TF_YAW=0
```

如果机器人端 RealSense root frame 就是 `camera_link`，并且没有其他地方连接到 `d435_link`，应打开：

```bash
ENABLE_REALSENSE_STATIC_TF=true
```

## 推荐 TF 树

最干净的目标形态是：

```text
pelvis
└── waist_yaw_link -> waist_roll_link -> torso_link
    ├── mid360_link -> livox_frame
    └── d435_link -> camera_link
        ├── camera_color_frame -> camera_color_optical_frame
        └── camera_depth_frame -> camera_depth_optical_frame
```

`body` 只作为兼容别名使用，不建议把新传感器长期挂到 `body` 下。

## 两种 RealSense 接法

### 接法 A：使用现场 G1 `/camera/*` topic

当前现场 G1 的 RealSense topic 形态是：

```text
/camera/color/image_raw
/camera/depth/image_rect_raw
/camera/depth/color/points
```

这通常意味着 RealSense root frame 仍是 `camera_link`。本机桥接脚本打开：

```bash
ENABLE_REALSENSE_STATIC_TF=true
REALSENSE_PARENT_FRAME=d435_link
REALSENSE_BASE_FRAME=camera_link
```

优点：与当前 G1 实际 topic list 一致，不用额外传命名参数。  
缺点：多一个 `d435_link -> camera_link` 静态 TF 兼容层。

### 接法 B：让 RealSense root frame 直接对齐 URDF 的 `d435_link`

机器人端启动 RealSense 时使用：

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera \
  camera_name:=d435 \
  base_frame_id:=link \
  publish_tf:=true \
  pointcloud.enable:=true \
  align_depth.enable:=true \
  enable_sync:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  depth_module.depth_profile:=1280x720x30 \
  rgb_camera.color_profile:=1280x720x30
```

这样 RealSense root frame 会变成 `d435_link`，能直接接到 URDF 里的 `d435_link`。  
注意：topic 也会变成 `/camera/d435/...`，需要同步调整：

```bash
REALSENSE_TOPIC_PREFIX=/camera/d435
```

并把 Foxglove layout 中的 `/camera/color/...`、`/camera/depth/...` 改成 `/camera/d435/color/...`、`/camera/d435/depth/...`。

## 优化建议

1. 优先让 TF 来源唯一。
   - `robot_state_publisher` 负责 G1 URDF 内部 TF。
   - RealSense wrapper 负责相机内部 TF。
   - 静态 TF 只负责跨系统连接，例如 `d435_link -> camera_link`。

2. 不要让两个节点发布同一条 TF。
   - 如果机器人端已经有 `pelvis -> body`，关闭 `ENABLE_G1_BODY_FRAME_ALIAS`。
   - 如果 RealSense root 已经是 `d435_link`，关闭 `ENABLE_REALSENSE_STATIC_TF`。

3. 避免把传感器挂到 `pelvis` 或 `body`。
   - D435i 应挂到 `d435_link`。
   - Mid-360 应挂到 `mid360_link`。
   - `pelvis` 是机器人根帧，不适合直接承载传感器安装姿态。

4. 统一 topic prefix 和 frame 命名。
   - 当前 layout 默认 topic prefix 是 `/camera`，这是现场 G1 实测结果。
   - 部分 RealSense ROS2 配置会使用 `/camera/camera` 或 `/camera/d435`，遇到时要同步改 layout 和 `REALSENSE_TOPIC_PREFIX`。
   - 如果使用 `camera_name:=d435`，topic prefix 会变成 `/camera/d435`。

5. 排查 TF 时先看树，再看数据。

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo pelvis body
ros2 run tf2_ros tf2_echo d435_link camera_link
ros2 run tf2_ros tf2_echo d435_link camera_color_optical_frame
ros2 topic echo /tf_static --once
```

如果 Foxglove 报：

```text
Missing transform from frame <A> to frame <B>
```

就用：

```bash
ros2 run tf2_ros tf2_echo B A
```

验证是否真的存在从目标显示坐标系到消息 frame 的链路。

## 推荐当前运行方式

保持当前 Foxglove layout 不改 topic 时：

```bash
ENABLE_G1_BODY_FRAME_ALIAS=true \
ENABLE_REALSENSE_STATIC_TF=true \
REALSENSE_PARENT_FRAME=d435_link \
REALSENSE_BASE_FRAME=camera_link \
./foxglove/run_foxglove_bridge_g1.sh
```

机器人端 RealSense：

```bash
ros2 launch realsense2_camera rs_launch.py \
  publish_tf:=true \
  pointcloud.enable:=true \
  align_depth.enable:=true \
  enable_sync:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  depth_module.depth_profile:=1280x720x30 \
  rgb_camera.color_profile:=1280x720x30
```

如果实际 topic 是 `/camera/camera/...`，本机启动时同步设置：

```bash
REALSENSE_TOPIC_PREFIX=/camera/camera
```

并修改 Foxglove layout 的 D435i topic。

## 参考

- RealSense ROS2 wrapper 参数说明：<https://github.com/realsenseai/realsense-ros#parameters>
- RealSense ROS2 wrapper 官方文档：<https://dev.realsenseai.com/docs/ros2-wrapper/>
