# G1 Foxglove 配置审计

本文件记录 `unitree_ros2/foxglove` 当前配置相对官方 `unitree_ros/robots/g1_description` 的检查结果。

## 官方资源对照

已对照：

```text
/home/guobing/My_Repositories/unitree_ros/robots/g1_description
/home/guobing/My_Repositories/unitree_ros2/foxglove/g1_description
```

结论：

- 当前使用的 `g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf` 保持本地官方资源原值：`d435_joint` 为 `xyz="0.0576235 0.01753 0.42987"`、`rpy="0 0.8307767239493009 0"`。
- 宇树服务控制台 2026-01-23 的 G1 D435i 安装说明写的是 `xyz="0.0576235 0.01753 0.41987"`、pitch `47.5°`、`roll/yaw` 为 0。本地 URDF 的 pitch 约为 47.6°，与说明基本一致；Z 值差 0.01 m，当前认为可接受，不修改官方模型资产。
- 官方 G1 模型根帧是 `pelvis`，不是 `body`、`base_link` 或 `trunk`。
- 官方 G1 URDF 已包含传感器安装 link：
  - `torso_link -> d435_link`
  - `torso_link -> mid360_link`
- 因此 Foxglove 侧不应把 D435i 或 Mid-360 直接挂到 `pelvis`/`body`；应优先挂到 `d435_link` / `mid360_link`。

## 当前 Foxglove 布局

主配置文件：

```text
foxglove/g1_foxglove.json
```

当前布局包含：

- D435i RGB：`/camera/color/image_raw`
- D435i Depth：`/camera/depth/image_rect_raw`
- D435i PointCloud：`/camera/depth/color/points`
- Inspire Hand 右手触觉图：`/inspire_hand/touch/right/image`
- G1 URDF：`g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf`
- Mid-360 点云：`/utlidar/cloud_livox_mid360`

已移除会强制制造 TF 依赖的 3D image projection 配置：

```text
projectionFrameId
frameLocked
cameraInfoTopic
```

这样 RGB/Depth 作为普通图像面板显示，点云和 URDF 通过真实 TF 树对齐。

## 当前启动脚本行为

脚本：

```text
foxglove/run_foxglove_bridge_g1.sh
```

主要职责：

- 启动 `g1_lowstate_to_joint_states`
- 启动 `robot_state_publisher`
- 可选启动 Inspire Hand / touch 可视化桥接
- 可选发布 Mid-360 frame 兼容 TF
- 可选发布 RealSense 到 G1 的安装 TF
- 可选发布 `body` frame 兼容别名
- 启动 `foxglove_bridge` 并使用保守 topic 白名单

## TF 目标树

推荐目标：

```text
pelvis
└── waist_yaw_link -> waist_roll_link -> torso_link
    ├── mid360_link -> livox_frame
    └── d435_link -> camera_link
        ├── camera_color_frame -> camera_color_optical_frame
        └── camera_depth_frame -> camera_depth_optical_frame
```

解释：

- `pelvis -> ... -> d435_link` 来自官方 G1 URDF 和 `robot_state_publisher`。
- `d435_link -> camera_link` 是机器人安装座到 RealSense wrapper 根帧的连接，可由本机脚本临时补，也可由机器人端发布。
- `camera_link -> camera_*_optical_frame` 应由 `realsense-ros` 自己发布。
- `pelvis -> body` 只是兼容机器人端非官方 `frame_id=body` 的数据，不建议作为新传感器安装父帧。

## 推荐启动

机器人端 RealSense 使用 `rs_launch.py` 默认命名时：

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

本机 Foxglove：

```bash
ENABLE_G1_BODY_FRAME_ALIAS=true \
ENABLE_REALSENSE_STATIC_TF=true \
REALSENSE_PARENT_FRAME=d435_link \
REALSENSE_BASE_FRAME=camera_link \
./foxglove/run_foxglove_bridge_g1.sh
```

现场 G1 实际 topic list 是 `/camera/*`，本机脚本默认也按这个前缀桥接：

```bash
REALSENSE_TOPIC_PREFIX=/camera
```

如果机器人端被改成 `/camera/camera/*` 或 `/camera/d435/*`，才需要同步调整 `REALSENSE_TOPIC_PREFIX` 和 `g1_foxglove.json`。

## 排查命令

先确认 topic：

```bash
ros2 topic list | rg 'camera|utlidar|joint_states|tf'
```

再确认 TF：

```bash
ros2 run tf2_ros tf2_echo pelvis body
ros2 run tf2_ros tf2_echo d435_link camera_link
ros2 run tf2_ros tf2_echo d435_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo mid360_link livox_frame
```

生成整棵树：

```bash
ros2 run tf2_tools view_frames
```

Foxglove 报错：

```text
Missing transform from frame <A> to frame <B>
```

对应检查：

```bash
ros2 run tf2_ros tf2_echo B A
```

## 已知边界

- 当前 `g1_foxglove.json` 默认使用 `/camera/*`；不会自动跟随 `REALSENSE_TOPIC_PREFIX` 改 layout。
- `ENABLE_G1_BODY_FRAME_ALIAS=true` 是为现场兼容 `body` frame 数据；如果机器人端已经发布 body 相关 TF，应关闭避免重复。
- `ENABLE_REALSENSE_STATIC_TF=true` 是本机临时补 `d435_link -> camera_link`；如果机器人端已经提供这条 TF，应关闭避免重复。
- `g1_foxglove_layout.template.json` 是旧的单 3D Scene 模板，不是当前完整工作区布局。
