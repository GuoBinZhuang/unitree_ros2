# foxglove 知识库

## 概览
- 这里不是 ROS2 package；主要放 G1 可视化资产、Foxglove layout、桥接脚本和现场排查文档。
- 当前 G1 可视化主入口是 `g1_foxglove.json` + `run_foxglove_bridge_g1.sh`，不是旧的 `g1_foxglove_layout.template.json`。
- 官方参照源是 `/home/guobing/My_Repositories/unitree_ros/robots/g1_description`；当前 `g1_description/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf` 已确认与官方同名文件一致。

## 结构
```text
foxglove/
├── g1_description/                    # G1 官方 URDF/MJCF/mesh 静态资产；不参与 colcon build
├── foxglove_usage_zh.md               # G1 Foxglove 中文使用总说明
├── g1_foxglove.json                   # 当前推荐 Foxglove 工作区布局：D435i RGB/Depth/PointCloud + G1 + Mid-360 + 右手触觉
├── g1_foxglove_audit.md               # 官方资源对照、布局/topic/TF 审计结论
├── teleimager_foxglove.md             # xr_teleoperate/teleimager 图像服务接入 Foxglove 说明
├── tf_g1_realsense.md                 # G1 + RealSense + Foxglove TF 说明
└── run_foxglove_bridge_g1.sh          # 本机 Foxglove bridge + G1 JointState/TF 辅助启动脚本
```

## 优先查看
| 任务 | 位置 | 备注 |
|---|---|---|
| 中文使用总说明 | `foxglove_usage_zh.md` | 现场启动、布局导入、topic/TF/常见错误总入口 |
| 现场启动/改白名单 | `run_foxglove_bridge_g1.sh` | 改 topic 时同步检查 layout 和 `TOPIC_PATTERNS` |
| 当前 Foxglove 布局 | `g1_foxglove.json` | 默认使用 `/camera/*`、`/utlidar/cloud_livox_mid360`、`/joint_states` |
| Teleimager 图像服务 | `teleimager_foxglove.md` | 从 PC2 WebRTC/ZMQ 图像服务转 ROS2 Image/CompressedImage |
| TF/RealSense 排查 | `tf_g1_realsense.md` | 先看推荐 TF 树，再查 `Missing transform` |
| 官方资源对照结论 | `g1_foxglove_audit.md` | 记录当前配置相对 `unitree_ros/robots/g1_description` 的检查结果 |
| 选 G1 模型变体 | `g1_description/README.md` | 先看 `mode_machine`、手部配置、腰部锁定状态 |

## 当前布局约定
- `g1_foxglove.json` 当前包含：
  - D435i RGB：`/camera/color/image_raw`
  - D435i Depth：`/camera/depth/image_rect_raw`
  - D435i PointCloud：`/camera/depth/color/points`
  - Teleimager 头部图像：`/teleimager/head/compressed`
  - Inspire Hand 右手触觉图：`/inspire_hand/touch/right/image`
  - G1 URDF：`g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf`
  - Mid-360 点云：`/utlidar/cloud_livox_mid360`
- 不要在 3D 面板里重新加入 `projectionFrameId`、`frameLocked`、`cameraInfoTopic` 来投影相机图像；这些字段曾导致 `camera_color_optical_frame -> pelvis` 缺 TF 报错。
- RGB/Depth 应作为普通 Image 面板显示；点云和 URDF 通过真实 TF 树对齐。
- 当前按现场 G1 实际 topic list 消费 `/camera/*`。如果机器人端 RealSense topic 改成 `/camera/camera/*` 或 `/camera/d435/*`，必须同时改 `g1_foxglove.json` 的 topic 和脚本里的 `REALSENSE_TOPIC_PREFIX`。
- `run_foxglove_bridge_g1.sh` 默认只桥接 RealSense 标准 ROS 消息话题：RGB/Depth image、camera_info、aligned depth、点云和 `/camera/imu`。不要默认桥接 `metadata`、`imu_info`、`extrinsics`，这些依赖 `realsense2_camera_msgs`，本机未安装该包时会让 `foxglove_bridge` 反复报 schemaDefinition 错误。
- Teleimager 图像桥接默认使用双 Python：`TELEIMAGER_PYTHON=/usr/bin/python3` 运行 ROS2 publisher，`TELEIMAGER_CLIENT_PYTHON=/home/guobing/anaconda3/bin/python` 运行 ZMQ helper。不要用 Anaconda Python 3.13 加载 ROS2 Jazzy 的 `rclpy`。

## TF 约定
- 官方 G1 模型根帧是 `pelvis`，不是 `body`、`base_link` 或 `trunk`。
- 官方 G1 URDF 已包含：
  - `torso_link -> d435_link`
  - `torso_link -> mid360_link`
- D435i 安装姿态保持本地官方 URDF 原值：`torso_link -> d435_link` 使用 `xyz="0.0576235 0.01753 0.42987"`、`rpy="0 0.8307767239493009 0"`，pitch 约 `47.6°`，roll/yaw 为 0。宇树服务控制台 2026-01-23 说明中的 Z 值为 `0.41987`，差 0.01 m，当前不修改官方模型资产。
- 推荐 TF 树：
```text
pelvis
└── waist_yaw_link -> waist_roll_link -> torso_link
    ├── mid360_link -> livox_frame
    └── d435_link -> camera_link
        ├── camera_color_frame -> camera_color_optical_frame
        └── camera_depth_frame -> camera_depth_optical_frame
```
- `robot_state_publisher` 负责 G1 URDF 内部 TF；RealSense wrapper 负责 `camera_link -> camera_*_optical_frame`；静态 TF 只负责跨系统连接，例如 `d435_link -> camera_link`。
- `body` 只作为兼容别名使用。脚本默认发布 `pelvis -> body`，用于兼容机器人端 `frame_id=body` 的数据；如果机器人端已经发布 body 相关 TF，设置 `ENABLE_G1_BODY_FRAME_ALIAS=false` 避免重复。
- D435i 不应长期挂到 `pelvis` 或 `body`；应挂到 `d435_link`。Mid-360 不应挂到 `pelvis` 或 `body`；应挂到 `mid360_link`。

## 启动约定
本机 Foxglove 推荐启动：
```bash
ENABLE_G1_BODY_FRAME_ALIAS=true \
ENABLE_REALSENSE_STATIC_TF=true \
REALSENSE_PARENT_FRAME=d435_link \
REALSENSE_BASE_FRAME=camera_link \
./foxglove/run_foxglove_bridge_g1.sh
```

机器人端 RealSense 推荐启动：
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

关键环境变量：
```bash
REALSENSE_TOPIC_PREFIX=/camera
ENABLE_REALSENSE_STATIC_TF=false|true
REALSENSE_PARENT_FRAME=d435_link
REALSENSE_BASE_FRAME=camera_link
ENABLE_G1_BODY_FRAME_ALIAS=true|false
LIDAR_FRAME_ID=livox_frame
ENABLE_TELEIMAGER_IMAGE_BRIDGE=false|true
TELEIMAGER_HOST=192.168.123.164
TELEIMAGER_TOPIC_PREFIX=/teleimager
TELEIMAGER_CAMERAS=head,left_wrist,right_wrist
TELEIMAGER_PUBLISH_RAW=false|true
TELEIMAGER_PYTHON=/usr/bin/python3
TELEIMAGER_CLIENT_PYTHON=/home/guobing/anaconda3/bin/python
```

## 排查命令
确认 topic：
```bash
ros2 topic list | rg 'camera|teleimager|utlidar|joint_states|tf'
```

确认 TF：
```bash
ros2 run tf2_ros tf2_echo pelvis body
ros2 run tf2_ros tf2_echo d435_link camera_link
ros2 run tf2_ros tf2_echo d435_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo mid360_link livox_frame
ros2 run tf2_tools view_frames
```

Foxglove 报：
```text
Missing transform from frame <A> to frame <B>
```

用这个方向查：
```bash
ros2 run tf2_ros tf2_echo B A
```

## 本目录约定
- `g1_description/` 里是静态资产；大量 `.STL`、`.urdf`、`.xml` 不是源码生成物。
- `run_foxglove_bridge_g1.sh` 是 zsh 脚本，依赖 Jazzy 环境与当前机器硬编码 `ROOT_DIR`。
- 脚本会启动 `g1_lowstate_to_joint_states`、`robot_state_publisher`、可选 Inspire Hand/touch 桥接、可选 Teleimager 图像桥接、可选静态 TF，以及 `foxglove_bridge`。
- 改 topic 名时必须同步改 layout、bridge 白名单和文档；不要只改其中一个。
- `g1_foxglove copy*.json` 是现场副本/历史布局；主配置优先改 `g1_foxglove.json`。

## 本目录反模式
- 不要把这里当成示例源码树；业务控制逻辑不在 `foxglove/`。
- 不要优先改 `g1_foxglove_layout.template.json` 来影响当前工作区；它只是旧模板。
- 不要继续优先使用 README 里标成 Deprecated 的 `g1_23dof`、`g1_29dof`、`g1_29dof_with_hand`、`g1_29dof_lock_waist` 老模型。
- 不要让两个节点发布同一条 TF；发现重复 TF 时优先关闭脚本里的兼容静态 TF。
- 不要把传感器安装 TF 写成 `pelvis -> camera_*_optical_frame`；光学坐标转换应由相机模型/RealSense wrapper 提供。
