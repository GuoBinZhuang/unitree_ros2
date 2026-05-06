# G1 Foxglove 中文使用说明

本文是 `unitree_ros2/foxglove` 的现场使用入口，说明如何用 Foxglove 查看 G1 机器人模型、关节状态、RealSense、Mid-360、Inspire Hand 触觉图和 Teleimager 图像服务。

更细的专题文档：

- `tf_g1_realsense.md`：G1 + RealSense + Foxglove TF 排查。
- `teleimager_foxglove.md`：`xr_teleoperate` / `teleimager` 图像服务接入 Foxglove。
- `g1_foxglove_audit.md`：当前 layout、topic、TF 和官方模型资产审计记录。

## 目录内容

当前目录不是 ROS2 package，主要用于存放 Foxglove 布局、G1 模型资产和启动脚本：

```text
foxglove/
├── g1_foxglove.json              # 当前推荐 Foxglove 工作区布局
├── run_foxglove_bridge_g1.sh     # 本机 Foxglove bridge + G1 可视化辅助启动脚本
├── teleimager_to_ros_image.py    # Teleimager ZMQ 图像转 ROS2 Image/CompressedImage
├── g1_description/               # G1 URDF/MJCF/mesh 静态资产
├── my-panel/                     # 自定义 Foxglove panel 扩展
├── tf_g1_realsense.md
├── teleimager_foxglove.md
└── g1_foxglove_audit.md
```

日常优先使用：

```text
g1_foxglove.json
run_foxglove_bridge_g1.sh
```

不要优先修改 `g1_foxglove copy*.json` 或旧模板类文件；这些是现场副本或历史布局。

## 当前布局显示内容

`g1_foxglove.json` 当前默认包含：

| 面板/图层 | Topic 或资源 | 用途 |
|---|---|---|
| Teleimager Head | `/teleimager/head/compressed` | PC2 图像服务头部相机图像 |
| D435i RGB | `/camera/color/image_raw` | RealSense 彩色图 |
| D435i Depth | `/camera/depth/image_rect_raw` | RealSense 深度图 |
| D435i PointCloud | `/camera/depth/color/points` | RealSense 点云 |
| Inspire Hand 触觉图 | `/inspire_hand/touch/right/image` | 右手触觉热力图 |
| Mid-360 点云 | `/utlidar/cloud_livox_mid360` | 激光雷达点云 |
| G1 URDF | `g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf` | 机器人 3D 模型 |
| JointState | `/joint_states` | 驱动 G1 URDF 关节运动 |

RGB、Depth 和 Teleimager 图像都作为普通 Image 面板显示。不要把相机图像重新配置为 3D 面板投影层，否则容易引入额外 TF 依赖并触发 `Missing transform`。

## 基本使用流程

### 1. 启动机器人端数据源

机器人端通常需要已经发布：

```text
/lowstate
/utlidar/cloud_livox_mid360
/camera/color/image_raw
/camera/depth/image_rect_raw
/camera/depth/color/points
/tf
/tf_static
```

如果使用 RealSense，机器人端推荐启动参数：

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

如果使用 `xr_teleoperate` / `teleimager` 图像服务，PC2 上需要先启动：

```bash
teleimager-server
```

本地主机可先用浏览器确认：

```text
https://192.168.123.164:60001
```

点击 Start 后能看到图像，再接入 Foxglove。

### 2. 启动本机 Foxglove bridge

从 `unitree_ros2` 根目录运行：

```bash
ENABLE_G1_BODY_FRAME_ALIAS=true \
ENABLE_REALSENSE_STATIC_TF=true \
REALSENSE_PARENT_FRAME=d435_link \
REALSENSE_BASE_FRAME=camera_link \
./foxglove/run_foxglove_bridge_g1.sh
```

脚本会启动：

- `g1_lowstate_to_joint_states`：把 `/lowstate` 转为 `/joint_states`。
- `robot_state_publisher`：发布 G1 URDF 内部 TF。
- 可选 Inspire Hand 关节和触觉桥接。
- 可选 Teleimager 图像桥接。
- 可选 RealSense / Mid-360 / body 兼容静态 TF。
- `foxglove_bridge`：默认端口 `8765`。

Foxglove 连接地址：

```text
ws://localhost:8765
```

如果在 Windows Foxglove Desktop 里连接 WSL，可尝试：

```text
ws://127.0.0.1:8765
```

### 3. 导入 Foxglove 布局

打开 Foxglove Desktop 后：

1. 连接 `ws://localhost:8765`。
2. 导入布局文件 `foxglove/g1_foxglove.json`。
3. 如果 3D 面板里的 URDF 路径失效，重新指向本机可访问的：

```text
/home/guobing/My_Repositories/unitree_ros2/foxglove/g1_description/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf
```

在 Windows Foxglove Desktop 中，URDF 文件路径可能显示为 WSL UNC 路径：

```text
\\wsl.localhost\Ubuntu-24.04\home\guobing\My_Repositories\unitree_ros2\foxglove\g1_description\g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf
```

## 常用启动组合

### 只看 G1 + Mid-360 + RealSense

```bash
ENABLE_G1_BODY_FRAME_ALIAS=true \
ENABLE_REALSENSE_STATIC_TF=true \
REALSENSE_PARENT_FRAME=d435_link \
REALSENSE_BASE_FRAME=camera_link \
./foxglove/run_foxglove_bridge_g1.sh
```

### 同时接入 Teleimager 图像

```bash
ENABLE_TELEIMAGER_IMAGE_BRIDGE=true \
TELEIMAGER_HOST=192.168.123.164 \
./foxglove/run_foxglove_bridge_g1.sh
```

Teleimager 桥接默认使用双 Python：

```bash
TELEIMAGER_PYTHON=/usr/bin/python3
TELEIMAGER_CLIENT_PYTHON=/home/guobing/anaconda3/bin/python
```

其中 `/usr/bin/python3` 用于 ROS2 `rclpy`，Anaconda Python 只作为 `pyzmq` 图像客户端 helper。不要用 Anaconda Python 3.13 加载 ROS2 Jazzy 的 `rclpy`。

### 只接 Teleimager 头部相机

```bash
ENABLE_TELEIMAGER_IMAGE_BRIDGE=true \
TELEIMAGER_CAMERAS=head \
./foxglove/run_foxglove_bridge_g1.sh
```

### RealSense topic prefix 不是 `/camera`

如果机器人端实际 topic 是 `/camera/camera/...`：

```bash
REALSENSE_TOPIC_PREFIX=/camera/camera \
./foxglove/run_foxglove_bridge_g1.sh
```

同时需要把 `g1_foxglove.json` 中的：

```text
/camera/color/...
/camera/depth/...
```

同步改成：

```text
/camera/camera/color/...
/camera/camera/depth/...
```

如果机器人端实际 topic 是 `/camera/d435/...`，同理设置：

```bash
REALSENSE_TOPIC_PREFIX=/camera/d435
```

并同步修改 layout。

## 关键环境变量

| 变量 | 默认值 | 说明 |
|---|---|---|
| `FOXGLOVE_BRIDGE_PORT` | `8765` | Foxglove bridge WebSocket 端口 |
| `LOWSTATE_TOPIC` | `/lowstate` | G1 lowstate 输入 |
| `JOINT_STATE_TOPIC` | `/joint_states` | G1 JointState 输出 |
| `G1_URDF_PATH` | `foxglove/g1_description/...FTP.urdf` | Foxglove/robot_state_publisher 使用的 G1 URDF |
| `ENABLE_G1_BODY_FRAME_ALIAS` | `true` | 是否发布 `pelvis -> body` 兼容 TF |
| `ENABLE_REALSENSE_D435I_BRIDGE` | `true` | 是否桥接 RealSense topic 白名单 |
| `REALSENSE_TOPIC_PREFIX` | `/camera` | RealSense topic 前缀 |
| `ENABLE_REALSENSE_STATIC_TF` | `false` | 是否发布 `d435_link -> camera_link` |
| `REALSENSE_PARENT_FRAME` | `d435_link` | RealSense 静态 TF 父帧 |
| `REALSENSE_BASE_FRAME` | `camera_link` | RealSense 静态 TF 子帧 |
| `LIDAR_FRAME_ID` | `livox_frame` | Mid-360 点云消息 frame |
| `ENABLE_INSPIRE_HAND_BRIDGE` | `true` | 是否启动 Inspire Hand JointState 桥接 |
| `ENABLE_INSPIRE_TOUCH_BRIDGE` | `true` | 是否启动 Inspire Hand 触觉可视化 |
| `ENABLE_TELEIMAGER_IMAGE_BRIDGE` | `false` | 是否启动 Teleimager 图像桥接 |
| `TELEIMAGER_HOST` | `192.168.123.164` | PC2 Teleimager 服务 IP |
| `TELEIMAGER_TOPIC_PREFIX` | `/teleimager` | Teleimager 输出 topic 前缀 |
| `TELEIMAGER_CAMERAS` | `head,left_wrist,right_wrist` | Teleimager 相机选择 |
| `TELEIMAGER_PUBLISH_RAW` | `false` | 是否额外发布 raw `sensor_msgs/Image` |
| `TELEIMAGER_PYTHON` | `/usr/bin/python3` | ROS2 publisher Python |
| `TELEIMAGER_CLIENT_PYTHON` | `/home/guobing/anaconda3/bin/python` | ZMQ helper Python |

## Topic 检查

启动脚本后，先确认 topic 是否存在：

```bash
ros2 topic list | rg 'camera|teleimager|utlidar|joint_states|tf|inspire'
```

常见应看到：

```text
/joint_states
/tf
/tf_static
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_rect_raw
/camera/depth/camera_info
/camera/depth/color/points
/utlidar/cloud_livox_mid360
/inspire_hand/touch/right/image
/teleimager/head/compressed
```

检查单个 topic 类型：

```bash
ros2 topic info /teleimager/head/compressed
ros2 topic info /camera/color/image_raw
ros2 topic info /utlidar/cloud_livox_mid360
```

检查是否有帧：

```bash
ros2 topic echo /teleimager/head/compressed --once
ros2 topic hz /camera/color/image_raw
ros2 topic hz /utlidar/cloud_livox_mid360
```

## TF 检查

官方 G1 模型根帧是：

```text
pelvis
```

推荐目标树：

```text
pelvis
└── waist_yaw_link -> waist_roll_link -> torso_link
    ├── mid360_link -> livox_frame
    └── d435_link -> camera_link
        ├── camera_color_frame -> camera_color_optical_frame
        └── camera_depth_frame -> camera_depth_optical_frame
```

常用检查：

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

就用反向检查：

```bash
ros2 run tf2_ros tf2_echo B A
```

如果这条链不存在，Foxglove 就无法把消息投到目标坐标系。

## Foxglove 面板使用要点

### Image 面板

用于显示：

```text
/teleimager/head/compressed
/camera/color/image_raw
/camera/depth/image_rect_raw
/inspire_hand/touch/right/image
```

Depth 图像可以使用 gradient / colormap 模式。触觉图也是普通 `sensor_msgs/Image`，适合用 color/gradient 显示。

### 3D 面板

用于显示：

```text
G1 URDF
/joint_states
/utlidar/cloud_livox_mid360
/camera/depth/color/points
TF
Grid
```

不要在 3D 面板里重新加入相机图像投影字段：

```text
projectionFrameId
frameLocked
cameraInfoTopic
```

这些字段曾导致 `camera_color_optical_frame -> pelvis` 缺 TF 报错。当前做法是图像用 Image 面板，点云和 URDF 通过真实 TF 树对齐。

### 自定义 G1 热力图 Panel

`my-panel/` 下是 Foxglove 扩展工程，当前注册了 G1 关节热力图面板。开发时：

```bash
cd foxglove/my-panel
npm install
npm run build
npm run local-install
```

安装后刷新 Foxglove Desktop，可在面板列表中找到自定义面板。

## 常见问题

### Foxglove 连接不上

先确认 bridge 是否在跑：

```bash
ros2 node list | rg foxglove
```

确认端口：

```bash
ss -ltnp | rg 8765
```

然后在 Foxglove 里连接：

```text
ws://localhost:8765
```

### Foxglove 看不到某个 topic

先确认 ROS2 里是否存在：

```bash
ros2 topic list | rg '<关键字>'
```

如果 ROS2 存在但 Foxglove 不显示，检查 `run_foxglove_bridge_g1.sh` 里的 `TOPIC_PATTERNS` 白名单。改 topic 名时必须同步改三处：

```text
run_foxglove_bridge_g1.sh
g1_foxglove.json
相关说明文档
```

### `rclpy._rclpy_pybind11` 报错

如果 traceback 里出现：

```text
/home/guobing/anaconda3/lib/python3.13
No module named 'rclpy._rclpy_pybind11'
```

说明误用了 Anaconda Python 3.13 加载 ROS2 Jazzy 的 `rclpy`。设置：

```bash
TELEIMAGER_PYTHON=/usr/bin/python3
```

Teleimager 的 ZMQ helper 可以继续用：

```bash
TELEIMAGER_CLIENT_PYTHON=/home/guobing/anaconda3/bin/python
```

### `No module named 'zmq'`

这是 ZMQ helper Python 缺 `pyzmq`。确认：

```bash
/home/guobing/anaconda3/bin/python -c 'import zmq; print("ok")'
```

如果你使用其他 Python，设置：

```bash
TELEIMAGER_CLIENT_PYTHON=/path/to/python
```

### Teleimager topic 出现但没有图像

先确认 PC2 图像服务：

```text
https://192.168.123.164:60001
```

再确认 topic 有数据：

```bash
ros2 topic hz /teleimager/head/compressed
ros2 topic echo /teleimager/head/compressed --once
```

如果 topic 没有数据，检查 PC2 的 `cam_config_server.yaml` 是否开启对应相机的 `enable_zmq`，并确认本地主机能访问 `60000` 和相机 `zmq_port`。

### 3D 模型不动

确认 `/joint_states` 有数据：

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
```

确认 `robot_state_publisher` 已启动：

```bash
ros2 node list | rg robot_state_publisher
```

如果 `/lowstate` 没有输入，`g1_lowstate_to_joint_states` 就无法驱动 URDF。

### 点云位置不对或上下翻转

先确认消息 frame：

```bash
ros2 topic echo /utlidar/cloud_livox_mid360 --once | rg frame_id
```

当前脚本默认把 `mid360_link -> livox_frame` 补一个 `roll=pi`，用于兼容现场点云 frame。如果现场驱动已经改为 `mid360_link` 或姿态已修正，可能需要调整：

```bash
LIDAR_FRAME_ID=mid360_link
```

或修改：

```bash
LIDAR_TF_ROLL
LIDAR_TF_PITCH
LIDAR_TF_YAW
```

### 重复 TF 警告

如果机器人端已经发布了同一条静态 TF，本机脚本不应重复发布。

常见关闭项：

```bash
ENABLE_G1_BODY_FRAME_ALIAS=false
ENABLE_REALSENSE_STATIC_TF=false
```

原则：

- `robot_state_publisher` 负责 G1 URDF 内部 TF。
- RealSense wrapper 负责相机内部 TF。
- 本机静态 TF 只补跨系统连接。

## 修改布局或 topic 的规则

改 topic 时不要只改一个地方。至少同步检查：

1. `g1_foxglove.json`：Foxglove 面板订阅的 topic。
2. `run_foxglove_bridge_g1.sh`：`TOPIC_PATTERNS` 白名单。
3. 说明文档：`foxglove_usage_zh.md`、`AGENTS.md`、专题文档。

如果改了 RealSense prefix：

```text
REALSENSE_TOPIC_PREFIX
g1_foxglove.json 中 /camera/...
```

如果改了 Teleimager prefix：

```text
TELEIMAGER_TOPIC_PREFIX
g1_foxglove.json 中 /teleimager/...
```

## 推荐现场检查清单

启动前：

```bash
source /opt/ros/jazzy/setup.zsh
ros2 topic list | rg 'lowstate|camera|utlidar|tf'
/usr/bin/python3 -c 'import rclpy, sensor_msgs, cv2, numpy; print("ros python ok")'
/home/guobing/anaconda3/bin/python -c 'import zmq; print("zmq python ok")'
```

启动后：

```bash
ros2 topic list | rg 'joint_states|camera|teleimager|utlidar|tf'
ros2 topic hz /joint_states
ros2 run tf2_ros tf2_echo d435_link camera_link
ros2 run tf2_ros tf2_echo mid360_link livox_frame
```

Foxglove 内：

```text
连接 ws://localhost:8765
导入 g1_foxglove.json
确认 Teleimager / RGB / Depth / Touch 图像面板有数据
确认 3D 面板中 G1 URDF、Mid-360 点云、D435i 点云显示正常
```

