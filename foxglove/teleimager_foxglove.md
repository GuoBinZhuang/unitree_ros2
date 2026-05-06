# Teleimager 图像接入 Foxglove

本文记录把 `xr_teleoperate` / `teleimager` 图像服务接入当前 G1 Foxglove 工作区的方式。

## 数据路径

推荐路径：

```text
PC2 teleimager image_server
  -> 本地主机 ZMQ helper
  -> 本地主机 ROS2 publisher
  -> ROS2 sensor_msgs/CompressedImage
  -> foxglove_bridge
  -> Foxglove Image panel
```

`teleimager_to_ros_image.py` 内部会启动两个 Python 进程：

```text
/usr/bin/python3
  运行 ROS2 publisher，负责 rclpy / sensor_msgs / cv2 / numpy

/home/guobing/anaconda3/bin/python
  运行 ZMQ helper，只负责从 PC2 teleimager 订阅 JPEG 帧
```

这样做是为了避开 ROS2 Jazzy 的 `rclpy` ABI 限制：Jazzy 的 `rclpy` 绑定系统 Python 3.12，不能用 Anaconda Python 3.13 加载。

当前默认 topic：

```text
/teleimager/head/compressed
/teleimager/left_wrist/compressed
/teleimager/right_wrist/compressed
```

如果设置 `TELEIMAGER_PUBLISH_RAW=true`，还会额外发布：

```text
/teleimager/head/image
/teleimager/left_wrist/image
/teleimager/right_wrist/image
```

Foxglove 主布局 `g1_foxglove.json` 默认显示 `/teleimager/head/compressed`。

## 前置条件

PC2 上按 `xr_teleoperate` 文档启动 teleimager：

```bash
teleimager-server
```

本地主机先确认浏览器能打开：

```text
https://192.168.123.164:60001
```

并且点击 Start 后能看到图像。

本地主机运行 ROS2 publisher 的 Python 必须能导入：

```text
rclpy
sensor_msgs
numpy
cv2
```

ZMQ 客户端 helper 使用另一个 Python，只需要能导入：

```text
zmq
```

注意：`xr_teleoperate` 文档里的 `tv` conda 环境通常是 Python 3.10，当前机器的 base Anaconda 可能是 Python 3.13，而 ROS2 Jazzy 的 `rclpy` 绑定系统 Python 3.12。不要用 Anaconda 的 `python3` 运行 ROS2 publisher；默认 `TELEIMAGER_PYTHON=/usr/bin/python3`。ZMQ helper 可以继续用 Anaconda Python，默认 `TELEIMAGER_CLIENT_PYTHON=/home/guobing/anaconda3/bin/python`。

启动前可快速检查：

```bash
source /opt/ros/jazzy/setup.zsh
/usr/bin/python3 -c 'import rclpy, sensor_msgs, cv2, numpy; print("ros publisher python ok")'
/home/guobing/anaconda3/bin/python -c 'import zmq; print("teleimager helper python ok")'
```

## 启动

从 `unitree_ros2` 根目录启动：

```bash
ENABLE_TELEIMAGER_IMAGE_BRIDGE=true \
TELEIMAGER_HOST=192.168.123.164 \
./foxglove/run_foxglove_bridge_g1.sh
```

常用可选项：

```bash
TELEIMAGER_TOPIC_PREFIX=/teleimager
TELEIMAGER_CAMERAS=head,left_wrist,right_wrist
TELEIMAGER_REQUEST_PORT=60000
TELEIMAGER_RATE=30
TELEIMAGER_PUBLISH_RAW=false
TELEIMAGER_PYTHON=/usr/bin/python3
TELEIMAGER_CLIENT_PYTHON=/home/guobing/anaconda3/bin/python
```

如果只想看头部图像：

```bash
ENABLE_TELEIMAGER_IMAGE_BRIDGE=true \
TELEIMAGER_CAMERAS=head \
./foxglove/run_foxglove_bridge_g1.sh
```

## 单独排查桥接脚本

先 source ROS 环境：

```bash
source /opt/ros/jazzy/setup.zsh
```

再单独启动：

```bash
/usr/bin/python3 ./foxglove/teleimager_to_ros_image.py \
  --host 192.168.123.164 \
  --cameras head \
  --topic-prefix /teleimager \
  --client-python /home/guobing/anaconda3/bin/python
```

确认 topic：

```bash
ros2 topic list | rg teleimager
ros2 topic echo /teleimager/head/compressed --once
```

## 排查

- 如果脚本启动时报 `Teleimager ZMQ 客户端 Python 环境不可用` 或 `No module named 'zmq'`：当前 `TELEIMAGER_CLIENT_PYTHON` 指向的 Python 没有安装 pyzmq。
- 如果脚本启动时报 `No module named 'rclpy'`：当前 Python 不是 ROS2 Jazzy 的 Python 环境，先 source ROS setup，或换成能导入 `rclpy` 的解释器。
- 如果脚本启动时报 `No module named 'rclpy._rclpy_pybind11'`，并且 traceback 里出现 `/home/guobing/anaconda3/lib/python3.13`：说明误用了 Anaconda Python 3.13。设置 `TELEIMAGER_PYTHON=/usr/bin/python3`。
- 如果报 `No selected teleimager ZMQ cameras are enabled`：PC2 的 `cam_config_server.yaml` 里对应相机没有开启 `enable_zmq`。
- 如果启动后没有帧但服务不退出：先用浏览器打开 `https://192.168.123.164:60001` 并点击 Start，确认 PC2 图像服务本身正常；再确认本地主机能访问 PC2 的 `60000` 配置端口和各相机 `zmq_port`。
- 如果 Foxglove 里 topic 不出现：确认 `ENABLE_TELEIMAGER_IMAGE_BRIDGE=true`，并确认 `run_foxglove_bridge_g1.sh` 的 topic 白名单已包含 `/teleimager/*`。
