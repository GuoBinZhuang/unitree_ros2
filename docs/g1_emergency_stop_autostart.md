# G1 急停程序开机自启与后台运行（systemd）

本文档说明如何把 `g1_emergency_stop_node` 托管为 systemd 服务，实现以下目标：

1. 开机自动启动。
2. 后台常驻运行。
3. 异常退出自动拉起。
4. 统一通过 `journalctl` 查看日志。

## 1. 适用范围

本文适用于本仓库当前提供的脚本：

1. `scripts/install_g1_emergency_stop_service.sh`
2. `scripts/run_g1_emergency_stop_node.sh`
3. `scripts/uninstall_g1_emergency_stop_service.sh`

支持的 ROS2 发行版自动检测顺序为：`jazzy` -> `humble` -> `foxy`。

## 2. 运行原理

服务启动后，systemd 会执行：

```bash
/usr/bin/env bash scripts/run_g1_emergency_stop_node.sh
```

运行脚本会按顺序加载环境：

1. `/opt/ros/<distro>/setup.bash`（可由 `ROS_SETUP_PATH` 覆盖）
2. 仓库根目录 `setup.sh`
3. 仓库根目录 `install/setup.sh`（如果存在）
4. `example/install/setup.sh`（如果存在）

最终执行：

```bash
ros2 run unitree_ros2_example g1_emergency_stop_node
```

## 3. 前置条件

1. 已完成编译并产出可执行文件。
2. 系统使用 systemd（Ubuntu 默认支持）。
3. 当前用户具备 sudo 权限。
4. 机器人网络配置与 DDS 配置有效。

建议先在仓库根目录执行：

```bash
colcon build --packages-select unitree_ros2_example --allow-overriding unitree_hg
```

若你之前手动运行过节点，也建议先确认能正常启动：

```bash
source /opt/ros/jazzy/setup.bash
source ./setup.sh
source ./install/setup.sh
ros2 run unitree_ros2_example g1_emergency_stop_node
```

## 4. 快速安装（推荐）

在仓库根目录执行：

```bash
./scripts/install_g1_emergency_stop_service.sh \
  --repo-dir /home/unitree/gb_ros_ws_g1/unitree_ros2 \
  --user unitree \
  --ros-distro jazzy \
  --service-name g1-emergency-stop.service
```

脚本会自动执行：

1. 生成 `/etc/systemd/system/g1-emergency-stop.service`。
2. `systemctl daemon-reload`。
3. `systemctl enable --now g1-emergency-stop.service`。

## 5. 安装脚本参数说明

安装脚本支持如下参数：

1. `--repo-dir <path>`：仓库根目录。
2. `--service-name <name>`：服务名，默认 `g1-emergency-stop.service`。
3. `--user <name>`：服务运行用户，默认当前用户。
4. `--ros-distro <name>`：显式指定 ROS2 发行版。
5. `-h` 或 `--help`：查看帮助。

示例：

```bash
./scripts/install_g1_emergency_stop_service.sh --help
```

## 6. 验证后台运行与自启

### 6.1 查看服务状态

```bash
sudo systemctl status g1-emergency-stop.service --no-pager
```

状态为 `Active: active (running)` 即表示正在后台运行。

### 6.2 查看日志

实时跟踪：

```bash
sudo journalctl -u g1-emergency-stop.service -f
```

查看最近 100 行：

```bash
sudo journalctl -u g1-emergency-stop.service -n 100 --no-pager
```

### 6.3 查看开机自启

```bash
systemctl is-enabled g1-emergency-stop.service
```

输出 `enabled` 表示开机自启已启用。

## 7. 日常运维命令

重启服务：

```bash
sudo systemctl restart g1-emergency-stop.service
```

停止服务：

```bash
sudo systemctl stop g1-emergency-stop.service
```

启动服务：

```bash
sudo systemctl start g1-emergency-stop.service
```

查看是否正在运行：

```bash
systemctl is-active g1-emergency-stop.service
```

## 8. 配置变更与升级流程

当你修改了以下任一内容时：

1. `scripts/run_g1_emergency_stop_node.sh`
2. 安装脚本生成的 service 配置
3. ROS2 安装路径或发行版
4. 仓库目录位置

建议执行以下流程：

```bash
sudo systemctl stop g1-emergency-stop.service
./scripts/install_g1_emergency_stop_service.sh \
  --repo-dir /home/unitree/gb_ros_ws_g1/unitree_ros2 \
  --user unitree \
  --ros-distro jazzy \
  --service-name g1-emergency-stop.service
sudo systemctl status g1-emergency-stop.service --no-pager
```

## 9. 常见问题与排查

### 9.1 提示命令不存在（`No such file or directory`）

现象示例：

```text
/scripts/install_g1_emergency_stop_service.sh: No such file or directory
```

原因：路径写成了 `/scripts/...`（绝对路径），而不是仓库内相对路径。

处理：

```bash
cd /home/unitree/gb_ros_ws_g1/unitree_ros2
./scripts/install_g1_emergency_stop_service.sh
```

### 9.2 服务反复重启，日志出现 `AMENT_TRACE_SETUP_FILES` 未绑定变量

原因：旧版运行脚本在严格 `set -u` 下直接 `source` ROS setup，触发未定义变量错误。

处理：

1. 确保仓库中 `scripts/run_g1_emergency_stop_node.sh` 已更新为最新版本。
2. 重新安装服务（覆盖 service 配置）。

```bash
./scripts/install_g1_emergency_stop_service.sh \
  --repo-dir /home/unitree/gb_ros_ws_g1/unitree_ros2 \
  --user unitree \
  --ros-distro jazzy
```

### 9.3 服务启动后很快退出

先看日志：

```bash
sudo journalctl -u g1-emergency-stop.service -n 200 --no-pager
```

重点关注：

1. ROS2 环境是否正确加载。
2. `g1_emergency_stop_node` 可执行文件是否存在。
3. GPIO 资源是否可访问（如 line 被占用）。
4. 网络与 DDS 接口是否与机器人环境匹配。

### 9.4 运行用户权限问题

如果你希望改为其他用户运行（例如 `root` 或专用服务账号），重新安装并指定：

```bash
./scripts/install_g1_emergency_stop_service.sh \
  --repo-dir /home/unitree/gb_ros_ws_g1/unitree_ros2 \
  --user <new_user> \
  --ros-distro jazzy
```

## 10. 卸载服务

默认卸载：

```bash
./scripts/uninstall_g1_emergency_stop_service.sh
```

指定自定义服务名：

```bash
./scripts/uninstall_g1_emergency_stop_service.sh my-estop.service
```

## 11. 安全建议

1. 首次部署后先在现场监护下验证急停链路。
2. 每次升级脚本或系统后都执行一次状态与日志检查。
3. 不要把急停节点和高风险动作程序放在同一终端手工混跑。
4. 现场交接时保留 `systemctl status` 与 `journalctl` 检查步骤。
