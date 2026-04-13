# G1 运控切换程序使用说明

## 1. 程序简介

`g1_motion_switcher_example` 用于通过 Unitree 提供的运控切换接口控制 G1 的运控模式，支持：

- 查询当前运控模式
- 释放当前运控模式，进入用户调试模式
- 选择并切换到指定运控模式

该程序基于当前工作区已有的 `MotionSwitchClient` 实现，封装了以下接口：

- `CheckMode()`
- `ReleaseMode()`
- `SelectMode()`

## 2. 程序位置

可执行程序位于：

```bash
/home/guobing/g1_text_ws/unitree_ros2/example/install/unitree_ros2_example/bin/g1_motion_switcher_example
```

源码位于：

```bash
/home/guobing/g1_text_ws/unitree_ros2/example/src/src/g1/high_level/g1_motion_switcher_example.cpp
```

## 3. 运行前准备

### 3.1 source 环境

运行前先加载 ROS2 和工作区环境：

```bash
source /opt/ros/jazzy/setup.bash
source /home/guobing/g1_text_ws/unitree_ros2/cyclonedds_ws/install/setup.bash
source /home/guobing/g1_text_ws/unitree_ros2/example/install/setup.bash
```

### 3.2 连接机器人

确保：

- 机器人已开机
- 电脑与机器人网络已正确连接
- DDS/ROS2 网络配置正确

如果未连接机器人，程序通常会返回：

- `-1`：RPC 调用超时

## 4. 使用方法

### 4.1 查询当前运控模式

```bash
g1_motion_switcher_example --check
```

作用：

- 查询当前机器人处于哪个运控模式
- 如果没有激活模式，说明当前已进入用户调试模式

### 4.2 释放运控模式

```bash
g1_motion_switcher_example --release
```

作用：

- 释放当前运控服务控制权
- 释放后可进入用户自定义调试/开发模式

### 4.3 切换到指定运控模式

例如切换到主运控模式 `ai`：

```bash
g1_motion_switcher_example --select=ai
```

作用：

- 将机器人切换到指定运控模式
- 文档中说明 `ai` 为主运控模式

## 5. 参数说明

| 参数 | 说明 |
|---|---|
| `--check` | 查询当前运控模式 |
| `--release` | 释放当前运控模式 |
| `--select=<mode_name>` | 切换到指定模式 |

注意：

- 一次运行**必须且只能指定一个参数**
- 例如下面这种写法是错误的：

```bash
g1_motion_switcher_example --check --release
```

## 6. 输出说明

### 查询成功示例

```bash
[INFO] [xxx] [g1_motion_switcher_example]: CheckMode 调用成功
[INFO] [xxx] [g1_motion_switcher_example]: 当前运控模式: form=0, name=ai
```

### 已释放模式示例

```bash
[INFO] [xxx] [g1_motion_switcher_example]: CheckMode 调用成功
[INFO] [xxx] [g1_motion_switcher_example]: 当前没有激活的运控模式，机器人处于用户调试模式
```

### 未连接机器人示例

```bash
[ERROR] [xxx] [g1_motion_switcher_example]: CheckMode 调用失败，错误码: -1，描述: RPC 调用超时
```

## 7. 常见错误码

程序已对以下错误码做了提示：

| 错误码 | 含义 |
|---|---|
| `0` | 成功 |
| `-1` | RPC 调用超时 |
| `-2` | RPC 调用未知错误 |
| `7001` | 请求参数错误 |
| `7002` | 切换服务繁忙，请稍后再试 |
| `7004` | 运控模式名不支持 |
| `7005` | 运控模式内部指令执行错误 |
| `7006` | 运控模式检测指令执行错误 |
| `7007` | 运控模式切换指令执行错误 |
| `7008` | 运控模式释放指令执行错误 |
| `7009` | 自定义配置错误 |

## 8. 直接使用完整路径运行

如果命令找不到，也可以直接运行二进制文件：

```bash
/home/guobing/g1_text_ws/unitree_ros2/example/install/unitree_ros2_example/bin/g1_motion_switcher_example --check
```

## 9. 示例流程

### 释放当前运控模式

```bash
g1_motion_switcher_example --release
```

### 查看当前状态

```bash
g1_motion_switcher_example --check
```

### 恢复到主运控模式

```bash
g1_motion_switcher_example --select=ai
```
