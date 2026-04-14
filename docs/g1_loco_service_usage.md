# G1 高层运动服务示例程序使用说明

## 1. 程序简介

`g1_loco_service_example` 是一个基于 G1 高层运动 RPC 接口的 ROS2 示例程序，封装了常用整机控制能力，包括：

- 查询当前 FSM 模式
- 切换阻尼、主运控、下蹲、落座、站立、零力矩模式
- 发送移动速度指令
- 停止移动
- 开关连续步态
- 开关 Move 持续响应模式
- 设置速度模式

程序内部使用：

- `unitree::robot::g1::LocoClient`
- `unitree::robot::g1::MotionSwitchClient`

其中 `MotionSwitchClient` 仅用于运行前检查当前是否仍在内置运控模式。如果机器人已经进入用户调试模式，高层运动服务将失效，程序会直接给出提示。

## 2. 源码与可执行文件位置

源码：

```bash
/home/guobing/g1_text_ws/unitree_ros2/example/src/src/g1/high_level/g1_loco_service_example.cpp
```

可执行文件：

```bash
/home/guobing/g1_text_ws/unitree_ros2/example/install/unitree_ros2_example/bin/g1_loco_service_example
```

## 3. 运行前准备

先加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source /home/guobing/g1_text_ws/unitree_ros2/cyclonedds_ws/install/setup.bash
source /home/guobing/g1_text_ws/unitree_ros2/example/install/setup.bash
```

确保：

- 机器人已开机
- 网络配置正确
- 当前不是用户调试模式

如果之前执行过：

```bash
g1_motion_switcher_example --release
```

那么请先恢复到内置运控模式，例如：

```bash
g1_motion_switcher_example --select=ai
```

## 4. 基本用法

一次运行只能指定一个参数。

### 4.1 查询状态

```bash
g1_loco_service_example --get_fsm_id
g1_loco_service_example --get_fsm_mode
```

### 4.2 模式切换

```bash
g1_loco_service_example --damp          # 阻尼模式：关节带阻尼，便于人工扶持/拖动，通常用于过渡与保护
g1_loco_service_example --zero_torque   # 零力矩模式：电机关节尽量不主动输出力矩（近似“松力”状态）
g1_loco_service_example --start         # 主运控模式：进入高层运动控制主状态（后续步态/移动控制前常先执行）
g1_loco_service_example --squat         # 下蹲模式：机器人下蹲到较低姿态
g1_loco_service_example --sit           # 落座模式：机器人从站立过渡到坐姿
g1_loco_service_example --stand_up      # 起立模式：从坐姿或低姿态恢复到站立
g1_loco_service_example --balance_stand # 平衡站立模式：维持稳定站立，常作为行走前的稳定准备状态
```

### 4.3 移动控制

发送 1 秒速度指令：

```bash
g1_loco_service_example --move="0.2 0.0 0.0"
```

显式指定持续时间：

```bash
g1_loco_service_example --set_velocity="0.2 0.0 0.0 2.0"
```

停止移动：

```bash
g1_loco_service_example --stop_move
```

### 4.4 连续步态与 Move 响应模式

开启/关闭连续步态：

```bash
g1_loco_service_example --continuous_gait=true
g1_loco_service_example --continuous_gait=false
```

开启/关闭 Move 持续响应模式：

```bash
g1_loco_service_example --switch_move_mode=true
g1_loco_service_example --switch_move_mode=false
```

### 4.5 设置速度模式

```bash
g1_loco_service_example --set_speed_mode=0
g1_loco_service_example --set_speed_mode=1
g1_loco_service_example --set_speed_mode=2
g1_loco_service_example --set_speed_mode=3
```

含义：

- `0`：1.0m/s
- `1`：2.0m/s
- `2`：2.7m/s
- `3`：3.0m/s

## 5. 常用流程示例

### 5.1 切到主运控并查询状态

```bash
g1_loco_service_example --start
g1_loco_service_example --get_fsm_id
g1_loco_service_example --get_fsm_mode
```

### 5.2 让机器人缓慢前进

```bash
g1_loco_service_example --start
g1_loco_service_example --balance_stand
g1_loco_service_example --move="0.2 0.0 0.0"
```

### 5.3 开启持续响应模式后连续发送 Move

```bash
g1_loco_service_example --switch_move_mode=true
g1_loco_service_example --move="0.2 0.0 0.0"
g1_loco_service_example --move="0.0 0.0 0.3"
g1_loco_service_example --stop_move
```

## 6. 注意事项

1. 高层运动服务依赖内置运控。
2. 如果机器人处于调试模式，高层运动服务不可用。
3. `--move` 默认只生效约 1 秒。
4. 连续步态和持续响应模式涉及持续运动，实际使用时请注意安全。

## 7. 典型错误

### 7.1 调试模式下运行

程序会提示：

```text
当前机器人处于用户调试模式，高层运动服务依赖内置运控，无法使用。请先切回 ai 等内置运控模式。
```

### 7.2 未连接机器人

可能看到超时错误：

```text
错误码: -1，描述: RPC 调用超时
```

### 7.3 参数格式错误

例如：

```bash
g1_loco_service_example --move="0.2 0.0"
```

会提示参数数量不正确。
