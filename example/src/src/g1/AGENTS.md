# g1 子树知识库

## 概览
- 这里是 G1 相关示例汇总；同一层同时包含高层服务、低层控制、手部 dex3、音频客户端和独立 GPIO 急停 demo。

## 结构
```text
example/src/src/g1/
├── high_level/     # loco / motion switch / arm action / arm sdk dds
├── lowlevel/       # 低层控制、双臂示例、behavior_lib 数据
├── audio_client/   # 音频/TTS/LED 示例
├── dex3/           # 灵巧手示例
└── GPIO/           # 独立 CMake 的 libgpiod 急停检测 demo
```

## 优先查看
| 任务 | 位置 | 备注 |
|---|---|---|
| 改高层运动/模式切换 | `high_level/g1_loco_service_example.cpp`, `high_level/g1_motion_switcher_example.cpp` | 运行前都会碰到“用户调试模式”约束 |
| 改低层控制 | `lowlevel/g1_low_level_example.cpp`, `lowlevel/g1_dual_arm_example.cpp` | 使用 `unitree_hg`；双臂示例额外依赖 `yaml-cpp` |
| 查动作/风险提示 | `high_level/g1_arm_action_example.cpp` | 某些动作可能导致跌倒 |
| 查行为数据依赖 | `lowlevel/behavior_lib/motion.seq` | 双臂/轨迹类示例会依赖资源文件 |
| 查急停 GPIO demo | `GPIO/AGENTS.md`, `GPIO/main.cpp`, `GPIO/gpio_estop.hpp` | 这里不是 colcon 目标，而是独立 CMake 小工程 |

## 本目录约定
- 高层示例基本都是“一次运行只接受一个操作参数”；CLI 解析失败会直接退出并打印用法。
- `g1_loco_service_example` 会先检查当前运控模式；若机器人处于用户调试模式，高层服务直接不可用。
- 低层示例统一走 `unitree_hg` 的 `lowcmd/lowstate`；不要混入 `unitree_go` 的消息或 topic 假设。
- `g1_dual_arm_example` 只有 `yaml-cpp >= 0.6` 时才会在 `example/src/CMakeLists.txt` 中被启用。
- `GPIO/` 是独立构建链：本地 `cmake -S . -B build`，不经过根目录 `colcon build`。

## 本目录反模式
- 不要在用户调试模式下排查高层 loco / motion switch 失败；先切回内置运控模式。
- 不要给 `g1_loco_service_example`、`g1_motion_switcher_example` 这类程序同时传多个动作参数。
- 不要忽略 `g1_arm_action_example.cpp` 的风险提示；某些隐藏动作不会在 APP 展示，但程序仍可执行。
- 不要把 `GPIO/` 的独立 CMake 规则误抄到 `example/src/CMakeLists.txt`；它不是当前 ROS2 包的一部分。

## 备注
- 这是 example 工作区里最值得单独下钻的目录：子功能多、硬件风险高、同时混有 ROS2 示例与非 ROS2 独立 demo。
- 若文档命令与源码不一致，优先信这里的源码与 `example/src/CMakeLists.txt` 的真实挂载关系。
