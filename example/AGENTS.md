# example 知识库

## 概览
- 这是示例工作区；真实 ROS2 package 根在 `example/src`，可执行程序源码集中在 `example/src/src`。

## 结构
```text
example/
└── src/
    ├── CMakeLists.txt        # 所有示例目标入口
    ├── package.xml
    ├── include/
    │   ├── common/           # 公共 client / CRC / 工具
    │   ├── g1/ b2/           # 机器人专用头
    │   └── nlohmann/         # vendored 第三方 JSON，默认只读
    └── src/
        ├── common/           # 公共实现
        ├── go2/ b2/ b2w/
        ├── g1/high_level/
        ├── g1/lowlevel/
        ├── g1/dex3/
        └── h1-2/lowlevel/
```

## 优先查看
| 任务 | 位置 | 备注 |
|---|---|---|
| 新增示例可执行文件 | `src/CMakeLists.txt` | 需同时加 `add_executable`、依赖、`install` |
| 复用公共 client | `src/include/common`, `src/src/common` | 多机器人示例共享 |
| 改 G1 高层示例 | `src/src/g1/high_level` | loco / motion switch / arm action |
| 改 G1 低层示例 | `src/src/g1/lowlevel` | 含 `behavior_lib` 数据目录 |
| 改 Go2/B2 示例 | `src/src/go2`, `src/src/b2`, `src/src/b2w` | 多数依赖 `unitree_go` |

## 本目录约定
- `example/src/CMakeLists.txt` 是单一事实源；有没有示例、是否安装、启用哪些编译特性，都在这里看。
- 许多示例显式要求 `cxx_std_20`，即使包默认是 C++14。
- 共享头实现分离：声明常在 `include/common`，实现常在 `src/common`。
- `yaml-cpp` 不是硬依赖；`g1_dual_arm_example` 仅在版本满足时构建。

## 本目录反模式
- 不要编辑 `src/include/nlohmann/**` 来修业务问题。
- 不要只新增 `add_executable` 不补 `ament_target_dependencies` 或 `install`。
- 不要忽视机器人型号分层；`unitree_go` 与 `unitree_hg` 消息不可混用。
- 不要在真实机器人上直接运行带明显风险的示例而不看注释/文档；`g1_arm_action_example.cpp` 已提示某些动作可能导致跌倒。

## 备注
- 本目录文件最多、变化面最广；未来 agent 处理“新增示例/改示例/排查构建失败”应先读这里。
- 若只做文档或 CI 变更，不必下钻到本目录。
