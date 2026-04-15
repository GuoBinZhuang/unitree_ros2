# 项目知识库

**生成时间：** 2026-04-15 15:27:14 +08:00
**提交：** `9717120`
**分支：** `emergency_stop`

## 概览
- 这是 Unitree 机器人 ROS2 支持仓库；核心是 CycloneDDS 兼容消息包 + 示例程序。
- 仓库不是标准“根目录单工作区”；实际是两个并列工作区：`cyclonedds_ws/` 与 `example/`。

## 结构
```text
unitree_ros2/
├── cyclonedds_ws/   # 消息包工作区；先构建 DDS，再构建 unitree_* 包
├── example/         # 示例工作区；依赖上面的 unitree_* 消息包
├── docs/            # G1 专项中文使用说明；含硬编码路径示例
├── .github/         # Foxy/Humble CI；可直接当构建脚本参考
├── .devcontainer/   # Foxy/Humble 容器开发环境
├── setup*.sh        # 本机环境入口；设置 ROS2 + CycloneDDS 接口
├── build/ install/ log/  # colcon 产物；忽略
└── CHANGELOG.md     # 版本变更与 breaking changes
```

## 优先查看
| 任务 | 位置 | 备注 |
|---|---|---|
| 理解整体构建顺序 | `README.md`, `.github/workflows/build-*.yml` | CI 比 README 更接近当前可用流程 |
| 调整本机网络接口 | `setup.sh`, `setup_local.sh`, `setup_default.sh` | `setup.sh` 当前默认 `humble + eth2` |
| 修改 ROS2 消息定义 | `cyclonedds_ws/src/unitree/*/msg`, 对应 `CMakeLists.txt` | `unitree_go`/`unitree_hg`/`unitree_api` 分家明显 |
| 修改示例程序 | `example/src/src/**`, `example/src/include/**` | 可执行目标全在 `example/src/CMakeLists.txt` |
| 查 G1 高层服务使用方式 | `docs/g1_loco_service_usage.md` | 文档比 README 更具体 |
| 查破坏性变更 | `CHANGELOG.md` | 特别关注已删除 API 与 HG 手部消息变更 |

## 代码地图
| 区域 | 角色 | 位置 | 说明 |
|---|---|---|---|
| `unitree_api` | request/response 协议消息 | `cyclonedds_ws/src/unitree/unitree_api` | 供高层接口调用使用 |
| `unitree_go` | Go2/B2 消息集 | `cyclonedds_ws/src/unitree/unitree_go` | 包含 `SportMode*`、`Low*`、`WirelessController` |
| `unitree_hg` | G1/H1/H1-2 消息集 | `cyclonedds_ws/src/unitree/unitree_hg` | 含 `Hand*`、`PressSensorState` |
| `example/src/include/common` | 公共 client/CRC/工具头 | `example/src/include/common` | 多个示例复用 |
| `example/src/src/common` | 公共实现 | `example/src/src/common` | Go2/B2/G1 示例依赖 |
| `example/src/src/g1/high_level` | G1 高层控制示例 | `example/src/src/g1/high_level` | loco、motion switch、arm action |
| `example/src/src/g1/lowlevel` | G1 低层控制示例 | `example/src/src/g1/lowlevel` | 含 `behavior_lib` 数据目录 |

## 仓库约定（只写非通用项）
- 先构建 `cyclonedds_ws`，再构建 `example`；不要反过来。
- 本仓库默认中间件是 `rmw_cyclonedds_cpp`；多数排障都和 DDS/网络接口有关，不是业务代码本身。
- `example/src/CMakeLists.txt` 是示例真入口；新增/删除示例要同时更新 `add_executable`、`ament_target_dependencies`、`install`。
- `example/src/include/nlohmann/**` 是 vendored 第三方库；默认视为只读。
- `.github/workflows/build-foxy.yml` 与 `build-humble.yml` 直接展示了 CI 认可的依赖、环境变量、构建顺序。
- 仓库存在 ROS2 版本混用痕迹：README 与 `setup_local.sh`/`setup_default.sh` 偏 Foxy，`setup.sh` 当前改成 Humble，`docs/` 中 G1 文档又出现 Jazzy。

## 本项目反模式
- 不要把 `build/`、`install/`、`log/` 当源码目录分析或修改。
- 不要在 `example/src/include/nlohmann/**` 中做业务修改；那是第三方 vendored 代码。
- 不要继续引用 `CHANGELOG.md` 中 v0.2.0 已移除的 sport API：`SwitchGait`、`Trigger`、`BodyHeight`、`FootRaiseHeight`、`TrajectoryFollow`、`ContinuousGait`、`Wallow`、以及对应 Get API。
- 不要假设所有脚本都使用同一 ROS2 发行版；改环境脚本前先核对目标机器。
- 不要在机器人已进入“用户调试模式”时排查 G1 高层服务问题；`docs/g1_loco_service_usage.md` 明确说明此时高层服务不可用。

## 独特风格
- 消息包层几乎只有 `.msg + CMakeLists + package.xml`；业务逻辑基本不在 `cyclonedds_ws`。
- 示例层把“公共头/实现”和“按机器人型号分目录的示例”并存：`include/common` + `src/common` 对 `go2/`、`g1/`、`b2/`、`b2w/`、`h1-2/` 提供支撑。
- G1 文档偏任务导向，直接给运行命令、错误码和安全提醒；适合当操作手册，不适合当真理源。

## 常用命令
```bash
# 根目录环境
source ./setup.sh

# 构建 CycloneDDS 相关消息包工作区
cd cyclonedds_ws
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-select unitree_go unitree_hg unitree_api

# 构建示例
cd ../example
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-select unitree_ros2_example

# 典型连通性检查
source ~/unitree_ros2/setup.sh
ros2 topic list
```

## 备注
- 当前环境未安装 `clangd`；LSP codemap 不可用，目录决策主要基于结构、CMake、文档与 CI。
- 当前层级：`cyclonedds_ws/`、`example/`、`docs/`、`.github/`、`example/src/src/g1/`、`example/src/src/g1/GPIO/`。
- 若 README / setup 脚本 / 文档冲突，优先信 `.github/workflows/`，再回查对应源码目录。
