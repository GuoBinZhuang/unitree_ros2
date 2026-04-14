# docs 知识库

## 概览
- 这里放的是面向操作的说明文档，当前重点是 G1 高层运动与 motion switch 场景。

## 结构
```text
docs/
├── g1_loco_service_usage.md
├── g1_motion_switcher_usage.md
└── image/
```

## 优先查看
| 任务 | 位置 | 备注 |
|---|---|---|
| 查 G1 高层运动服务参数 | `g1_loco_service_usage.md` | 中文、命令级说明、错误示例多 |
| 查 G1 motion switch 用法 | `g1_motion_switcher_usage.md` | 与内置运控/调试模式切换有关 |
| 查配图引用 | `image/` | README 和文档会引用 |

## 本目录约定
- 文档偏“如何运行”，不是源码真理源；实现细节仍应回到 `example/src/src/g1/high_level`。
- 文档里出现了硬编码路径（如 `/home/guobing/g1_text_ws/unitree_ros2/...`）；迁移环境时要改心智，不要照抄路径。
- 文档里的 ROS2 版本和根目录脚本未必一致；出现 Jazzy/Humble/Foxy 混用时，以目标部署环境和 CI 为准。

## 本目录反模式
- 不要把文档中的绝对路径提交回代码或脚本。
- 不要只看文档命令就改源码接口；接口真相在 `example/` 与 `cyclonedds_ws/`。
- 不要忽略安全提醒；高层持续运动和模式切换都可能带来真实硬件风险。

## 备注
- 本目录规模不大，但领域独立且高价值，适合作为单独 AGENTS 节点。
- 若文档内容与源码不一致，优先信 `example/` 实现与 `.github/` 构建流程。
