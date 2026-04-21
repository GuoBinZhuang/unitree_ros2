# .github 知识库

## 概览
- 这里只有工作流；它们是本仓库最接近“可复现构建脚本”的地方。

## 结构
```text
.github/
└── workflows/
    ├── build-foxy.yml
    ├── build-humble.yml
    ├── release.yml
    └── auto-tag.yml
```

## 优先查看
| 任务 | 位置 | 备注 |
|---|---|---|
| 查 Humble 构建流程 | `workflows/build-humble.yml` | 容器、依赖、构建顺序完整 |
| 查 Foxy 构建流程 | `workflows/build-foxy.yml` | 包含 CycloneDDS 先构建逻辑 |
| 查发布自动化 | `workflows/release.yml`, `workflows/auto-tag.yml` | 只在做版本/发布相关改动时看 |

## 本目录约定
- CI 在容器里构建，不依赖本机状态。
- 两个 build workflow 都会先拉取 `rmw_cyclonedds` 和 `cyclonedds`，再构建 `cyclonedds`，再构建 `unitree_*` 包，最后构建 `unitree_ros2_example`。
- workflow 使用 `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`；本地排查 C++ 问题时可照抄。
- `build-foxy.yml` 与 `build-humble.yml` 当前都只在 `master` 分支的 push / pull_request 上触发。

## 本目录反模式
- 不要只改 README 构建说明而不核对 workflow；CI 才是回归基准。
- 不要假设 master 以外分支会自动触发同样逻辑；当前触发条件写死在 workflow。
- 不要省略 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；这是仓库构建与运行前提之一。
- 不要把 release / auto-tag 工作流当成常规构建入口；它们解决的是发版，不是日常编译验证。

## 备注
- 当 README、setup 脚本、实际构建行为冲突时，优先参考这里，再回查源码。
- 做发布相关工作前，先区分“构建验证”和“打 tag / release”两条流程。
