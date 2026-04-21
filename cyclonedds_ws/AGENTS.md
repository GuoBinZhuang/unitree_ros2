# cyclonedds_ws 知识库

## 概览
- 这是消息包工作区，不是示例区；主要职责是生成 `unitree_api`、`unitree_go`、`unitree_hg` 的 ROS2 接口。

## 结构
```text
cyclonedds_ws/
├── src/cyclonedds.xml        # DDS 配置
└── src/unitree/
    ├── unitree_api/          # request/response 协议消息
    ├── unitree_go/           # Go2/B2 消息
    └── unitree_hg/           # G1/H1/H1-2 消息
```

## 优先查看
| 任务 | 位置 | 备注 |
|---|---|---|
| 增删协议消息 | `src/unitree/*/msg/*.msg` | 改消息就要看对应 `CMakeLists.txt` |
| 调整接口生成 | `src/unitree/*/CMakeLists.txt` | 都使用 `rosidl_generate_interfaces` + DDS IDL |
| 查包依赖 | `src/unitree/*/package.xml` | 三个包边界清晰 |
| 查 DDS 配置 | `src/cyclonedds.xml` | 与网络/发现问题有关 |

## 本目录约定
- 这里几乎没有业务逻辑；重心是消息定义与生成配置。
- 三个包都使用 `ament_cmake`、`geometry_msgs`、`rosidl_default_generators`、`rosidl_generator_dds_idl`。
- `unitree_go` 消息更多，覆盖 Go2/B2；`unitree_hg` 面向 G1/H1/H1-2；`unitree_api` 是通用请求响应协议层。
- 修改 `.msg` 后，通常三个层面都要同步检查：消息字段、IDL 生成、示例代码编译面。

## 本目录反模式
- 不要把这里当成示例运行入口；可执行程序不在本目录。
- 不要只改 `.msg` 不看 `CHANGELOG.md`；HG 手部相关消息已发生 breaking change。
- 不要忽略 `unitree_api` 与 `unitree_go`/`unitree_hg` 的职责差异：前者是协议请求响应，后两者是状态/控制消息。

## 备注
- 评分高的原因：50 个文件、清晰模块边界、100% 代码型文件、对整个仓库构建链路居中。
- 如需继续细分，优先到 `src/unitree/` 层而不是每个 `msg/` 子目录。
