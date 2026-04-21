# foxglove 知识库

## 概览
- 这里不是 ROS2 package；主要放 G1 可视化资源：URDF/MJCF 模型、mesh、Foxglove layout 模板，以及桥接脚本。

## 结构
```text
foxglove/
├── g1_description/                # G1 模型变体；README 列 mode_machine 与废弃型号
├── g1_foxglove_layout.template.json  # 面板布局；topicPath 约定在这里
└── run_foxglove_bridge_g1.sh      # 仅启动 foxglove_bridge 的局部脚本
```

## 优先查看
| 任务 | 位置 | 备注 |
|---|---|---|
| 选 G1 模型变体 | `g1_description/README.md` | 先看 `mode_machine` 对照表，再选 URDF/MJCF 文件 |
| 改 Foxglove 3D/Plot 面板 | `g1_foxglove_layout.template.json` | `topicPath`、3D 图层、plot 路径都写死在模板里 |
| 改局部 bridge 话题白名单 | `run_foxglove_bridge_g1.sh` | 当前脚本只启动 bridge，且与根层同名脚本内容一致 |
| 对照根层同名脚本 | `../run_foxglove_bridge_g1.sh` | 当前内容与本目录脚本一致，不存在额外 `robot_state_publisher` 步骤 |

## 本目录约定
- `g1_description/` 里是静态资产，不参与 `colcon build`；大量 `.STL`、`.urdf`、`.xml` 只是资源，不是源码生成物。
- `g1_description/README.md` 明确区分 Up-to-date 与 Deprecated 模型；选型前先核对 `mode_machine`、手部配置、腰部锁定状态。
- `foxglove/run_foxglove_bridge_g1.sh` 与根目录 `run_foxglove_bridge_g1.sh` 都是 zsh 脚本，且当前文件内容一致：都依赖 Jazzy 环境与硬编码 `ROOT_DIR`。
- 当前两个脚本都只启动 `foxglove_bridge`；如果后续要注入 `robot_description`，需要显式新增 `robot_state_publisher` 相关步骤，而不是假定根脚本已处理。
- layout 模板默认绑定 `/lowstate`、`/secondary_imu`、`/wirelesscontroller`、`/utlidar/*`、`/grid_clouds` 等 topic；改 topic 名时要同步改模板和 bridge 白名单。

## 本目录反模式
- 不要把这里当成示例源码树；业务控制逻辑不在 `foxglove/`。
- 不要直接照抄脚本里的 `ROOT_DIR="/home/guobing/My_Repositories/unitree_ros2"`；它绑定当前机器目录与本机安装路径。
- 不要只改 `g1_foxglove_layout.template.json` 不改 bridge 白名单；面板能订阅的话题受脚本 `TOPIC_WHITELIST` 限制。
- 不要继续优先使用 README 里标成 Deprecated 的 `g1_23dof`、`g1_29dof`、`g1_29dof_with_hand`、`g1_29dof_lock_waist` 老模型。

## 备注
- `foxglove/` 文件数很多，但绝大多数是 mesh 资产；真正高价值入口很少，先读 README、layout、bridge 脚本即可。
- 若只排查 Foxglove 3D 不显示机器人，先区分是“当前 bridge 脚本根本没提供 `robot_description` / `/tf` 补充链路”还是“URDF 路径/模型变体选错”。
