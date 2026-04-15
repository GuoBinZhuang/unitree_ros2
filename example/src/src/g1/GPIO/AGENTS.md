# G1 GPIO 急停 demo 知识库

## 概览
- 这是一个独立的 libgpiod 急停检测 demo，不走仓库主 colcon 构建链；核心是 `gpio_estop.hpp` 的 epoll + 线程化 GPIO 监听。

## 结构
```text
example/src/src/g1/GPIO/
├── CMakeLists.txt   # 独立 CMake 入口，仅支持 libgpiod
├── main.cpp         # demo 主程序、现场诊断、运行日志
└── gpio_estop.hpp   # 急停检测器实现
```

## 优先查看
| 任务 | 位置 | 备注 |
|---|---|---|
| 改构建方式 | `CMakeLists.txt` | 固定 `USE_LIBGPIOD=ON`；直接 `cmake -S . -B build` |
| 改 GPIO 映射/判定逻辑 | `gpio_estop.hpp` | 当前固定 `gpiochip1:15` 输入、`gpiochip1:14` 输出低电平 |
| 查启动诊断/占用冲突 | `main.cpp` | 会打印 `gpioinfo` / `pgrep` 结果帮助定位占线问题 |

## 本目录约定
- 仅支持 libgpiod 后端；`USE_LIBGPIOD=OFF` 会直接 `FATAL_ERROR` / `#error`。
- 当前模式是 detection-only：按下/释放只上报状态，不直接触发机器人运控动作。
- 构建命令默认使用本目录下的 `build/`；这里的 `build/` 指这个独立 demo 的产物目录，不是仓库根的 colcon 产物。
- `startupSelfCheck()` 会在启动阶段校验输入线电平和输出线状态；失败就退出，避免“无保护运行”。
- 运行期故障会通过 `runtime_error_`、事件序号和 epoll 线程上报，而不是静默吞错。

## 本目录反模式
- 不要关闭 libgpiod 后端再尝试编译；当前源码树根本不支持其他后端。
- 不要改线号/有效电平后却不同时更新自检预期与现场文档；这里的映射是硬编码约定。
- 不要把这个 demo 当成 ROS2 节点来接入 `example/src/CMakeLists.txt`；它是独立 CMake 工程。
- 不要忽略占用冲突诊断；`epoll_ctl add fd failed`、line request 失败通常不是业务逻辑 bug，而是 GPIO 已被别的进程占用。

## 备注
- `gpio_estop.hpp` 是仓库里少见的复杂单文件热点；涉及线程、条件变量、epoll、FD 生命周期和回调队列。
- 若需要把急停信号接入上层控制，先保持这里的 detection-only 语义不变，再在外层做动作绑定。
