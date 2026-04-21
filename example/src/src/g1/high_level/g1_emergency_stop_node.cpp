#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "common/ut_errror.hpp"
#include "g1/g1_audio_client.hpp"
#include "g1/g1_loco_client.hpp"
#include "g1/g1_motion_switch_client.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../GPIO/gpio_estop.hpp"

namespace {

// 本文件实现 G1 机器人的 ROS2 急停节点，职责大致分成四层：
// 1. 通过 GPIO 检测实体急停按钮的按下/释放边沿；
// 2. 在检测到急停后立即发送高层 Damp 指令，让机器人切入阻尼模式；
// 3. 结合 TTS / LED / 日志把当前状态告知现场人员；
// 4. 在检测线程故障、收到系统退出信号等异常场景下，尽量走“受控退出”而不是直接中断。
//
// 这里的实现重点不是“尽快退出程序”，而是“在任何退出前尽量先把机器人放到更安全的状态”。

// G1 阻尼模式 FSM ID。
constexpr int kDampFsmId = 1;
// AudioClient 默认播报通道。
constexpr int kDefaultSpeakerId = 0;
// 启动、自检、异常退出等关键场景使用的固定播报文案。
constexpr const char *kStartupSelfCheckSuccessText = "急停自检通过";
constexpr const char *kStartupPressedWaitText =
    "急停按下状态，请抬起急停";
constexpr const char *kInterruptedShutdownText =
    "警告！急停程序已由键盘中断";
constexpr const char *kDampSwitchFailureShutdownText =
    "警告！急停动作失败，请立即检查";
constexpr const char *kDebugModeEStopFailureText =
    "警告！调试模式，急停失效";
constexpr const char *kDebugModeInvalidText =
    "警告！调试模式，急停无效";
constexpr const char *kProtectionLostShutdownText =
    "警告！急停保护失效，程序即将退出，请立即人工接管";
constexpr const char *kUnhandledExceptionText =
    "警告！急停程序异常退出，请检查系统状态";
// 不同链路使用的超时/轮询参数：退出提示尽量短，急停主动作优先保证切阻尼。
constexpr auto kShutdownTtsTimeout = std::chrono::milliseconds(200);
constexpr auto kShutdownLedTimeout = std::chrono::milliseconds(150);
constexpr auto kEmergencyDampRpcTimeout = std::chrono::milliseconds(1000);
constexpr auto kEmergencyFsmVerifyTimeout = std::chrono::milliseconds(300);
constexpr auto kEmergencyActionTtsTimeout = std::chrono::milliseconds(500);
constexpr auto kEmergencyActionLedTimeout = std::chrono::milliseconds(300);
// 开机阶段语音服务可能稍晚就绪：自检成功播报使用“短超时 + 重试”策略。
// 当前窗口约为 8 * 0.8s + 7 * 0.7s ~= 11.3s。
constexpr auto kStartupSelfCheckTtsTimeout = std::chrono::milliseconds(800);
constexpr auto kStartupSelfCheckTtsRetryInterval = std::chrono::milliseconds(700);
constexpr int kStartupSelfCheckTtsMaxAttempts = 8;
constexpr auto kMotionModeMonitorPeriod = std::chrono::seconds(1);
constexpr auto kMotionModeCheckTimeout = std::chrono::milliseconds(200);
constexpr auto kDebugModeAnnouncementDelay = std::chrono::seconds(5);
constexpr auto kDetectorRetryInterval = std::chrono::seconds(1);
constexpr auto kDetectorStartupDelay = std::chrono::milliseconds(500);
constexpr auto kDetectorHealthPollInterval = std::chrono::milliseconds(250);
constexpr auto kMainLoopFailureBurstWindow = std::chrono::seconds(5);
constexpr int kMainLoopMaxBurstFailures = 3;
constexpr size_t kExecutorThreadCount = 4;

// 信号处理仅做原子置位，避免在异步信号上下文做复杂操作。
volatile std::sig_atomic_t g_shutdown_signal = 0;
// 退出码采用“只升不降”的锁存策略，避免后续路径把更严重的错误覆盖掉。
std::atomic<int> g_requested_exit_code{0};

void ShutdownSignalHandler(int signal) {
  g_shutdown_signal = signal;
}

int ConsumeShutdownSignal() {
  // 读取后立即清零，保证每个信号只消费一次。
  const int signal = g_shutdown_signal;
  g_shutdown_signal = 0;
  return signal;
}

// 多个失败路径可能并发请求退出码，这里保留最高优先级的退出状态。
void RequestProcessExitCode(int exit_code) {
  int current_code = g_requested_exit_code.load(std::memory_order_acquire);
  while (current_code < exit_code &&
         !g_requested_exit_code.compare_exchange_weak(
             current_code, exit_code, std::memory_order_acq_rel,
             std::memory_order_acquire)) {
  }
}

// 读取当前已锁存的进程退出码。
int RequestedProcessExitCode() {
  return g_requested_exit_code.load(std::memory_order_acquire);
}

// LED 提示使用的 RGB 三通道颜色。
struct LedColor {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// 通知结果类型：成功通常表示已确认切到阻尼，失败表示需要人工注意。
enum class NotifyType {
  kSuccess,
  kFailure,
};

// 高层运控模式可用性：内置运控可继续调用高层服务，用户调试模式则视为急停高层链路不可信。
enum class MotionModeAvailability {
  kUnknown,
  kBuiltin,
  kUserDebug,
};

// 单次急停动作的最终结果，用来决定后续提示和日志级别。
enum class DampAttemptResult {
  kSwitched,
  kFailed,
  kDebugModeUnconfirmed,
};

// 把常见退出信号转成便于日志打印的名称。
const char *SignalName(int signal) {
  switch (signal) {
    case SIGINT:
      return "SIGINT";
    case SIGTERM:
      return "SIGTERM";
    default:
      return "UNKNOWN";
  }
}

void TryAnnounceWithTemporaryExecutor(
    const std::shared_ptr<unitree::ros2::g1::AudioClient> &audio_node,
    const char *text) {
  // 当主 executor 已经崩溃或还没来得及正常工作时，临时拉起一个小型 executor，
  // 尽量把“程序异常退出”这类高优先级提示播报出去。
  if (!audio_node || text == nullptr || !rclcpp::ok()) {
    return;
  }

  try {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(audio_node);
    std::atomic<bool> spin_running{true};
    std::thread spin_thread([&executor, &spin_running]() {
      while (spin_running.load(std::memory_order_acquire) && rclcpp::ok()) {
        executor.spin_some(std::chrono::milliseconds(20));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });

    int32_t tts_ret = UT_ROBOT_TASK_UNKNOWN_ERROR;
    try {
      tts_ret =
          audio_node->TtsMaker(text, kDefaultSpeakerId, kShutdownTtsTimeout);
    } catch (...) {
      spin_running.store(false, std::memory_order_release);
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
      throw;
    }

    spin_running.store(false, std::memory_order_release);
    executor.cancel();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }

    if (tts_ret != UT_ROBOT_SUCCESS) {
      std::cerr << "Unhandled-exception TTS failed, code=" << tts_ret
                << std::endl;
    }

    executor.remove_node(audio_node);
  } catch (const std::exception &e) {
    std::cerr << "Unhandled-exception TTS helper failed: " << e.what()
              << std::endl;
  } catch (...) {
    std::cerr << "Unhandled-exception TTS helper failed: unknown"
              << std::endl;
  }
}

class G1EmergencyStopNode final : public rclcpp::Node {
 public:
  // 构造时完成三件事：
  // 1. 读取急停相关参数；
  // 2. 绑定 GPIO 按下/释放回调；
  // 3. 启动两个后台线程，分别负责检测器生命周期和运控模式巡检。
  G1EmergencyStopNode()
      : rclcpp::Node("g1_emergency_stop_node"),
        // 消抖时间，避免机械触点抖动被误判成多次按下/释放。
        debounce_ms_(declare_parameter<int>("debounce_ms", 20)),
        // 某些接线方式按下时输出低电平，因此提供 active_low 参数适配。
        active_low_(declare_parameter<bool>("active_low", false)),
        // 急停动作失败后的最大重试次数；总尝试次数 = 1 次首发 + max_retries。
        max_retries_(declare_parameter<int>("max_retries", 2)),
        // 连续重试间的固定退避时间，避免对高层服务造成突发冲击。
        retry_backoff_ms_(declare_parameter<int>("retry_backoff_ms", 100)),
        // 同类型灯光通知的冷却时间，防止短时间内反复闪烁刷屏。
        notification_cooldown_ms_(
            declare_parameter<int>("notification_cooldown_ms", 2000)),
        success_led_({
          ClampColor(declare_parameter<int>("success_led_r", 0)),
          ClampColor(declare_parameter<int>("success_led_g", 255)),
          ClampColor(declare_parameter<int>("success_led_b", 0)),
        }),
        failure_led_({
          ClampColor(declare_parameter<int>("failure_led_r", 255)),
          ClampColor(declare_parameter<int>("failure_led_g", 0)),
          ClampColor(declare_parameter<int>("failure_led_b", 0)),
        }),
        rpc_callback_group_(
            create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
        detector_(debounce_ms_, active_low_),
        loco_client_(this, rpc_callback_group_),
        motion_switch_client_(this, rpc_callback_group_),
        audio_client_(std::make_shared<unitree::ros2::g1::AudioClient>()),
        last_notify_type_(NotifyType::kFailure),
        estop_active_(false),
        detector_started_(false),
        shutting_down_(false) {
    // GPIO 仅负责产生按下/释放事件，具体动作在回调内串联执行。
    detector_.onPressed([this]() { OnPressedEdge(); });
    detector_.onReleased([this]() { OnReleasedEdge(); });

    RCLCPP_INFO(get_logger(),
                "急停节点初始化完成: debounce_ms=%d active_low=%s retries=%d backoff_ms=%d",
                debounce_ms_, active_low_ ? "true" : "false", max_retries_, retry_backoff_ms_);

    // 将启动/恢复和运控模式巡检搬到独立工作线程，避免同步 RPC 挤占 executor。
    detector_supervisor_thread_ =
        std::thread([this]() { DetectorSupervisorLoop(); });
    motion_mode_monitor_thread_ =
        std::thread([this]() { MotionModeMonitorLoop(); });
  }

  ~G1EmergencyStopNode() override {
    // 先置位关机标记，阻止回调触发新动作，再停止检测器。
    shutting_down_.store(true, std::memory_order_release);
    worker_cv_.notify_all();
    StopDetector();
    JoinWorkerThread(detector_supervisor_thread_);
    JoinWorkerThread(motion_mode_monitor_thread_);
  }

  // AudioClient 需要由主函数加入 executor，这里暴露共享节点句柄。
  std::shared_ptr<unitree::ros2::g1::AudioClient> GetAudioClientNode() const {
    return audio_client_;
  }

 private:
  class ScopedAtomicFlag final {
   public:
    // RAII 辅助类：函数进入时把某个“正在执行”标志置真，
    // 无论正常返回还是异常退出，析构时都统一清零。
    explicit ScopedAtomicFlag(std::atomic<bool> &flag) : flag_(flag) {}
    ScopedAtomicFlag(const ScopedAtomicFlag &) = delete;
    ScopedAtomicFlag &operator=(const ScopedAtomicFlag &) = delete;
    ~ScopedAtomicFlag() { flag_.store(false, std::memory_order_release); }

   private:
    std::atomic<bool> &flag_;
  };

  // 高层运动服务错误码映射。
  static std::map<int32_t, std::string> BuildLocoErrorMap() {
    return {
        {unitree::robot::g1::UT_ROBOT_LOCO_ERR_LOCOSTATE_NOT_AVAILABLE,
         "高层运动状态不可用"},
        {unitree::robot::g1::UT_ROBOT_LOCO_ERR_INVALID_FSM_ID,
         "无效的运控模式 ID"},
        {unitree::robot::g1::UT_ROBOT_LOCO_ERR_INVALID_TASK_ID,
         "无效的任务 ID"},
        {UT_ROBOT_TASK_TIMEOUT, "RPC 调用超时"},
        {UT_ROBOT_TASK_UNKNOWN_ERROR, "RPC 调用未知错误"},
    };
  }

  // 运控模式查询/切换服务的常见错误码映射，用于把日志从裸数字提升到可读中文。
  static std::map<int32_t, std::string> BuildMotionErrorMap() {
    return {
        {7001, "请求参数错误"},
        {7002, "切换服务繁忙"},
        {7004, "运控模式名不支持"},
        {7005, "运控模式内部指令执行错误"},
        {7006, "运控模式检测指令执行错误"},
        {7007, "运控模式切换指令执行错误"},
        {7008, "运控模式释放指令执行错误"},
        {7009, "自定义配置错误"},
        {UT_ROBOT_TASK_TIMEOUT, "RPC 调用超时"},
        {UT_ROBOT_TASK_UNKNOWN_ERROR, "RPC 调用未知错误"},
    };
  }

  static std::string ResolveErrorDescription(
      int32_t code, const std::map<int32_t, std::string> &error_map) {
    // 错误码映射表并不覆盖所有情况，因此兜底返回“未定义错误”。
    const auto it = error_map.find(code);
    if (it == error_map.end()) {
      return "未定义错误";
    }
    return it->second;
  }

  // LED 颜色参数由 ROS 参数读入，统一裁剪到 0~255 的有效范围。
  static uint8_t ClampColor(int value) {
    if (value < 0) {
      return 0;
    }
    if (value > 255) {
      return 255;
    }
    return static_cast<uint8_t>(value);
  }

  // 析构阶段统一 join 工作线程，避免线程泄漏或悬空访问 this。
  static void JoinWorkerThread(std::thread &thread) {
    if (thread.joinable()) {
      thread.join();
    }
  }

  template <typename Rep, typename Period>
  bool WaitForWorkerDelayOrShutdown(
      std::chrono::duration<Rep, Period> delay) const {
    // 后台线程不直接裸 sleep，而是挂在条件变量上等待。
    // 这样析构或保护失效时可以立即唤醒线程，缩短退出尾延迟。
    std::unique_lock<std::mutex> lock(worker_wait_mutex_);
    return worker_cv_.wait_for(lock, delay, [this]() {
      return shutting_down_.load(std::memory_order_acquire);
    });
  }

  // 默认沿用 AudioClient 自带的 TTS RPC 超时参数。
  void AnnounceByTts(const char *text, const char *scene) {
    AnnounceByTts(text, scene, unitree::ros2::g1::ROBOT_API_AUDIO_TTS_TIMEOUT);
  }

  void AnnounceByTts(const char *text, const char *scene,
                     std::chrono::milliseconds timeout) {
    // TTS 仅作为“增强提示”，失败不会影响急停主流程。
    if (text == nullptr) {
      return;
    }

    int32_t tts_ret = UT_ROBOT_TASK_UNKNOWN_ERROR;
    try {
      tts_ret = audio_client_->TtsMaker(text, kDefaultSpeakerId, timeout);
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "%s 播报异常: %s", scene, e.what());
      return;
    } catch (...) {
      RCLCPP_WARN(get_logger(), "%s 播报异常: unknown", scene);
      return;
    }
    if (tts_ret == UT_ROBOT_SUCCESS) {
      RCLCPP_INFO(get_logger(), "%s 播报成功: %s", scene, text);
      return;
    }

    // 语音播报可能已被设备受理但响应超时；此处不重试，避免重复播报。
    if (tts_ret == UT_ROBOT_TASK_TIMEOUT) {
      RCLCPP_WARN(get_logger(), "%s 播报超时(code=%d)，为避免重复播报不再重试",
                  scene, tts_ret);
      return;
    }

    RCLCPP_WARN(get_logger(), "%s 播报失败, code=%d", scene, tts_ret);
  }

  void AnnounceStartupSelfCheckByTts() {
    // 启动自检播报是“首次可感知提示”，允许在短时间内重试，
    // 以覆盖语音服务慢启动导致的首发失败场景。
    for (int attempt = 1; attempt <= kStartupSelfCheckTtsMaxAttempts;
         ++attempt) {
      // 退出流程触发后不再继续播报重试，避免拖慢停机。
      if (shutting_down_.load(std::memory_order_acquire)) {
        return;
      }

      int32_t tts_ret = UT_ROBOT_TASK_UNKNOWN_ERROR;
      try {
        tts_ret = audio_client_->TtsMaker(kStartupSelfCheckSuccessText,
                                          kDefaultSpeakerId,
                                          kStartupSelfCheckTtsTimeout);
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(),
                    "启动自检 播报异常(attempt=%d/%d): %s",
                    attempt, kStartupSelfCheckTtsMaxAttempts, e.what());
      } catch (...) {
        RCLCPP_WARN(get_logger(),
                    "启动自检 播报异常(attempt=%d/%d): unknown",
                    attempt, kStartupSelfCheckTtsMaxAttempts);
      }

      if (tts_ret == UT_ROBOT_SUCCESS) {
        RCLCPP_INFO(get_logger(), "启动自检 播报成功: %s (attempt=%d/%d)",
                    kStartupSelfCheckSuccessText, attempt,
                    kStartupSelfCheckTtsMaxAttempts);
        return;
      }

      if (attempt >= kStartupSelfCheckTtsMaxAttempts) {
        break;
      }

      RCLCPP_WARN(get_logger(),
                  "启动自检 播报未成功(code=%d, attempt=%d/%d)，"
                  "可能语音服务尚未就绪，将继续重试",
                  tts_ret, attempt, kStartupSelfCheckTtsMaxAttempts);
      // 使用条件变量等待，便于析构/受控退出时被立即唤醒。
      if (WaitForWorkerDelayOrShutdown(kStartupSelfCheckTtsRetryInterval)) {
        return;
      }
    }

    RCLCPP_WARN(get_logger(), "启动自检 播报最终失败：超过最大重试次数(%d)",
                kStartupSelfCheckTtsMaxAttempts);
  }

  bool HandleExternalStopSignalIfNeeded() {
    // 将常见外部停止信号统一收敛到受控退出流程。
    const int signal = ConsumeShutdownSignal();
    if (signal != SIGINT && signal != SIGTERM) {
      return false;
    }

    bool expected = false;
    if (!shutting_down_.compare_exchange_strong(expected, true,
                                                std::memory_order_acq_rel)) {
      return true;
    }

    RCLCPP_ERROR(get_logger(), "触发受控退出: 收到外部信号 %s",
                 SignalName(signal));
    StopDetector();
    damp_mode_confirmed_.store(false, std::memory_order_release);
    AnnounceByTts(kInterruptedShutdownText, "退出提示", kShutdownTtsTimeout);
    NotifyWithLed(NotifyType::kFailure, kShutdownLedTimeout);
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return true;
  }

  void HandleProtectionLoss(const std::string &reason) {
    // “保护失效”表示急停链路本身已经不再可信，例如 GPIO 检测线程异常。
    // 此时不能把程序继续当成一个有效保护装置运行，需要锁存故障并主动退出。
    bool expected = false;
    if (!protection_lost_.compare_exchange_strong(expected, true,
                                                  std::memory_order_acq_rel)) {
      return;
    }

    RequestProcessExitCode(1);
    startup_ready_.store(false, std::memory_order_release);
    damp_mode_confirmed_.store(false, std::memory_order_release);
    estop_active_.store(true, std::memory_order_release);

    RCLCPP_ERROR(get_logger(), "急停保护失效，锁存故障并准备退出: %s",
                 reason.c_str());
    StopDetector();

    HandleEmergencyStopAction("急停保护失效，尝试切换阻尼模式");
    AnnounceByTts(kProtectionLostShutdownText, "保护失效", kShutdownTtsTimeout);
    NotifyWithLed(NotifyType::kFailure, kShutdownLedTimeout);

    shutting_down_.store(true, std::memory_order_release);
    worker_cv_.notify_all();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void DetectorSupervisorLoop() {
    // 这个线程是 GPIO 急停检测器的“监工”：
    // - 启动阶段负责自检与自动重试；
    // - 运行阶段持续监听检测器健康状态；
    // - 一旦检测线程自身失效，就升级为保护失效并停机。
    WaitForWorkerDelayOrShutdown(kDetectorStartupDelay);

    while (!shutting_down_.load(std::memory_order_acquire)) {
      if (HandleExternalStopSignalIfNeeded()) {
        return;
      }

      StartDetector();
      if (shutting_down_.load(std::memory_order_acquire)) {
        return;
      }

      // 启动成功后转入运行时健康巡检；检测器一旦故障，则由本线程升级为保护失效并触发退出。
      uint64_t event_seq = detector_.getEventSequence();
      while (!shutting_down_.load(std::memory_order_acquire)) {
        detector_.waitForEvent(event_seq, kDetectorHealthPollInterval);

        if (HandleExternalStopSignalIfNeeded()) {
          return;
        }
        if (shutting_down_.load(std::memory_order_acquire)) {
          return;
        }
        if (!detector_started_.load(std::memory_order_acquire)) {
          break;
        }

        std::string runtime_error;
        if (!detector_.getRuntimeError(runtime_error)) {
          continue;
        }

        bool expected = false;
        // 只上报一次运行时故障，避免故障期间重复刷屏和重复提示。
        if (!runtime_fault_reported_.compare_exchange_strong(
                expected, true, std::memory_order_acq_rel)) {
          break;
        }

        HandleProtectionLoss("GPIO 急停检测线程故障: " + runtime_error);
        return;
      }
    }
  }

  void StartDetector() {
    bool expected = false;
    if (!detector_start_loop_running_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) {
      return;
    }
    const ScopedAtomicFlag start_loop_guard(detector_start_loop_running_);

    bool waiting_release_announced = false;
    bool startup_pressed_action_attempted = false;

    // 启动阶段循环自检：支持“急停处于按下状态时等待抬起后自动重检”。
    while (!shutting_down_.load(std::memory_order_acquire)) {
      if (HandleExternalStopSignalIfNeeded()) {
        return;
      }

      std::string startup_error;
      {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (!detector_started_.load(std::memory_order_acquire)) {
          // 每次重新启动检测器前，都把与“本轮保护状态”有关的标志恢复到初始值，
          // 避免沿用上一次启动/故障周期残留下来的状态。
          runtime_fault_reported_.store(false, std::memory_order_release);
          startup_ready_.store(false, std::memory_order_release);
          damp_mode_confirmed_.store(false, std::memory_order_release);
          estop_active_.store(false, std::memory_order_release);

          try {
            detector_.start();
            detector_started_.store(true, std::memory_order_release);
          } catch (const std::exception &e) {
            startup_error = e.what();
          }
        }
      }

      if (!startup_error.empty()) {
        HandleProtectionLoss("启动 GPIO 急停检测失败: " + startup_error);
        return;
      }

      // 启动后做自检：若急停已按下，则提示用户抬起后自动重检。
      const EStopDetector::StartupSelfCheckResult self_check =
          detector_.startupSelfCheckDetailed();
      if (self_check.state == EStopDetector::StartupSelfCheckState::kPassed) {
        // 自检通过后才允许把节点标记为 startup_ready_。
        // 在这之前即使收到了按下沿，也只锁存状态，不立即认定保护链已经稳定。
        const bool detector_active = detector_.isActive();
        const bool pending_pressed_before_ready =
            estop_active_.exchange(detector_active, std::memory_order_acq_rel);
        startup_ready_.store(true, std::memory_order_release);
        ResetMotionModeTracking();
        RCLCPP_INFO(get_logger(), "GPIO 急停检测已启动: %s",
                    self_check.report.c_str());
        if (detector_active || pending_pressed_before_ready) {
          estop_active_.store(true, std::memory_order_release);
          RCLCPP_WARN(get_logger(),
                      "启动完成前检测到急停事件，立即补执行阻尼切换");
          HandleEmergencyStopAction("启动完成前检测到急停事件");
          return;
        }
        // 启动自检播报允许短时重试，其他场景仍保持“单次播报不重试”。
        AnnounceStartupSelfCheckByTts();
        return;
      }

      if (self_check.state == EStopDetector::StartupSelfCheckState::kTriggered) {
        // 启动自检阶段如果发现按钮本来就是按下状态：
        // 1. 先尝试补发一次急停动作，避免机器人仍处于活动模式；
        // 2. 然后等待用户松开按钮，再自动重做自检。
        if (!startup_pressed_action_attempted) {
          startup_pressed_action_attempted = true;
          estop_active_.store(true, std::memory_order_release);
          HandleEmergencyStopAction("启动自检检测到急停已处于按下状态");
        }
        if (!waiting_release_announced) {
          waiting_release_announced = true;
          RCLCPP_WARN(get_logger(),
                      "启动自检检测到急停可能处于按下状态，等待抬起后自动重检: %s",
                      self_check.report.c_str());
          AnnounceByTts(kStartupPressedWaitText, "启动自检");
        }
        WaitForWorkerDelayOrShutdown(std::chrono::milliseconds(300));
        continue;
      }

      RCLCPP_ERROR(get_logger(), "GPIO 急停自检失败，进入保护失效退出流程: %s",
                   self_check.report.c_str());
      HandleProtectionLoss("GPIO 急停自检失败: " + self_check.report);
      return;
    }
  }

  void StopDetector() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (!detector_started_.load(std::memory_order_acquire)) {
      return;
    }

    detector_.stop();
    detector_started_.store(false, std::memory_order_release);
    startup_ready_.store(false, std::memory_order_release);
    damp_mode_confirmed_.store(false, std::memory_order_release);
    ResetMotionModeTracking();
    RCLCPP_INFO(get_logger(), "GPIO 急停检测已停止");
  }

  void MotionModeMonitorLoop() {
    // 高层 Damp / mode 查询依赖机器人处于“内置运控模式”。
    // 如果机器人切到用户调试模式，高层急停就可能失效，因此这里持续做低频巡检。
    while (!shutting_down_.load(std::memory_order_acquire)) {
      WaitForWorkerDelayOrShutdown(kMotionModeMonitorPeriod);
      if (HandleExternalStopSignalIfNeeded()) {
        return;
      }
      if (!startup_ready_.load(std::memory_order_acquire) ||
          shutting_down_.load(std::memory_order_acquire)) {
        continue;
      }

      std::string form;
      std::string name;
      const MotionModeAvailability availability =
          QueryMotionMode(&form, &name, nullptr, false, kMotionModeCheckTimeout);
      if (availability == MotionModeAvailability::kUnknown) {
        continue;
      }

      const auto now = std::chrono::steady_clock::now();
      bool entered_debug_mode = false;
      bool recovered_from_debug_mode = false;
      bool should_announce_debug_mode = false;

      {
        std::lock_guard<std::mutex> lock(mode_state_mutex_);
        if (availability == MotionModeAvailability::kUserDebug) {
          // 首次发现进入用户调试模式时先记时间，不立刻播报。
          // 这样可以过滤短暂切换，避免现场频繁收到误报。
          if (last_motion_mode_availability_ !=
              MotionModeAvailability::kUserDebug) {
            last_motion_mode_availability_ = availability;
            debug_mode_detected_at_ = now;
            debug_mode_announcement_pending_ = true;
            entered_debug_mode = true;
          } else if (debug_mode_announcement_pending_ &&
                     now - debug_mode_detected_at_ >=
                         kDebugModeAnnouncementDelay) {
            debug_mode_announcement_pending_ = false;
            should_announce_debug_mode = true;
          }
        } else {
          recovered_from_debug_mode =
              last_motion_mode_availability_ ==
              MotionModeAvailability::kUserDebug;
          last_motion_mode_availability_ = availability;
          debug_mode_announcement_pending_ = false;
        }
      }

      if (entered_debug_mode) {
        RCLCPP_WARN(get_logger(),
                    "检测到机器人进入用户调试模式，急停高层服务不可用；若 5 秒后仍保持调试模式，将播报失效提示");
      }

      if (recovered_from_debug_mode) {
        RCLCPP_INFO(get_logger(),
                    "检测到机器人已退出用户调试模式，取消急停无效播报等待");
      }

      if (should_announce_debug_mode &&
          !shutting_down_.load(std::memory_order_acquire)) {
        AnnounceByTts(kDebugModeInvalidText, "运控模式监测");
      }
    }
  }

  void OnPressedEdge() {
    // GPIO 检测到按下沿后的入口：
    // - 如果系统尚未完成启动自检，只记录“当前为按下状态”；
    // - 如果已经 ready，则只在首次按下时触发急停动作。
    if (shutting_down_.load(std::memory_order_acquire)) {
      return;
    }

    if (!startup_ready_.load(std::memory_order_acquire)) {
      estop_active_.store(true, std::memory_order_release);
      return;
    }

    bool expected = false;
    // 仅在“非急停 -> 急停”按下沿触发一次动作，按住不重复触发。
    if (!estop_active_.compare_exchange_strong(expected, true,
                                               std::memory_order_acq_rel)) {
      return;
    }

    HandleEmergencyStopAction("检测到急停按下沿");
  }

  void HandleEmergencyStopAction(const char *trigger_context) {
    // 所有急停动作串行化执行，避免多个来源（启动补偿、真实按下沿、保护失效）
    // 同时并发发送 Damp / 播报 / 灯光提示，造成状态交错。
    std::lock_guard<std::mutex> lock(action_mutex_);
    if (shutting_down_.load(std::memory_order_acquire)) {
      RCLCPP_WARN(get_logger(), "%s，但节点正在退出，跳过新的急停动作", trigger_context);
      return;
    }
    RCLCPP_WARN(get_logger(), "%s，开始切换阻尼模式", trigger_context);

    damp_mode_confirmed_.store(false, std::memory_order_release);
    const DampAttemptResult result = SwitchToDampWithRetry();
    if (result == DampAttemptResult::kSwitched) {
      damp_mode_confirmed_.store(true, std::memory_order_release);
      // 急停按下期间保持红灯，避免“切阻尼成功后变绿”造成现场误判。
      NotifyWithLed(NotifyType::kFailure, kEmergencyActionLedTimeout);
      RCLCPP_INFO(get_logger(), "急停处理完成：机器人已进入阻尼模式（按下期间保持红灯）");
      return;
    }

    NotifyWithLed(NotifyType::kFailure, kEmergencyActionLedTimeout);
    if (result == DampAttemptResult::kDebugModeUnconfirmed) {
      RCLCPP_WARN(
          get_logger(),
          "机器人处于调试模式：已尝试发送 Damp 保险命令，但未确认进入阻尼模式");
      AnnounceByTts(kDebugModeEStopFailureText, "急停处理",
                    kEmergencyActionTtsTimeout);
      return;
    }

    RCLCPP_ERROR(get_logger(), "急停处理失败：未能切换至阻尼模式");
    AnnounceByTts(kDampSwitchFailureShutdownText, "急停处理",
                  kEmergencyActionTtsTimeout);
  }

  void OnReleasedEdge() {
    // 释放急停只表示“按钮已松开”，不代表可以自动恢复运动。
    // 这里明确选择保守策略：如果已经确认进入阻尼模式，就保持阻尼等待人工处理。
    if (shutting_down_.load(std::memory_order_acquire)) {
      return;
    }

    if (!startup_ready_.load(std::memory_order_acquire)) {
      estop_active_.store(false, std::memory_order_release);
      return;
    }

    const bool was_active = estop_active_.exchange(false, std::memory_order_acq_rel);
    if (!was_active) {
      return;
    }

    if (damp_mode_confirmed_.load(std::memory_order_acquire)) {
      RCLCPP_INFO(get_logger(), "检测到急停释放沿：保持阻尼模式，不自动恢复");
      return;
    }

    RCLCPP_WARN(get_logger(),
                "检测到急停释放沿：此前未确认切换到阻尼模式，请立即人工检查机器人状态");
  }

  // 急停主动作：尝试切阻尼，失败按参数重试，并在每次成功后做 FSM 校验。
  DampAttemptResult SwitchToDampWithRetry() {
    if (shutting_down_.load(std::memory_order_acquire)) {
      return DampAttemptResult::kFailed;
    }

    const int total_attempts = max_retries_ + 1;

    // 先直接发送第一条 Damp，避免在急停热路径上额外等待运控模式查询 RPC。
    const int32_t first_damp_ret = loco_client_.Damp(kEmergencyDampRpcTimeout);
    if (first_damp_ret == UT_ROBOT_SUCCESS) {
      if (!shutting_down_.load(std::memory_order_acquire) &&
          VerifyDampFsm(kEmergencyFsmVerifyTimeout)) {
        return DampAttemptResult::kSwitched;
      }

      RCLCPP_WARN(get_logger(),
                  "首条 Damp 调用返回成功，但 FSM 校验未通过 (attempt=1/%d)",
                  total_attempts);
    } else {
      RCLCPP_ERROR(
          get_logger(),
          "首条 Damp 调用失败 (attempt=1/%d), code=%d, desc=%s", total_attempts,
          first_damp_ret,
          ResolveErrorDescription(first_damp_ret, BuildLocoErrorMap()).c_str());
    }

    if (shutting_down_.load(std::memory_order_acquire)) {
      return DampAttemptResult::kFailed;
    }

    const MotionModeAvailability motion_mode = CheckMotionMode();
    if (motion_mode == MotionModeAvailability::kUserDebug) {
      // 用户调试模式下，高层服务可能已经被旁路或失效。
      // 这里不再继续盲目重试，改为尽快提示人工接管。
      RCLCPP_ERROR(get_logger(),
                   "已直接发送一次 Damp 保险命令，但机器人仍处于用户调试模式且未确认进入阻尼模式");
      return DampAttemptResult::kDebugModeUnconfirmed;
    }

    for (int attempt = 1; attempt <= max_retries_; ++attempt) {
      if (shutting_down_.load(std::memory_order_acquire)) {
        break;
      }

      const int32_t damp_ret = loco_client_.Damp(kEmergencyDampRpcTimeout);
      if (damp_ret == UT_ROBOT_SUCCESS) {
        if (!shutting_down_.load(std::memory_order_acquire) &&
            VerifyDampFsm(kEmergencyFsmVerifyTimeout)) {
          return DampAttemptResult::kSwitched;
        }

        RCLCPP_WARN(get_logger(),
                    "Damp 调用返回成功，但 FSM 校验未通过 (attempt=%d/%d)",
                    attempt + 1, total_attempts);
      } else {
        RCLCPP_ERROR(
            get_logger(),
            "Damp 调用失败 (attempt=%d/%d), code=%d, desc=%s",
            attempt + 1, total_attempts, damp_ret,
            ResolveErrorDescription(damp_ret, BuildLocoErrorMap()).c_str());
      }

      if (attempt >= max_retries_) {
        break;
      }

      if (shutting_down_.load(std::memory_order_acquire)) {
        break;
      }

      // 简单固定退避，降低服务繁忙时的连续冲击。
      std::this_thread::sleep_for(std::chrono::milliseconds(retry_backoff_ms_));
    }

    return DampAttemptResult::kFailed;
  }

  MotionModeAvailability CheckMotionMode() {
    // 急停首发失败后再查询当前运控模式，用于区分：
    // - 只是临时 RPC 失败，仍可继续重试；
    // - 机器人已进入用户调试模式，此时应尽快人工接管。
    std::string form;
    std::string name;
    int32_t motion_mode_error_code = UT_ROBOT_SUCCESS;
    const MotionModeAvailability availability =
        QueryMotionMode(&form, &name, &motion_mode_error_code, true,
                        kMotionModeCheckTimeout);
    if (availability == MotionModeAvailability::kUnknown) {
      RCLCPP_WARN(get_logger(), "无法确认当前运控模式，继续尝试 Damp, code=%d, desc=%s",
                  motion_mode_error_code,
                  ResolveErrorDescription(motion_mode_error_code,
                                          BuildMotionErrorMap())
                      .c_str());
      return MotionModeAvailability::kUnknown;
    }

    if (availability == MotionModeAvailability::kUserDebug) {
      RCLCPP_WARN(get_logger(),
                  "当前机器人处于用户调试模式，停止额外 Damp 重试并等待人工接管");
      return MotionModeAvailability::kUserDebug;
    }

    RCLCPP_INFO(get_logger(), "当前运控模式: form=%s, name=%s", form.c_str(),
                name.c_str());
    return MotionModeAvailability::kBuiltin;
  }

  MotionModeAvailability QueryMotionMode(std::string *form,
                                         std::string *name,
                                         int32_t *error_code,
                                         bool log_failure,
                                         std::chrono::milliseconds timeout) {
    // Unitree 的模式查询接口在“用户调试模式”下通常拿不到内置模式名，
    // 这里约定：name 为空视为用户调试模式，非空视为内置运控模式。
    std::string local_form;
    std::string local_name;
    const int32_t ret =
        motion_switch_client_.CheckMode(local_form, local_name, timeout);
    if (ret != UT_ROBOT_SUCCESS) {
      if (error_code != nullptr) {
        *error_code = ret;
      }
      if (log_failure) {
        RCLCPP_WARN(get_logger(),
                    "无法通过运控切换服务确认当前模式，code=%d, desc=%s", ret,
                    ResolveErrorDescription(ret, BuildMotionErrorMap()).c_str());
      }
      return MotionModeAvailability::kUnknown;
    }

    if (form != nullptr) {
      *form = local_form;
    }
    if (name != nullptr) {
      *name = local_name;
    }
    if (error_code != nullptr) {
      *error_code = UT_ROBOT_SUCCESS;
    }
    return local_name.empty() ? MotionModeAvailability::kUserDebug
                              : MotionModeAvailability::kBuiltin;
  }

  void ResetMotionModeTracking() {
    // 检测器重启、节点停机时都重置调试模式跟踪状态，
    // 防止把上一轮检测结果误带到新一轮生命周期里。
    std::lock_guard<std::mutex> lock(mode_state_mutex_);
    last_motion_mode_availability_ = MotionModeAvailability::kUnknown;
    debug_mode_announcement_pending_ = false;
    debug_mode_detected_at_ = std::chrono::steady_clock::time_point{};
  }

  bool VerifyDampFsm(std::chrono::milliseconds timeout) {
    // 仅靠 Damp RPC 返回成功还不够，必须再读一次 FSM ID，
    // 确认真正切到了阻尼模式，才能认为急停动作完成。
    int fsm_id = -1;
    const int32_t fsm_ret = loco_client_.GetFsmId(fsm_id, timeout);
    if (fsm_ret != UT_ROBOT_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "GetFsmId 校验失败, code=%d, desc=%s", fsm_ret,
                   ResolveErrorDescription(fsm_ret, BuildLocoErrorMap()).c_str());
      return false;
    }

    if (fsm_id != kDampFsmId) {
      RCLCPP_ERROR(get_logger(), "FSM 校验失败：期望=%d 实际=%d", kDampFsmId,
                   fsm_id);
      return false;
    }

    RCLCPP_INFO(get_logger(), "FSM 校验成功：当前已是阻尼模式 (fsm_id=%d)", fsm_id);
    return true;
  }

  void NotifyWithLed(NotifyType type, std::chrono::milliseconds timeout) {
    // LED 与 TTS 一样都是“告警增强层”。
    // 即使通知失败，也不能反过来影响急停主动作判定。
    if (ShouldSkipNotification(type)) {
      RCLCPP_WARN(get_logger(), "灯光提示被节流，跳过本次通知");
      return;
    }

    // 同一入口统一处理成功/失败颜色。
    const LedColor color = (type == NotifyType::kSuccess) ? success_led_ : failure_led_;

    int32_t led_ret = UT_ROBOT_TASK_UNKNOWN_ERROR;
    try {
      led_ret = audio_client_->LedControl(color.r, color.g, color.b, timeout);
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "LedControl 异常: %s", e.what());
      return;
    } catch (...) {
      RCLCPP_WARN(get_logger(), "LedControl 异常: unknown");
      return;
    }
    if (led_ret != UT_ROBOT_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "LedControl 失败, code=%d", led_ret);
      return;
    }

    RCLCPP_INFO(get_logger(), "灯光提示成功: rgb=(%u,%u,%u)",
                static_cast<unsigned>(color.r), static_cast<unsigned>(color.g),
                static_cast<unsigned>(color.b));

    {
      std::lock_guard<std::mutex> lock(notify_mutex_);
      last_notify_at_ = std::chrono::steady_clock::now();
      last_notify_type_ = type;
    }
  }

  bool ShouldSkipNotification(NotifyType type) const {
    std::lock_guard<std::mutex> lock(notify_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (last_notify_at_.time_since_epoch().count() == 0) {
      return false;
    }

    if (type != last_notify_type_) {
      return false;
    }

    // 同类型通知在冷却时间内丢弃，避免短时重复提示。
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_notify_at_)
            .count();
    return elapsed_ms < notification_cooldown_ms_;
  }

  // 配置参数：按钮电平/重试/通知策略。
  const int debounce_ms_;
  const bool active_low_;
  const int max_retries_;
  const int retry_backoff_ms_;
  const int notification_cooldown_ms_;
  const LedColor success_led_;
  const LedColor failure_led_;

  // 对外部设备/服务的访问句柄。
  rclcpp::CallbackGroup::SharedPtr rpc_callback_group_;
  EStopDetector detector_;
  unitree::robot::g1::LocoClient loco_client_;
  unitree::robot::g1::MotionSwitchClient motion_switch_client_;
  std::shared_ptr<unitree::ros2::g1::AudioClient> audio_client_;

  // 两个后台线程：一个监管 GPIO 检测器，一个巡检运控模式。
  std::thread detector_supervisor_thread_;
  std::thread motion_mode_monitor_thread_;

  // 各类互斥量分别保护生命周期、急停动作、通知状态、模式状态和后台等待。
  mutable std::mutex lifecycle_mutex_;
  mutable std::mutex action_mutex_;
  mutable std::mutex notify_mutex_;
  mutable std::mutex mode_state_mutex_;
  mutable std::mutex worker_wait_mutex_;
  mutable std::condition_variable worker_cv_;

  std::chrono::steady_clock::time_point last_notify_at_{};
  NotifyType last_notify_type_;
  std::chrono::steady_clock::time_point debug_mode_detected_at_{};
  MotionModeAvailability last_motion_mode_availability_{
      MotionModeAvailability::kUnknown};
  bool debug_mode_announcement_pending_{false};

  // 原子状态位：
  // estop_active_             当前是否处于“按钮按下/急停激活”状态；
  // damp_mode_confirmed_      是否已确认机器人进入阻尼模式；
  // protection_lost_          急停保护链是否已经失效并锁存故障；
  // runtime_fault_reported_   检测线程运行时故障是否已上报过；
  // detector_start_loop_running_ 是否已有线程在执行启动/自检循环；
  // detector_started_         GPIO 检测器是否已启动；
  // startup_ready_            启动自检是否完成；
  // shutting_down_            是否进入全局退出流程。
  std::atomic<bool> estop_active_;
  std::atomic<bool> damp_mode_confirmed_{false};
  std::atomic<bool> protection_lost_{false};
  std::atomic<bool> runtime_fault_reported_{false};
  std::atomic<bool> detector_start_loop_running_{false};
  std::atomic<bool> detector_started_;
  std::atomic<bool> startup_ready_{false};
  std::atomic<bool> shutting_down_;
};

}  // namespace

int main(int argc, char **argv) {
  // 常见终止信号统一走受控退出，避免 supervisor 停止时跳过清理流程。
  std::signal(SIGINT, ShutdownSignalHandler);
  std::signal(SIGTERM, ShutdownSignalHandler);

  // 主循环采用“可重建节点”的写法：
  // 如果节点在运行中抛出异常，会尝试播报异常并重新拉起；
  // 只有短时间内连续多次异常，才判定为不可恢复错误并退出进程。
  int exit_code = 0;
  int short_lived_failure_count = 0;
  while (true) {
    std::shared_ptr<unitree::ros2::g1::AudioClient> audio_node;
    const auto loop_started_at = std::chrono::steady_clock::now();

    try {
      if (!rclcpp::ok()) {
        rclcpp::InitOptions init_options;
        init_options.shutdown_on_sigint = false;
        rclcpp::init(argc, argv, init_options);
      }

      // AudioClient 也是 Node，需要与主节点一起加入同一个 executor。
      auto estop_node = std::make_shared<G1EmergencyStopNode>();
      audio_node = estop_node->GetAudioClientNode();

      // executor 主要负责消费各类 RPC 响应；线程数留足余量，避免后续扩展时再次饿死回调。
      rclcpp::executors::MultiThreadedExecutor executor(
          rclcpp::ExecutorOptions(), kExecutorThreadCount);
      executor.add_node(estop_node);
      executor.add_node(audio_node);
      executor.spin();

      exit_code = std::max(exit_code, RequestedProcessExitCode());
      short_lived_failure_count = 0;
      if (!rclcpp::ok()) {
        break;
      }
    } catch (const std::exception &e) {
      // 主循环级别异常说明节点已无法继续当前轮次运行。
      // 此时先尝试对外提示，再根据“是否短时间内连续失败”决定要不要重启。
      TryAnnounceWithTemporaryExecutor(audio_node, kUnhandledExceptionText);
      std::cerr << "Fatal error in g1_emergency_stop_node: " << e.what()
                << std::endl;
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }

      const auto loop_runtime = std::chrono::steady_clock::now() - loop_started_at;
      if (loop_runtime > kMainLoopFailureBurstWindow) {
        short_lived_failure_count = 0;
      }
      ++short_lived_failure_count;
      if (short_lived_failure_count >= kMainLoopMaxBurstFailures) {
        std::cerr << "Fatal error in g1_emergency_stop_node: exceeded "
                  << kMainLoopMaxBurstFailures
                  << " consecutive short-lived failures, exiting."
                  << std::endl;
        RequestProcessExitCode(1);
        exit_code = std::max(exit_code, RequestedProcessExitCode());
        break;
      }

      std::this_thread::sleep_for(kDetectorRetryInterval);
      continue;
    }
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  exit_code = std::max(exit_code, RequestedProcessExitCode());
  return exit_code;
}
