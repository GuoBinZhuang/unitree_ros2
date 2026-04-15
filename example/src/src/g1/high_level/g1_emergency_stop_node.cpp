#include <atomic>
#include <chrono>
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
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../GPIO/gpio_estop.hpp"

namespace {

// G1 阻尼模式 FSM ID。
constexpr int kDampFsmId = 1;

struct LedColor {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

enum class NotifyType {
  kSuccess,
  kFailure,
};

class G1EmergencyStopNode final : public rclcpp::Node {
 public:
  G1EmergencyStopNode()
      : rclcpp::Node("g1_emergency_stop_node"),
        debounce_ms_(declare_parameter<int>("debounce_ms", 20)),
        active_low_(declare_parameter<bool>("active_low", false)),
        max_retries_(declare_parameter<int>("max_retries", 2)),
        retry_backoff_ms_(declare_parameter<int>("retry_backoff_ms", 100)),
        notification_cooldown_ms_(
            declare_parameter<int>("notification_cooldown_ms", 2000)),
        speaker_id_(declare_parameter<int>("speaker_id", 0)),
        success_text_(declare_parameter<std::string>("success_text", "急停触发，已切换阻尼模式")),
        failure_text_(declare_parameter<std::string>("failure_text", "急停触发，切换阻尼模式失败，请立即检查")),
        success_led_({
          ClampColor(declare_parameter<int>("success_led_r", 255)),
          ClampColor(declare_parameter<int>("success_led_g", 180)),
          ClampColor(declare_parameter<int>("success_led_b", 0)),
        }),
        failure_led_({
          ClampColor(declare_parameter<int>("failure_led_r", 255)),
          ClampColor(declare_parameter<int>("failure_led_g", 0)),
          ClampColor(declare_parameter<int>("failure_led_b", 0)),
        }),
        detector_(debounce_ms_, active_low_),
        loco_client_(this),
        motion_switch_client_(this),
        audio_client_(std::make_shared<unitree::ros2::g1::AudioClient>()),
        last_notify_type_(NotifyType::kFailure),
        estop_active_(false),
        detector_started_(false),
        shutting_down_(false) {
    // GPIO 仅负责产生按下/释放事件，具体动作在回调内串联执行。
    detector_.onPressed([this]() { OnPressedEdge(); });
    detector_.onReleased([this]() { OnReleasedEdge(); });

    // 给底层通信一点初始化时间，再启动 GPIO 检测器。
    startup_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
      startup_timer_->cancel();
      StartDetector();
    });
    // 周期巡检检测器线程健康，避免静默故障。
    health_timer_ = create_wall_timer(std::chrono::milliseconds(250),
                                      [this]() { MonitorDetectorHealth(); });

    RCLCPP_INFO(get_logger(),
                "急停节点初始化完成: debounce_ms=%d active_low=%s retries=%d backoff_ms=%d",
                debounce_ms_, active_low_ ? "true" : "false", max_retries_, retry_backoff_ms_);
  }

  ~G1EmergencyStopNode() override {
    // 先置位关机标记，阻止回调触发新动作，再停止检测器。
    shutting_down_.store(true, std::memory_order_release);
    StopDetector();
  }

  std::shared_ptr<unitree::ros2::g1::AudioClient> GetAudioClientNode() const {
    return audio_client_;
  }

 private:
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
    const auto it = error_map.find(code);
    if (it == error_map.end()) {
      return "未定义错误";
    }
    return it->second;
  }

  static uint8_t ClampColor(int value) {
    if (value < 0) {
      return 0;
    }
    if (value > 255) {
      return 255;
    }
    return static_cast<uint8_t>(value);
  }

  void StartDetector() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (detector_started_.load(std::memory_order_acquire)) {
      return;
    }

    runtime_fault_reported_.store(false, std::memory_order_release);

    try {
      detector_.start();

      // 启动后立即做一次自检，不通过则主动退出，避免无保护运行。
      std::string report;
      if (!detector_.startupSelfCheck(report)) {
        RCLCPP_ERROR(get_logger(), "GPIO 急停自检失败: %s", report.c_str());
        detector_.stop();
        rclcpp::shutdown();
        return;
      }

      detector_started_.store(true, std::memory_order_release);
      estop_active_.store(detector_.isActive(), std::memory_order_release);
      RCLCPP_INFO(get_logger(), "GPIO 急停检测已启动: %s", report.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "启动 GPIO 急停检测失败: %s", e.what());
      rclcpp::shutdown();
    }
  }

  void StopDetector() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (!detector_started_.load(std::memory_order_acquire)) {
      return;
    }

    detector_.stop();
    detector_started_.store(false, std::memory_order_release);
    RCLCPP_INFO(get_logger(), "GPIO 急停检测已停止");
  }

  void MonitorDetectorHealth() {
    if (!detector_started_.load(std::memory_order_acquire) ||
        shutting_down_.load(std::memory_order_acquire)) {
      return;
    }

    std::string runtime_error;
    if (!detector_.getRuntimeError(runtime_error)) {
      return;
    }

    bool expected = false;
    // 只上报一次运行时故障，避免故障期间重复刷屏和重复播报。
    if (!runtime_fault_reported_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) {
      return;
    }

    RCLCPP_ERROR(get_logger(), "GPIO 急停检测线程故障: %s", runtime_error.c_str());
    NotifyWithTtsLed(NotifyType::kFailure);
  }

  void OnPressedEdge() {
    if (shutting_down_.load(std::memory_order_acquire)) {
      return;
    }

    bool expected = false;
    // 仅在“非急停 -> 急停”按下沿触发一次动作，按住不重复触发。
    if (!estop_active_.compare_exchange_strong(expected, true,
                                               std::memory_order_acq_rel)) {
      return;
    }

    std::lock_guard<std::mutex> lock(action_mutex_);
    RCLCPP_WARN(get_logger(), "检测到急停按下沿，开始切换阻尼模式");

    const bool switched = SwitchToDampWithRetry();
    if (switched) {
      NotifyWithTtsLed(NotifyType::kSuccess);
      RCLCPP_INFO(get_logger(), "急停处理完成：机器人已进入阻尼模式");
      return;
    }

    NotifyWithTtsLed(NotifyType::kFailure);
    RCLCPP_ERROR(get_logger(), "急停处理失败：未能切换至阻尼模式");
  }

  void OnReleasedEdge() {
    const bool was_active = estop_active_.exchange(false, std::memory_order_acq_rel);
    if (!was_active) {
      return;
    }

    RCLCPP_INFO(get_logger(), "检测到急停释放沿：保持阻尼模式，不自动恢复");
  }

  // 急停主动作：尝试切阻尼，失败按参数重试，并在每次成功后做 FSM 校验。
  bool SwitchToDampWithRetry() {
    CheckMotionMode();

    for (int attempt = 0; attempt <= max_retries_; ++attempt) {
      const int32_t damp_ret = loco_client_.Damp();
      if (damp_ret == UT_ROBOT_SUCCESS) {
        if (VerifyDampFsm()) {
          return true;
        }

        RCLCPP_WARN(get_logger(),
                    "Damp 调用返回成功，但 FSM 校验未通过 (attempt=%d/%d)",
                    attempt + 1, max_retries_ + 1);
      } else {
        RCLCPP_ERROR(
            get_logger(),
            "Damp 调用失败 (attempt=%d/%d), code=%d, desc=%s",
            attempt + 1, max_retries_ + 1, damp_ret,
            ResolveErrorDescription(damp_ret, BuildLocoErrorMap()).c_str());
      }

      if (attempt == max_retries_) {
        break;
      }

      // 简单固定退避，降低服务繁忙时的连续冲击。
      std::this_thread::sleep_for(std::chrono::milliseconds(retry_backoff_ms_));
    }

    return false;
  }

  void CheckMotionMode() {
    std::string form;
    std::string name;
    const int32_t ret = motion_switch_client_.CheckMode(form, name);
    if (ret != UT_ROBOT_SUCCESS) {
      RCLCPP_WARN(get_logger(), "无法确认当前运控模式，继续尝试 Damp, code=%d, desc=%s",
                  ret,
                  ResolveErrorDescription(ret, BuildMotionErrorMap()).c_str());
      return;
    }

    if (name.empty()) {
      RCLCPP_WARN(get_logger(),
                  "当前机器人处于用户调试模式，高层服务可能不可用，仍将尝试 Damp");
      return;
    }

    RCLCPP_INFO(get_logger(), "当前运控模式: form=%s, name=%s", form.c_str(),
                name.c_str());
  }

  bool VerifyDampFsm() {
    int fsm_id = -1;
    const int32_t fsm_ret = loco_client_.GetFsmId(fsm_id);
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

  void NotifyWithTtsLed(NotifyType type) {
    if (ShouldSkipNotification(type)) {
      RCLCPP_WARN(get_logger(), "声光提示被节流，跳过本次通知");
      return;
    }

    // 同一入口统一处理成功/失败文案和颜色。
    const std::string &text = (type == NotifyType::kSuccess) ? success_text_ : failure_text_;
    const LedColor color = (type == NotifyType::kSuccess) ? success_led_ : failure_led_;

    const int32_t tts_ret = audio_client_->TtsMaker(text, speaker_id_);
    if (tts_ret != UT_ROBOT_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "TtsMaker 失败, code=%d", tts_ret);
    }

    const int32_t led_ret = audio_client_->LedControl(color.r, color.g, color.b);
    if (led_ret != UT_ROBOT_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "LedControl 失败, code=%d", led_ret);
    }

    if (tts_ret == UT_ROBOT_SUCCESS && led_ret == UT_ROBOT_SUCCESS) {
      RCLCPP_INFO(get_logger(), "声光提示成功: text=\"%s\" rgb=(%u,%u,%u)", text.c_str(),
                  static_cast<unsigned>(color.r), static_cast<unsigned>(color.g),
                  static_cast<unsigned>(color.b));
    }

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

    // 同类型通知在冷却时间内丢弃，避免短时重复播报。
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_notify_at_)
            .count();
    return elapsed_ms < notification_cooldown_ms_;
  }

  const int debounce_ms_;
  const bool active_low_;
  const int max_retries_;
  const int retry_backoff_ms_;
  const int notification_cooldown_ms_;
  const int speaker_id_;
  const std::string success_text_;
  const std::string failure_text_;
  const LedColor success_led_;
  const LedColor failure_led_;

  EStopDetector detector_;
  unitree::robot::g1::LocoClient loco_client_;
  unitree::robot::g1::MotionSwitchClient motion_switch_client_;
  std::shared_ptr<unitree::ros2::g1::AudioClient> audio_client_;

  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  mutable std::mutex lifecycle_mutex_;
  mutable std::mutex action_mutex_;
  mutable std::mutex notify_mutex_;

  std::chrono::steady_clock::time_point last_notify_at_{};
  NotifyType last_notify_type_;

  std::atomic<bool> estop_active_;
  std::atomic<bool> runtime_fault_reported_{false};
  std::atomic<bool> detector_started_;
  std::atomic<bool> shutting_down_;
};

}  // namespace

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);

    // AudioClient 也是 Node，需要与主节点一起加入同一个 executor。
    auto estop_node = std::make_shared<G1EmergencyStopNode>();
    auto audio_node = estop_node->GetAudioClientNode();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(estop_node);
    executor.add_node(audio_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Fatal error in g1_emergency_stop_node: " << e.what()
              << std::endl;
    return 1;
  }
}
