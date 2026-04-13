#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "common/ut_errror.hpp"
#include "g1/g1_loco_client.hpp"
#include "g1/g1_motion_switch_client.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

enum class LocoCommand {
  kGetFsmId,
  kGetFsmMode,
  kDamp,
  kStart,
  kSquat,
  kSit,
  kStandUp,
  kZeroTorque,
  kBalanceStand,
  kContinuousGait,
  kSwitchMoveMode,
  kMove,
  kSetVelocity,
  kStopMove,
  kSetSpeedMode,
};

struct CommandLineOptions {
  LocoCommand command = LocoCommand::kGetFsmId;
  std::vector<float> float_values;
  bool bool_value = false;
  int int_value = 0;
};

class G1LocoServiceExample : public rclcpp::Node {
 public:
  explicit G1LocoServiceExample(CommandLineOptions options)
      : Node("g1_loco_service_example"),
        options_(std::move(options)),
        loco_client_(this),
        motion_switch_client_(this) {
    worker_ = std::thread([this]() {
      std::this_thread::sleep_for(500ms);
      ExecuteCommand();
      rclcpp::shutdown();
    });
  }

  ~G1LocoServiceExample() override {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  bool HandleLocoResult(const std::string &action_name, int32_t result) {
    if (result == UT_ROBOT_SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "%s 调用成功", action_name.c_str());
      return true;
    }

    static const std::map<int32_t, std::string> kErrorDescriptions = {
        {unitree::robot::g1::UT_ROBOT_LOCO_ERR_LOCOSTATE_NOT_AVAILABLE,
         "高层运动状态不可用"},
        {unitree::robot::g1::UT_ROBOT_LOCO_ERR_INVALID_FSM_ID,
         "无效的运控模式 ID"},
        {unitree::robot::g1::UT_ROBOT_LOCO_ERR_INVALID_TASK_ID,
         "无效的任务 ID"},
        {UT_ROBOT_TASK_TIMEOUT, "RPC 调用超时"},
        {UT_ROBOT_TASK_UNKNOWN_ERROR, "RPC 调用未知错误"},
    };

    auto it = kErrorDescriptions.find(result);
    const std::string description =
        it == kErrorDescriptions.end() ? "未定义错误" : it->second;

    RCLCPP_ERROR(this->get_logger(), "%s 调用失败，错误码: %d，描述: %s",
                 action_name.c_str(), result, description.c_str());
    return false;
  }

  bool CheckMotionServiceReady() {
    std::string form;
    std::string name;
    const int32_t result = motion_switch_client_.CheckMode(form, name);
    if (result != UT_ROBOT_SUCCESS) {
      RCLCPP_WARN(this->get_logger(),
                  "无法通过运控切换服务确认当前模式，后续将直接尝试调用高层运动服务，错误码: %d",
                  result);
      return true;
    }

    if (name.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "当前机器人处于用户调试模式，高层运动服务依赖内置运控，无法使用。请先切回 ai 等内置运控模式。"
      );
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "当前运控模式: form=%s, name=%s",
                form.c_str(), name.c_str());
    return true;
  }

  void ExecuteCommand() {
    if (!CheckMotionServiceReady()) {
      return;
    }

    switch (options_.command) {
      case LocoCommand::kGetFsmId:
        ExecuteGetFsmId();
        break;
      case LocoCommand::kGetFsmMode:
        ExecuteGetFsmMode();
        break;
      case LocoCommand::kDamp:
        ExecuteSimpleCommand("Damp", [this]() { return loco_client_.Damp(); });
        break;
      case LocoCommand::kStart:
        ExecuteSimpleCommand("Start",
                             [this]() { return loco_client_.Start(); });
        break;
      case LocoCommand::kSquat:
        ExecuteSimpleCommand("Squat",
                             [this]() { return loco_client_.Squat(); });
        break;
      case LocoCommand::kSit:
        ExecuteSimpleCommand("Sit", [this]() { return loco_client_.Sit(); });
        break;
      case LocoCommand::kStandUp:
        ExecuteSimpleCommand("StandUp",
                             [this]() { return loco_client_.StandUp(); });
        break;
      case LocoCommand::kZeroTorque:
        ExecuteSimpleCommand("ZeroTorque",
                             [this]() { return loco_client_.ZeroTorque(); });
        break;
      case LocoCommand::kBalanceStand:
        ExecuteSimpleCommand("BalanceStand",
                             [this]() { return loco_client_.BalanceStand(); });
        break;
      case LocoCommand::kContinuousGait:
        ExecuteSimpleCommand(
            "ContinuousGait",
            [this]() { return loco_client_.ContinuousGait(options_.bool_value); });
        break;
      case LocoCommand::kSwitchMoveMode:
        ExecuteSimpleCommand(
            "SwitchMoveMode",
            [this]() { return loco_client_.SwitchMoveMode(options_.bool_value); });
        break;
      case LocoCommand::kMove:
        ExecuteMove();
        break;
      case LocoCommand::kSetVelocity:
        ExecuteSetVelocity();
        break;
      case LocoCommand::kStopMove:
        ExecuteSimpleCommand("StopMove",
                             [this]() { return loco_client_.StopMove(); });
        break;
      case LocoCommand::kSetSpeedMode:
        ExecuteSimpleCommand(
            "SetSpeedMode",
            [this]() { return loco_client_.SetSpeedMode(options_.int_value); });
        break;
    }
  }

  template <typename Func>
  void ExecuteSimpleCommand(const std::string &action_name, Func func) {
    const int32_t result = func();
    HandleLocoResult(action_name, result);
  }

  void ExecuteGetFsmId() {
    int fsm_id = -1;
    const int32_t result = loco_client_.GetFsmId(fsm_id);
    if (!HandleLocoResult("GetFsmId", result)) {
      return;
    }
    RCLCPP_INFO(this->get_logger(), "当前 FSM ID: %d", fsm_id);
  }

  void ExecuteGetFsmMode() {
    int fsm_mode = -1;
    const int32_t result = loco_client_.GetFsmMode(fsm_mode);
    if (!HandleLocoResult("GetFsmMode", result)) {
      return;
    }
    RCLCPP_INFO(this->get_logger(), "当前 FSM 模式: %d (%s)", fsm_mode,
                fsm_mode == 0 ? "站立状态" : "移动状态");
  }

  void ExecuteMove() {
    const int32_t result = loco_client_.Move(options_.float_values.at(0),
                                             options_.float_values.at(1),
                                             options_.float_values.at(2));
    if (!HandleLocoResult("Move", result)) {
      return;
    }
    RCLCPP_INFO(this->get_logger(), "速度指令已发送: vx=%f, vy=%f, vyaw=%f",
                options_.float_values.at(0), options_.float_values.at(1),
                options_.float_values.at(2));
  }

  void ExecuteSetVelocity() {
    const float duration = options_.float_values.size() == 4
                               ? options_.float_values.at(3)
                               : 1.0F;
    const int32_t result =
        loco_client_.SetVelocity(options_.float_values.at(0),
                                 options_.float_values.at(1),
                                 options_.float_values.at(2), duration);
    if (!HandleLocoResult("SetVelocity", result)) {
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "速度已设置: vx=%f, vy=%f, omega=%f, duration=%f",
                options_.float_values.at(0), options_.float_values.at(1),
                options_.float_values.at(2), duration);
  }

  CommandLineOptions options_;
  unitree::robot::g1::LocoClient loco_client_;
  unitree::robot::g1::MotionSwitchClient motion_switch_client_;
  std::thread worker_;
};

std::vector<float> ParseFloatList(const std::string &value) {
  std::stringstream ss(value);
  std::vector<float> values;
  float number = NAN;
  while (ss >> number) {
    values.push_back(number);
    if (ss.peek() == ',' || ss.peek() == ' ') {
      ss.ignore();
    }
  }
  return values;
}

bool ParseBoolValue(const std::string &value, bool &result) {
  if (value == "true") {
    result = true;
    return true;
  }
  if (value == "false") {
    result = false;
    return true;
  }
  return false;
}

void PrintUsage(const char *program_name) {
  std::cout
      << "用法:\n"
      << "  " << program_name << " --get_fsm_id\n"
      << "  " << program_name << " --get_fsm_mode\n"
      << "  " << program_name << " --damp\n"
      << "  " << program_name << " --start\n"
      << "  " << program_name << " --squat\n"
      << "  " << program_name << " --sit\n"
      << "  " << program_name << " --stand_up\n"
      << "  " << program_name << " --zero_torque\n"
      << "  " << program_name << " --balance_stand\n"
      << "  " << program_name << " --continuous_gait=<true|false>\n"
      << "  " << program_name << " --switch_move_mode=<true|false>\n"
      << "  " << program_name << " --move=\"vx vy vyaw\"\n"
      << "  " << program_name << " --set_velocity=\"vx vy omega [duration]\"\n"
      << "  " << program_name << " --stop_move\n"
      << "  " << program_name << " --set_speed_mode=<0|1|2|3>\n\n"
      << "示例:\n"
      << "  " << program_name << " --start\n"
      << "  " << program_name << " --move=\"0.2 0.0 0.0\"\n"
      << "  " << program_name << " --set_velocity=\"0.2 0.0 0.0 2.0\"\n"
      << "  " << program_name << " --continuous_gait=true\n"
      << "  " << program_name << " --set_speed_mode=1\n";
}

bool ParseArguments(int argc, char const *argv[], CommandLineOptions &options) {
  int command_count = 0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto consume_no_value = [&](const std::string &expected,
                                      LocoCommand command) -> bool {
      if (arg == expected) {
        options.command = command;
        ++command_count;
        return true;
      }
      return false;
    };

    if (consume_no_value("--get_fsm_id", LocoCommand::kGetFsmId) ||
        consume_no_value("--get_fsm_mode", LocoCommand::kGetFsmMode) ||
        consume_no_value("--damp", LocoCommand::kDamp) ||
        consume_no_value("--start", LocoCommand::kStart) ||
        consume_no_value("--squat", LocoCommand::kSquat) ||
        consume_no_value("--sit", LocoCommand::kSit) ||
        consume_no_value("--stand_up", LocoCommand::kStandUp) ||
        consume_no_value("--zero_torque", LocoCommand::kZeroTorque) ||
        consume_no_value("--balance_stand", LocoCommand::kBalanceStand) ||
        consume_no_value("--stop_move", LocoCommand::kStopMove)) {
      continue;
    }

    const auto parse_with_value = [&](const std::string &prefix,
                                      std::string &value) -> bool {
      if (arg.substr(0, prefix.size()) == prefix) {
        value = arg.substr(prefix.size());
        return true;
      }
      return false;
    };

    std::string value;
    if (parse_with_value("--continuous_gait=", value)) {
      options.command = LocoCommand::kContinuousGait;
      ++command_count;
      if (!ParseBoolValue(value, options.bool_value)) {
        std::cerr << "--continuous_gait 只能是 true 或 false" << std::endl;
        return false;
      }
      continue;
    }

    if (parse_with_value("--switch_move_mode=", value)) {
      options.command = LocoCommand::kSwitchMoveMode;
      ++command_count;
      if (!ParseBoolValue(value, options.bool_value)) {
        std::cerr << "--switch_move_mode 只能是 true 或 false" << std::endl;
        return false;
      }
      continue;
    }

    if (parse_with_value("--move=", value)) {
      options.command = LocoCommand::kMove;
      options.float_values = ParseFloatList(value);
      ++command_count;
      if (options.float_values.size() != 3) {
        std::cerr << "--move 需要 3 个浮点数参数: vx vy vyaw" << std::endl;
        return false;
      }
      continue;
    }

    if (parse_with_value("--set_velocity=", value)) {
      options.command = LocoCommand::kSetVelocity;
      options.float_values = ParseFloatList(value);
      ++command_count;
      if (options.float_values.size() != 3 &&
          options.float_values.size() != 4) {
        std::cerr
            << "--set_velocity 需要 3 或 4 个浮点数参数: vx vy omega [duration]"
            << std::endl;
        return false;
      }
      continue;
    }

    if (parse_with_value("--set_speed_mode=", value)) {
      options.command = LocoCommand::kSetSpeedMode;
      ++command_count;
      try {
        options.int_value = std::stoi(value);
      } catch (const std::exception &) {
        std::cerr << "--set_speed_mode 需要整数参数" << std::endl;
        return false;
      }
      if (options.int_value < 0 || options.int_value > 3) {
        std::cerr << "--set_speed_mode 只能取 0 到 3" << std::endl;
        return false;
      }
      continue;
    }

    std::cerr << "未知参数: " << arg << std::endl;
    return false;
  }

  if (command_count != 1) {
    std::cerr << "必须且只能指定一个操作参数" << std::endl;
    return false;
  }

  return true;
}

int main(int argc, char const *argv[]) {
  CommandLineOptions options;
  if (!ParseArguments(argc, argv, options)) {
    PrintUsage(argv[0]);
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<G1LocoServiceExample>(options);
  rclcpp::spin(node);
  return 0;
}
