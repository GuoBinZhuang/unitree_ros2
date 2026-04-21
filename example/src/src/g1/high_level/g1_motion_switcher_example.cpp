#include <chrono>
#include <cstdint>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <utility>

#include "common/ut_errror.hpp"
#include "g1/g1_motion_switch_client.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

enum class MotionCommand {
  kCheck,
  kRelease,
  kSelect,
};

struct CommandLineOptions {
  MotionCommand command = MotionCommand::kCheck;
  std::string mode_name;
};

class G1MotionSwitcherExample : public rclcpp::Node {
 public:
  explicit G1MotionSwitcherExample(CommandLineOptions options)
      : Node("g1_motion_switcher_example"),
        options_(std::move(options)),
        client_(this) {
    worker_ = std::thread([this]() {
      std::this_thread::sleep_for(500ms);
      ExecuteCommand();
      rclcpp::shutdown();
    });
  }

  ~G1MotionSwitcherExample() override {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  bool HandleResult(const std::string &action_name, int32_t result) {
    if (result == UT_ROBOT_SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "%s 调用成功", action_name.c_str());
      return true;
    }

    static const std::map<int32_t, std::string> kErrorDescriptions = {
        {7001, "请求参数错误"},
        {7002, "切换服务繁忙，请稍后再试"},
        {7004, "运控模式名不支持"},
        {7005, "运控模式内部指令执行错误"},
        {7006, "运控模式检测指令执行错误"},
        {7007, "运控模式切换指令执行错误"},
        {7008, "运控模式释放指令执行错误"},
        {7009, "自定义配置错误"},
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

  void PrintCurrentMode() {
    std::string form;
    std::string name;
    const int32_t result = client_.CheckMode(form, name);
    if (!HandleResult("CheckMode", result)) {
      return;
    }

    if (name.empty()) {
      RCLCPP_INFO(this->get_logger(), "当前没有激活的运控模式，机器人处于用户调试模式");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "当前运控模式: form=%s, name=%s",
                form.c_str(), name.c_str());
  }

  void ExecuteCommand() {
    switch (options_.command) {
      case MotionCommand::kCheck:
        PrintCurrentMode();
        break;
      case MotionCommand::kRelease: {
        const int32_t result = client_.ReleaseMode();
        if (HandleResult("ReleaseMode", result)) {
          PrintCurrentMode();
        }
        break;
      }
      case MotionCommand::kSelect: {
        const int32_t result = client_.SelectMode(options_.mode_name);
        if (HandleResult("SelectMode", result)) {
          PrintCurrentMode();
        }
        break;
      }
    }
  }

  CommandLineOptions options_;
  unitree::robot::g1::MotionSwitchClient client_;
  std::thread worker_;
};

void PrintUsage(const char *program_name) {
  std::cout << "用法:\n"
            << "  " << program_name << " --check\n"
            << "  " << program_name << " --release\n"
            << "  " << program_name << " --select=<mode_name>\n\n"
            << "示例:\n"
            << "  " << program_name << " --check\n"
            << "  " << program_name << " --release\n"
            << "  " << program_name << " --select=ai\n";
}

bool ParseArguments(int argc, char const *argv[], CommandLineOptions &options) {
  int command_count = 0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--check") {
      options.command = MotionCommand::kCheck;
      ++command_count;
      continue;
    }

    if (arg == "--release") {
      options.command = MotionCommand::kRelease;
      ++command_count;
      continue;
    }

    const std::string prefix = "--select=";
    if (arg.substr(0, prefix.size()) == prefix) {
      options.command = MotionCommand::kSelect;
      options.mode_name = arg.substr(prefix.size());
      ++command_count;
      continue;
    }

    std::cerr << "未知参数: " << arg << std::endl;
    return false;
  }

  if (command_count != 1) {
    std::cerr << "必须且只能指定一个操作参数" << std::endl;
    return false;
  }

  if (options.command == MotionCommand::kSelect && options.mode_name.empty()) {
    std::cerr << "--select 需要提供运控模式名，例如 --select=ai"
              << std::endl;
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
  auto node = std::make_shared<G1MotionSwitcherExample>(options);
  rclcpp::spin(node);
  return 0;
}
