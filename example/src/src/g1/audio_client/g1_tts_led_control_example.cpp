#include <array>
#include <chrono>
#include <cstdint>
#include <future>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "g1/g1_audio_client.hpp"

namespace {

void PrintUsage(const char *program_name) {
  std::cout << "Usage:\n"
            << "  " << program_name << " \"text to speak\"\n"
            << "  " << program_name << " \"text to speak\" R G B\n"
            << "\n"
            << "Examples:\n"
            << "  " << program_name << " \"你好，欢迎使用 G1\"\n"
            << "  " << program_name << " \"任务开始执行\" 255 0 0\n"
            << "\n"
            << "When no arguments are provided, the program enters "
               "interactive mode.\n";
}

bool ParseColorChannel(const std::string &value, uint8_t &channel,
                       const char *channel_name) {
  try {
    size_t parsed_length = 0;
    const int parsed_value = std::stoi(value, &parsed_length);
    if (parsed_length != value.size() || parsed_value < 0 || parsed_value > 255) {
      std::cerr << "Invalid " << channel_name
                << " value. Expected an integer between 0 and 255.\n";
      return false;
    }
    channel = static_cast<uint8_t>(parsed_value);
    return true;
  } catch (const std::exception &) {
    std::cerr << "Invalid " << channel_name
              << " value. Expected an integer between 0 and 255.\n";
    return false;
  }
}

bool ReadInteractiveInput(std::string &text,
                          std::optional<std::array<uint8_t, 3>> &rgb) {
  std::cout << "Input text to speak: ";
  std::getline(std::cin, text);
  if (text.empty()) {
    std::cerr << "Text must not be empty.\n";
    return false;
  }

  std::cout << "Set RGB LED now? (y/N): ";
  std::string answer;
  std::getline(std::cin, answer);
  if (answer != "y" && answer != "Y") {
    rgb.reset();
    return true;
  }

  std::array<uint8_t, 3> color{};
  const std::array<const char *, 3> names = {"R", "G", "B"};
  for (size_t i = 0; i < names.size(); ++i) {
    std::cout << "Input " << names[i] << " (0-255): ";
    std::string value;
    std::getline(std::cin, value);
    if (!ParseColorChannel(value, color[i], names[i])) {
      return false;
    }
  }

  rgb = color;
  return true;
}

bool ParseArguments(int argc, char **argv, std::string &text,
                    std::optional<std::array<uint8_t, 3>> &rgb) {
  if (argc == 1) {
    return ReadInteractiveInput(text, rgb);
  }

  if (argc != 2 && argc != 5) {
    PrintUsage(argv[0]);
    return false;
  }

  text = argv[1];
  if (text.empty()) {
    std::cerr << "Text must not be empty.\n";
    return false;
  }

  if (argc == 5) {
    std::array<uint8_t, 3> color{};
    if (!ParseColorChannel(argv[2], color[0], "R") ||
        !ParseColorChannel(argv[3], color[1], "G") ||
        !ParseColorChannel(argv[4], color[2], "B")) {
      return false;
    }
    rgb = color;
  } else {
    rgb.reset();
  }

  return true;
}

int RunExample(const std::shared_ptr<unitree::ros2::g1::AudioClient> &client,
               const std::string &text,
               const std::optional<std::array<uint8_t, 3>> &rgb) {
  const int32_t tts_ret = client->TtsMaker(text, 0);
  std::cout << "TtsMaker ret: " << tts_ret << std::endl;
  if (tts_ret != 0) {
    return tts_ret;
  }

  if (!rgb.has_value()) {
    return 0;
  }

  const auto &[r, g, b] = *rgb;
  const int32_t led_ret = client->LedControl(r, g, b);
  std::cout << "LedControl ret: " << led_ret << "  rgb=("
            << static_cast<int>(r) << ", " << static_cast<int>(g) << ", "
            << static_cast<int>(b) << ")" << std::endl;
  return led_ret;
}

}  // namespace

int main(int argc, char **argv) {
  std::string text;
  std::optional<std::array<uint8_t, 3>> rgb;
  if (!ParseArguments(argc, argv, text, rgb)) {
    return 1;
  }

  try {
    rclcpp::init(argc, argv);
    auto client = std::make_shared<unitree::ros2::g1::AudioClient>();
    std::promise<int> result_promise;
    auto result_future = result_promise.get_future();

    std::thread worker([client, text, rgb, &result_promise]() mutable {
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(1s);
      result_promise.set_value(RunExample(client, text, rgb));
      rclcpp::shutdown();
    });

    rclcpp::spin(client);
    worker.join();
    return result_future.get();
  } catch (const rclcpp::exceptions::RCLError &e) {
    std::cerr << "RCLError caught: " << e.what() << std::endl;
    return 1;
  }
}
