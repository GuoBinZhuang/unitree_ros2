#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "inspire_hand_state.hpp"

using namespace std::chrono_literals;

namespace {

struct JointBinding {
  size_t state_index;
  const char* joint_name;
  double default_lower_limit;
  double default_upper_limit;
  bool invert;
};

struct JointCalibration {
  double lower_limit;
  double upper_limit;
  double offset;
};

struct HandSnapshot {
  std::array<int16_t, 6> values{};
};

struct UrdfJointLimit {
  double lower_limit;
  double upper_limit;
};

enum class StateField {
  kAuto,
  kAngleAct,
  kPosAct,
};

constexpr std::array<JointBinding, 6> kLeftBindings = {{
    {0, "left_little_1_joint", 0.0, 1.4381, false},
    {1, "left_ring_1_joint", 0.0, 1.4381, false},
    {2, "left_middle_1_joint", 0.0, 1.4381, false},
    {3, "left_index_1_joint", 0.0, 1.4381, false},
    {4, "left_thumb_2_joint", 0.0, 0.5864, true},
    {5, "left_thumb_1_joint", 0.0, 1.1641, true},
}};

constexpr std::array<JointBinding, 6> kRightBindings = {{
    {0, "right_little_1_joint", 0.0, 1.4381, false},
    {1, "right_ring_1_joint", 0.0, 1.4381, false},
    {2, "right_middle_1_joint", 0.0, 1.4381, false},
    {3, "right_index_1_joint", 0.0, 1.4381, false},
    {4, "right_thumb_2_joint", 0.0, 0.5864, true},
    {5, "right_thumb_1_joint", 0.0, 1.1641, true},
}};

bool IsDynamicJointType(const std::string& joint_type) {
  return joint_type == "revolute" || joint_type == "continuous" ||
         joint_type == "prismatic";
}

StateField ParseStateField(const std::string& state_field) {
  if (state_field == "auto") {
    return StateField::kAuto;
  }
  if (state_field == "angle_act") {
    return StateField::kAngleAct;
  }
  if (state_field == "pos_act") {
    return StateField::kPosAct;
  }

  throw std::runtime_error(
      "Parameter 'state_field' must be one of: auto, angle_act, pos_act");
}

std::array<double, 12> ParseJointOffsets(const std::vector<double>& offsets) {
  std::array<double, 12> parsed_offsets{};
  const auto count = std::min(parsed_offsets.size(), offsets.size());
  for (size_t index = 0; index < count; ++index) {
    parsed_offsets[index] = offsets[index];
  }
  return parsed_offsets;
}

std::unordered_map<std::string, UrdfJointLimit> LoadJointLimitsFromUrdf(
    const std::string& urdf_path) {
  std::ifstream urdf_stream(urdf_path);
  if (!urdf_stream.is_open()) {
    throw std::runtime_error("Failed to open URDF: " + urdf_path);
  }

  const std::regex joint_start_regex(
      R"joint(<joint\s+name="([^"]+)"\s+type="([^"]+)")joint");
  const std::regex limit_regex(
      R"joint(<limit[^>]*lower="([^"]+)"[^>]*upper="([^"]+)")joint");
  std::unordered_map<std::string, UrdfJointLimit> joint_limits;
  std::string line;
  bool in_joint = false;
  std::string current_joint_name;
  bool current_joint_movable = false;
  std::optional<UrdfJointLimit> current_joint_limit;

  while (std::getline(urdf_stream, line)) {
    if (!in_joint) {
      std::smatch match;
      if (!std::regex_search(line, match, joint_start_regex)) {
        continue;
      }

      current_joint_name = match[1].str();
      current_joint_movable = IsDynamicJointType(match[2].str());
      current_joint_limit.reset();
      in_joint = true;
    }

    std::smatch limit_match;
    if (current_joint_movable &&
        std::regex_search(line, limit_match, limit_regex)) {
      current_joint_limit = UrdfJointLimit{
          .lower_limit = std::stod(limit_match[1].str()),
          .upper_limit = std::stod(limit_match[2].str()),
      };
    }

    if (line.find("</joint>") == std::string::npos) {
      continue;
    }

    if (current_joint_movable && current_joint_limit.has_value()) {
      joint_limits.emplace(current_joint_name, *current_joint_limit);
    }

    in_joint = false;
  }

  return joint_limits;
}

double ScaleHandValue(int16_t raw_value, const JointCalibration& calibration,
                      bool invert, bool invert_thumbs, int input_min,
                      int input_max) {
  const double normalized = std::clamp(
      (static_cast<double>(raw_value) - static_cast<double>(input_min)) /
          static_cast<double>(input_max - input_min),
      0.0, 1.0);
  double mapped_normalized = normalized;
  if (invert && invert_thumbs) {
    mapped_normalized = 1.0 - mapped_normalized;
  }

  return calibration.lower_limit +
         mapped_normalized *
             (calibration.upper_limit - calibration.lower_limit) +
         calibration.offset;
}

std::optional<HandSnapshot> BuildSnapshot(
    const inspire::inspire_hand_state& message, StateField state_field) {
  const std::vector<int16_t>* selected_values = nullptr;
  if (state_field == StateField::kAngleAct) {
    selected_values = &message.angle_act();
  } else if (state_field == StateField::kPosAct) {
    selected_values = &message.pos_act();
  } else if (message.angle_act().size() == 6) {
    selected_values = &message.angle_act();
  } else {
    selected_values = &message.pos_act();
  }

  if (selected_values->size() != 6) {
    return std::nullopt;
  }

  HandSnapshot snapshot;
  for (size_t index = 0; index < snapshot.values.size(); ++index) {
    snapshot.values[index] = (*selected_values)[index];
  }
  return snapshot;
}

}  // namespace

class G1InspireHandToJointStates : public rclcpp::Node {
 public:
  G1InspireHandToJointStates() : Node("g1_inspire_hand_to_joint_states") {
    const auto joint_state_topic =
        this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    const auto urdf_path =
        this->declare_parameter<std::string>("urdf_path", "");
    const auto left_hand_topic = this->declare_parameter<std::string>(
        "left_hand_topic", "rt/inspire_hand/state/l");
    const auto right_hand_topic = this->declare_parameter<std::string>(
        "right_hand_topic", "rt/inspire_hand/state/r");
    const auto state_field_param =
        this->declare_parameter<std::string>("state_field", "auto");
    const auto network_interface =
        this->declare_parameter<std::string>("network_interface", "");
    const auto publish_period_ms =
        this->declare_parameter<int>("publish_period_ms", 50);
    input_min_ = this->declare_parameter<int>("input_min", 0);
    input_max_ = this->declare_parameter<int>("input_max", 1000);
    invert_thumbs_ = this->declare_parameter<bool>("invert_thumbs", true);
    const auto joint_offsets = this->declare_parameter<std::vector<double>>(
        "joint_offsets", std::vector<double>(12, 0.0));

    if (input_max_ <= input_min_) {
      throw std::runtime_error(
          "Parameter 'input_max' must be greater than 'input_min'");
    }

    state_field_ = ParseStateField(state_field_param);

    if (joint_offsets.size() != 12) {
      RCLCPP_WARN(
          this->get_logger(),
          "Parameter 'joint_offsets' expects 12 entries (left 6 + right 6). "
          "Received %zu entries and will pad or truncate with zeros.",
          joint_offsets.size());
    }
    const auto parsed_joint_offsets = ParseJointOffsets(joint_offsets);
    left_joint_calibrations_ =
        BuildJointCalibrations(kLeftBindings, parsed_joint_offsets, 0, urdf_path);
    right_joint_calibrations_ = BuildJointCalibrations(
        kRightBindings, parsed_joint_offsets, kLeftBindings.size(), urdf_path);

    publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 10);

    try {
      unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    } catch (const std::exception& error) {
      throw std::runtime_error("Failed to initialize Unitree DDS channel: " +
                               std::string(error.what()));
    }

    if (!left_hand_topic.empty()) {
      left_subscriber_ = std::make_shared<
          unitree::robot::ChannelSubscriber<inspire::inspire_hand_state>>(
          left_hand_topic);
      left_subscriber_->InitChannel(
          [this](const void* message) {
            this->UpdateLeftHandState(
                *static_cast<const inspire::inspire_hand_state*>(message));
          },
          10);
    }

    if (!right_hand_topic.empty()) {
      right_subscriber_ = std::make_shared<
          unitree::robot::ChannelSubscriber<inspire::inspire_hand_state>>(
          right_hand_topic);
      right_subscriber_->InitChannel(
          [this](const void* message) {
            this->UpdateRightHandState(
                *static_cast<const inspire::inspire_hand_state*>(message));
          },
          10);
    }

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(publish_period_ms),
        [this]() { this->PublishJointState(); });

    RCLCPP_INFO(
        this->get_logger(),
        "Bridging Inspire hand DDS topics [%s, %s] into %s using %s "
        "(input range [%d, %d])",
        left_hand_topic.c_str(), right_hand_topic.c_str(),
        joint_state_topic.c_str(), state_field_param.c_str(), input_min_,
        input_max_);
  }

 private:
  template <size_t kCount>
  std::array<JointCalibration, kCount> BuildJointCalibrations(
      const std::array<JointBinding, kCount>& bindings,
      const std::array<double, 12>& joint_offsets, size_t offset_index,
      const std::string& urdf_path) {
    std::unordered_map<std::string, UrdfJointLimit> urdf_joint_limits;
    if (!urdf_path.empty()) {
      urdf_joint_limits = LoadJointLimitsFromUrdf(urdf_path);
    }

    std::array<JointCalibration, kCount> calibrations{};
    for (size_t index = 0; index < bindings.size(); ++index) {
      const auto& binding = bindings[index];
      auto lower_limit = binding.default_lower_limit;
      auto upper_limit = binding.default_upper_limit;

      const auto urdf_limit_it = urdf_joint_limits.find(binding.joint_name);
      if (urdf_limit_it != urdf_joint_limits.end()) {
        lower_limit = urdf_limit_it->second.lower_limit;
        upper_limit = urdf_limit_it->second.upper_limit;
      } else if (!urdf_path.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "Joint '%s' not found in URDF %s. Falling back to built-in "
                    "limits [%.4f, %.4f].",
                    binding.joint_name, urdf_path.c_str(), lower_limit,
                    upper_limit);
      }

      calibrations[index] = JointCalibration{
          .lower_limit = lower_limit,
          .upper_limit = upper_limit,
          .offset = joint_offsets[offset_index + index],
      };
    }

    return calibrations;
  }

  void UpdateLeftHandState(const inspire::inspire_hand_state& message) {
    const auto snapshot = BuildSnapshot(message, state_field_);
    if (!snapshot.has_value()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Left Inspire hand state does not contain 6 values");
      return;
    }

    std::scoped_lock lock(state_mutex_);
    left_hand_state_ = snapshot;
  }

  void UpdateRightHandState(const inspire::inspire_hand_state& message) {
    const auto snapshot = BuildSnapshot(message, state_field_);
    if (!snapshot.has_value()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Right Inspire hand state does not contain 6 values");
      return;
    }

    std::scoped_lock lock(state_mutex_);
    right_hand_state_ = snapshot;
  }

  void AppendHandState(const HandSnapshot& snapshot,
                       const std::array<JointBinding, 6>& bindings,
                       const std::array<JointCalibration, 6>& calibrations,
                       sensor_msgs::msg::JointState& joint_state) const {
    for (size_t index = 0; index < bindings.size(); ++index) {
      const auto& binding = bindings[index];
      joint_state.name.emplace_back(binding.joint_name);
      joint_state.position.emplace_back(ScaleHandValue(
          snapshot.values[binding.state_index], calibrations[index],
          binding.invert, invert_thumbs_, input_min_, input_max_));
    }
  }

  void PublishJointState() {
    std::optional<HandSnapshot> left_snapshot;
    std::optional<HandSnapshot> right_snapshot;
    {
      std::scoped_lock lock(state_mutex_);
      left_snapshot = left_hand_state_;
      right_snapshot = right_hand_state_;
    }

    if (!left_snapshot.has_value() && !right_snapshot.has_value()) {
      return;
    }

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->now();

    if (left_snapshot.has_value()) {
      AppendHandState(*left_snapshot, kLeftBindings, left_joint_calibrations_,
                      joint_state);
    }
    if (right_snapshot.has_value()) {
      AppendHandState(*right_snapshot, kRightBindings, right_joint_calibrations_,
                      joint_state);
    }

    if (!joint_state.name.empty()) {
      publisher_->publish(joint_state);
    }
  }

  StateField state_field_{StateField::kAuto};
  int input_min_{0};
  int input_max_{1000};
  bool invert_thumbs_{true};
  std::array<JointCalibration, 6> left_joint_calibrations_{};
  std::array<JointCalibration, 6> right_joint_calibrations_{};
  std::mutex state_mutex_;
  std::optional<HandSnapshot> left_hand_state_;
  std::optional<HandSnapshot> right_hand_state_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  unitree::robot::ChannelSubscriberPtr<inspire::inspire_hand_state>
      left_subscriber_;
  unitree::robot::ChannelSubscriberPtr<inspire::inspire_hand_state>
      right_subscriber_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<G1InspireHandToJointStates>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
