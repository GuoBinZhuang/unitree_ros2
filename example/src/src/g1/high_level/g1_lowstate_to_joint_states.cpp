#include <algorithm>
#include <chrono>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_hg/msg/low_state.hpp>

using LowState = unitree_hg::msg::LowState;
using namespace std::chrono_literals;

namespace {

struct JointBinding {
  size_t motor_index;
  std::string joint_name;
};

struct UrdfJointInfo {
  std::string name;
  bool movable;
  bool mimic;
};

bool IsDynamicJointType(const std::string& joint_type) {
  return joint_type == "revolute" || joint_type == "continuous" ||
         joint_type == "prismatic";
}

std::vector<std::string> BuildDefaultJointBindingParameters() {
  return {
      "0:left_hip_pitch_joint",    "1:left_hip_roll_joint",
      "2:left_hip_yaw_joint",      "3:left_knee_joint",
      "4:left_ankle_pitch_joint",  "5:left_ankle_roll_joint",
      "6:right_hip_pitch_joint",   "7:right_hip_roll_joint",
      "8:right_hip_yaw_joint",     "9:right_knee_joint",
      "10:right_ankle_pitch_joint", "11:right_ankle_roll_joint",
      "12:waist_yaw_joint",        "13:waist_roll_joint",
      "14:waist_pitch_joint",      "15:left_shoulder_pitch_joint",
      "16:left_shoulder_roll_joint", "17:left_shoulder_yaw_joint",
      "18:left_elbow_joint",       "19:left_wrist_roll_joint",
      "20:left_wrist_pitch_joint", "21:left_wrist_yaw_joint",
      "22:right_shoulder_pitch_joint", "23:right_shoulder_roll_joint",
      "24:right_shoulder_yaw_joint", "25:right_elbow_joint",
      "26:right_wrist_roll_joint", "27:right_wrist_pitch_joint",
      "28:right_wrist_yaw_joint",
  };
}

std::string Trim(const std::string& value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }

  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

std::vector<JointBinding> ParseJointBindings(
    const std::vector<std::string>& binding_parameters) {
  std::vector<JointBinding> bindings;
  bindings.reserve(binding_parameters.size());

  for (const auto& entry : binding_parameters) {
    const auto separator = entry.find(':');
    if (separator == std::string::npos) {
      throw std::runtime_error(
          "Joint binding must use '<motor_index>:<joint_name>': " + entry);
    }

    const auto motor_index_text = Trim(entry.substr(0, separator));
    const auto joint_name = Trim(entry.substr(separator + 1));
    if (motor_index_text.empty() || joint_name.empty()) {
      throw std::runtime_error("Joint binding must not be empty: " + entry);
    }

    size_t motor_index = 0;
    try {
      motor_index = static_cast<size_t>(std::stoul(motor_index_text));
    } catch (const std::exception&) {
      throw std::runtime_error("Invalid motor index in joint binding: " +
                               entry);
    }

    bindings.push_back({motor_index, joint_name});
  }

  return bindings;
}

std::vector<UrdfJointInfo> LoadJointInfosFromUrdf(const std::string& urdf_path) {
  std::ifstream urdf_stream(urdf_path);
  if (!urdf_stream.is_open()) {
    throw std::runtime_error("Failed to open URDF: " + urdf_path);
  }

  const std::regex joint_start_regex(
      R"joint(<joint\s+name="([^"]+)"\s+type="([^"]+)")joint");
  const std::regex mimic_regex(R"joint(<mimic\s+joint="([^"]+)")joint");
  std::vector<UrdfJointInfo> joint_infos;
  std::string line;
  bool in_joint = false;
  UrdfJointInfo current_joint;

  while (std::getline(urdf_stream, line)) {
    if (!in_joint) {
      std::smatch match;
      if (!std::regex_search(line, match, joint_start_regex)) {
        continue;
      }

      current_joint = {
          .name = match[1].str(),
          .movable = IsDynamicJointType(match[2].str()),
          .mimic = false,
      };
      in_joint = true;
    }

    if (std::regex_search(line, mimic_regex)) {
      current_joint.mimic = true;
    }

    if (line.find("</joint>") != std::string::npos) {
      joint_infos.push_back(current_joint);
      in_joint = false;
    }
  }

  if (joint_infos.empty()) {
    throw std::runtime_error("No joints found in URDF: " + urdf_path);
  }

  return joint_infos;
}

}  // namespace

class G1LowStateToJointStates : public rclcpp::Node {
 public:
  G1LowStateToJointStates() : Node("g1_lowstate_to_joint_states") {
    const auto lowstate_topic =
        this->declare_parameter<std::string>("lowstate_topic", "/lowstate");
    const auto joint_state_topic =
        this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    const auto urdf_path =
        this->declare_parameter<std::string>("urdf_path", "");
    const auto joint_binding_parameters =
        this->declare_parameter<std::vector<std::string>>(
            "joint_bindings", BuildDefaultJointBindingParameters());

    if (urdf_path.empty()) {
      throw std::runtime_error("Parameter 'urdf_path' must not be empty");
    }

    joint_bindings_ = ParseJointBindings(joint_binding_parameters);
    const auto urdf_joint_infos = LoadJointInfosFromUrdf(urdf_path);
    std::set<std::string> requested_joint_names;
    for (const auto& binding : joint_bindings_) {
      requested_joint_names.insert(binding.joint_name);
    }

    std::set<std::string> movable_non_mimic_joint_names;
    for (const auto& joint_info : urdf_joint_infos) {
      if (!joint_info.movable || joint_info.mimic) {
        continue;
      }

      movable_non_mimic_joint_names.insert(joint_info.name);
      if (requested_joint_names.count(joint_info.name) == 0U) {
        continue;
      }

      joint_name_to_index_.emplace(joint_info.name, joint_names_.size());
      joint_names_.push_back(joint_info.name);
    }

    if (joint_names_.empty()) {
      throw std::runtime_error(
          "No configured joint bindings matched movable non-mimic URDF joints: " +
          urdf_path);
    }

    joint_positions_.assign(joint_names_.size(), 0.0);
    joint_velocities_.assign(joint_names_.size(), 0.0);

    for (const auto& binding : joint_bindings_) {
      if (movable_non_mimic_joint_names.count(binding.joint_name) == 0U) {
        RCLCPP_WARN(this->get_logger(),
                    "Joint '%s' is not present in the configured URDF",
                    binding.joint_name.c_str());
      }
    }

    for (const auto& joint_name : movable_non_mimic_joint_names) {
      if (requested_joint_names.count(joint_name) != 0U) {
        continue;
      }

      RCLCPP_WARN(this->get_logger(),
                  "URDF joint '%s' has no LowState mapping and will not be "
                  "published to JointState",
                  joint_name.c_str());
    }

    publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 10);
    subscription_ = this->create_subscription<LowState>(
        lowstate_topic, 10,
        [this](const LowState::SharedPtr msg) { UpdateJointState(*msg); });
    publish_timer_ =
        this->create_wall_timer(50ms, [this]() { PublishJointState(); });

    RCLCPP_INFO(this->get_logger(),
                "Bridging %zu mapped URDF joints from %s to %s using %s",
                joint_names_.size(), lowstate_topic.c_str(),
                joint_state_topic.c_str(), urdf_path.c_str());
  }

 private:
  void UpdateJointState(const LowState& lowstate) {
    for (const auto& binding : joint_bindings_) {
      if (binding.motor_index >= lowstate.motor_state.size()) {
        continue;
      }

      const auto joint_it = joint_name_to_index_.find(binding.joint_name);
      if (joint_it == joint_name_to_index_.end()) {
        continue;
      }

      const auto joint_index = joint_it->second;
      joint_positions_[joint_index] = lowstate.motor_state[binding.motor_index].q;
      joint_velocities_[joint_index] = lowstate.motor_state[binding.motor_index].dq;
    }
  }

  void PublishJointState() {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->now();
    joint_state.name = joint_names_;
    joint_state.position = joint_positions_;
    joint_state.velocity = joint_velocities_;

    publisher_->publish(joint_state);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<LowState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::vector<JointBinding> joint_bindings_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::unordered_map<std::string, size_t> joint_name_to_index_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<G1LowStateToJointStates>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
