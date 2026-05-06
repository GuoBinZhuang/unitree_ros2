#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "inspire_hand_touch.hpp"

using namespace std::chrono_literals;

namespace {

enum class SummaryGroup : size_t {
  Little = 0,
  Ring,
  Middle,
  Index,
  Thumb,
  Palm,
  Count,
};

constexpr std::array<const char*, static_cast<size_t>(SummaryGroup::Count)>
    kSummaryTopicNames = {{"little", "ring", "middle", "index", "thumb",
                           "palm"}};

using TouchAccessor =
    const std::vector<int16_t>& (inspire::inspire_hand_touch::*)() const;

struct TouchRegion {
  const char* label;
  size_t rows;
  size_t cols;
  SummaryGroup group;
  TouchAccessor accessor;
};

constexpr std::array<TouchRegion, 17> kTouchRegions = {{
    {"little_tip", 3, 3, SummaryGroup::Little,
     &inspire::inspire_hand_touch::fingerone_tip_touch},
    {"little_top", 12, 8, SummaryGroup::Little,
     &inspire::inspire_hand_touch::fingerone_top_touch},
    {"little_palm", 10, 8, SummaryGroup::Little,
     &inspire::inspire_hand_touch::fingerone_palm_touch},
    {"ring_tip", 3, 3, SummaryGroup::Ring,
     &inspire::inspire_hand_touch::fingertwo_tip_touch},
    {"ring_top", 12, 8, SummaryGroup::Ring,
     &inspire::inspire_hand_touch::fingertwo_top_touch},
    {"ring_palm", 10, 8, SummaryGroup::Ring,
     &inspire::inspire_hand_touch::fingertwo_palm_touch},
    {"middle_tip", 3, 3, SummaryGroup::Middle,
     &inspire::inspire_hand_touch::fingerthree_tip_touch},
    {"middle_top", 12, 8, SummaryGroup::Middle,
     &inspire::inspire_hand_touch::fingerthree_top_touch},
    {"middle_palm", 10, 8, SummaryGroup::Middle,
     &inspire::inspire_hand_touch::fingerthree_palm_touch},
    {"index_tip", 3, 3, SummaryGroup::Index,
     &inspire::inspire_hand_touch::fingerfour_tip_touch},
    {"index_top", 12, 8, SummaryGroup::Index,
     &inspire::inspire_hand_touch::fingerfour_top_touch},
    {"index_palm", 10, 8, SummaryGroup::Index,
     &inspire::inspire_hand_touch::fingerfour_palm_touch},
    {"thumb_tip", 3, 3, SummaryGroup::Thumb,
     &inspire::inspire_hand_touch::fingerfive_tip_touch},
    {"thumb_top", 12, 8, SummaryGroup::Thumb,
     &inspire::inspire_hand_touch::fingerfive_top_touch},
    {"thumb_middle", 3, 3, SummaryGroup::Thumb,
     &inspire::inspire_hand_touch::fingerfive_middle_touch},
    {"thumb_palm", 12, 8, SummaryGroup::Thumb,
     &inspire::inspire_hand_touch::fingerfive_palm_touch},
    {"palm", 14, 8, SummaryGroup::Palm, &inspire::inspire_hand_touch::palm_touch},
}};

constexpr size_t kTileColumns = 4;
constexpr size_t kTileWidth = 8;
constexpr size_t kTileHeight = 14;
constexpr size_t kTileGap = 1;
constexpr size_t kTileRows =
    (kTouchRegions.size() + kTileColumns - 1) / kTileColumns;
constexpr size_t kCanvasWidth =
    kTileColumns * kTileWidth + (kTileColumns - 1) * kTileGap;
constexpr size_t kCanvasHeight =
    kTileRows * kTileHeight + (kTileRows - 1) * kTileGap;

struct TouchSnapshot {
  std::array<std::vector<int16_t>, kTouchRegions.size()> values;
};

std::optional<TouchSnapshot> BuildSnapshot(
    const inspire::inspire_hand_touch& message) {
  TouchSnapshot snapshot;
  for (size_t index = 0; index < kTouchRegions.size(); ++index) {
    const auto& region = kTouchRegions[index];
    const auto& region_values = (message.*(region.accessor))();
    if (region_values.size() != region.rows * region.cols) {
      return std::nullopt;
    }
    snapshot.values[index] = region_values;
  }
  return snapshot;
}

float ComputeGroupSummary(const TouchSnapshot& snapshot, SummaryGroup group) {
  float max_value = 0.0F;
  for (size_t region_index = 0; region_index < kTouchRegions.size();
       ++region_index) {
    if (kTouchRegions[region_index].group != group) {
      continue;
    }
    for (const auto raw_value : snapshot.values[region_index]) {
      max_value = std::max(max_value, static_cast<float>(raw_value));
    }
  }
  return max_value;
}

sensor_msgs::msg::Image BuildTouchImage(const TouchSnapshot& snapshot,
                                        const std::string& frame_id,
                                        const builtin_interfaces::msg::Time& stamp) {
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = frame_id;
  image.height = static_cast<uint32_t>(kCanvasHeight);
  image.width = static_cast<uint32_t>(kCanvasWidth);
  image.encoding = sensor_msgs::image_encodings::MONO8;
  image.is_bigendian = false;
  image.step = static_cast<sensor_msgs::msg::Image::_step_type>(kCanvasWidth);
  image.data.assign(kCanvasWidth * kCanvasHeight, 0U);

  for (size_t region_index = 0; region_index < kTouchRegions.size();
       ++region_index) {
    const auto& region = kTouchRegions[region_index];
    const auto& region_values = snapshot.values[region_index];

    const size_t tile_row = region_index / kTileColumns;
    const size_t tile_col = region_index % kTileColumns;
    const size_t x_offset = tile_col * (kTileWidth + kTileGap) +
                            (kTileWidth - region.cols) / 2;
    const size_t y_offset = tile_row * (kTileHeight + kTileGap) +
                            (kTileHeight - region.rows) / 2;

    int16_t max_region_value = 0;
    for (const auto value : region_values) {
      max_region_value = std::max(max_region_value, value);
    }

    for (size_t row = 0; row < region.rows; ++row) {
      for (size_t col = 0; col < region.cols; ++col) {
        const size_t value_index = row * region.cols + col;
        uint8_t intensity = 0U;
        if (max_region_value > 0) {
          intensity = static_cast<uint8_t>(std::clamp(
              static_cast<int>(
                  (static_cast<float>(region_values[value_index]) * 255.0F) /
                  static_cast<float>(max_region_value)),
              0, 255));
        }
        const size_t image_index =
            (y_offset + row) * kCanvasWidth + (x_offset + col);
        image.data[image_index] = intensity;
      }
    }
  }

  return image;
}

std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr,
           static_cast<size_t>(SummaryGroup::Count)>
CreateSummaryPublishers(
    rclcpp::Node& node, const std::string& summary_namespace) {
  std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr,
             static_cast<size_t>(SummaryGroup::Count)>
      publishers;
  for (size_t index = 0; index < publishers.size(); ++index) {
    publishers[index] = node.create_publisher<std_msgs::msg::Float32>(
        summary_namespace + "/" + kSummaryTopicNames[index], 10);
  }
  return publishers;
}

}  // namespace

class G1InspireHandTouchToVisualization : public rclcpp::Node {
 public:
  G1InspireHandTouchToVisualization()
      : Node("g1_inspire_hand_touch_to_visualization") {
    const auto network_interface =
        this->declare_parameter<std::string>("network_interface", "");
    const auto left_touch_topic = this->declare_parameter<std::string>(
        "left_touch_topic", "rt/inspire_hand/touch/l");
    const auto right_touch_topic = this->declare_parameter<std::string>(
        "right_touch_topic", "rt/inspire_hand/touch/r");
    const auto left_image_topic = this->declare_parameter<std::string>(
        "left_image_topic", "/inspire_hand/touch/left/image");
    const auto right_image_topic = this->declare_parameter<std::string>(
        "right_image_topic", "/inspire_hand/touch/right/image");
    const auto left_summary_namespace = this->declare_parameter<std::string>(
        "left_summary_namespace", "/inspire_hand/touch/left");
    const auto right_summary_namespace = this->declare_parameter<std::string>(
        "right_summary_namespace", "/inspire_hand/touch/right");
    const auto publish_period_ms =
        this->declare_parameter<int>("publish_period_ms", 100);

    left_image_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(left_image_topic, 10);
    right_image_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(right_image_topic, 10);
    left_summary_publishers_ =
        CreateSummaryPublishers(*this, left_summary_namespace);
    right_summary_publishers_ =
        CreateSummaryPublishers(*this, right_summary_namespace);

    try {
      unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    } catch (const std::exception& error) {
      throw std::runtime_error("Failed to initialize Unitree DDS channel: " +
                               std::string(error.what()));
    }

    if (!left_touch_topic.empty()) {
      left_subscriber_ = std::make_shared<
          unitree::robot::ChannelSubscriber<inspire::inspire_hand_touch>>(
          left_touch_topic);
      left_subscriber_->InitChannel(
          [this](const void* message) {
            this->UpdateLeftTouch(
                *static_cast<const inspire::inspire_hand_touch*>(message));
          },
          10);
    }

    if (!right_touch_topic.empty()) {
      right_subscriber_ = std::make_shared<
          unitree::robot::ChannelSubscriber<inspire::inspire_hand_touch>>(
          right_touch_topic);
      right_subscriber_->InitChannel(
          [this](const void* message) {
            this->UpdateRightTouch(
                *static_cast<const inspire::inspire_hand_touch*>(message));
          },
          10);
    }

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(publish_period_ms),
        [this]() { this->PublishVisualization(); });

    RCLCPP_INFO(
        this->get_logger(),
        "Bridging Inspire touch DDS topics [%s, %s] to image topics [%s, %s]",
        left_touch_topic.c_str(), right_touch_topic.c_str(),
        left_image_topic.c_str(), right_image_topic.c_str());
  }

 private:
  void UpdateLeftTouch(const inspire::inspire_hand_touch& message) {
    const auto snapshot = BuildSnapshot(message);
    if (!snapshot.has_value()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Left Inspire touch payload shape mismatch");
      return;
    }

    std::scoped_lock lock(snapshot_mutex_);
    left_touch_snapshot_ = snapshot;
  }

  void UpdateRightTouch(const inspire::inspire_hand_touch& message) {
    const auto snapshot = BuildSnapshot(message);
    if (!snapshot.has_value()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Right Inspire touch payload shape mismatch");
      return;
    }

    std::scoped_lock lock(snapshot_mutex_);
    right_touch_snapshot_ = snapshot;
  }

  void PublishSummaryPublishers(
      const TouchSnapshot& snapshot,
      const std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr,
                       static_cast<size_t>(SummaryGroup::Count)>& publishers) {
    for (size_t index = 0; index < publishers.size(); ++index) {
      std_msgs::msg::Float32 message;
      message.data =
          ComputeGroupSummary(snapshot, static_cast<SummaryGroup>(index));
      publishers[index]->publish(message);
    }
  }

  void PublishVisualization() {
    std::optional<TouchSnapshot> left_snapshot;
    std::optional<TouchSnapshot> right_snapshot;
    {
      std::scoped_lock lock(snapshot_mutex_);
      left_snapshot = left_touch_snapshot_;
      right_snapshot = right_touch_snapshot_;
    }

    const auto now_ns = this->now().nanoseconds();
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
    if (left_snapshot.has_value()) {
      left_image_publisher_->publish(
          BuildTouchImage(*left_snapshot, "inspire_left_touch", stamp));
      PublishSummaryPublishers(*left_snapshot, left_summary_publishers_);
    }
    if (right_snapshot.has_value()) {
      right_image_publisher_->publish(
          BuildTouchImage(*right_snapshot, "inspire_right_touch", stamp));
      PublishSummaryPublishers(*right_snapshot, right_summary_publishers_);
    }
  }

  std::mutex snapshot_mutex_;
  std::optional<TouchSnapshot> left_touch_snapshot_;
  std::optional<TouchSnapshot> right_touch_snapshot_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_publisher_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr,
             static_cast<size_t>(SummaryGroup::Count)>
      left_summary_publishers_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr,
             static_cast<size_t>(SummaryGroup::Count)>
      right_summary_publishers_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  unitree::robot::ChannelSubscriberPtr<inspire::inspire_hand_touch>
      left_subscriber_;
  unitree::robot::ChannelSubscriberPtr<inspire::inspire_hand_touch>
      right_subscriber_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<G1InspireHandTouchToVisualization>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
