#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "nlohmann/json.hpp"
#include "time_tools.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "ut_errror.hpp"

class BaseClient {
  using Request = unitree_api::msg::Request;
  using Response = unitree_api::msg::Response;

  rclcpp::Node* node_;
  std::string topic_name_request_;
  std::string topic_name_response_;
  rclcpp::Publisher<Request>::SharedPtr req_puber_;
  rclcpp::Subscription<Response>::SharedPtr response_suber_;
  std::atomic<uint64_t> current_request_id_{0};
  std::mutex call_mutex_;
  std::mutex response_mutex_;
  std::condition_variable response_cv_;
  std::shared_ptr<const Response> received_response_;
  bool response_ready_{false};

 public:
  BaseClient(
      rclcpp::Node* node, const std::string& topic_name_request,
      std::string topic_name_response,
      const rclcpp::CallbackGroup::SharedPtr& callback_group = nullptr)
      : node_(node),
        topic_name_request_(topic_name_request),
        topic_name_response_(std::move(topic_name_response)),
        req_puber_(node_->create_publisher<Request>(topic_name_request,
                                                    rclcpp::QoS(1))),
        response_suber_(node_->create_subscription<Response>(
            topic_name_response_, rclcpp::QoS(1),
            [this](const std::shared_ptr<const Response> data) {
              OnResponseReceived(data);
            },
            CreateSubscriptionOptions(callback_group))) {}

  int32_t Call(Request req, nlohmann::json& js,
               std::chrono::milliseconds timeout) {
    std::lock_guard<std::mutex> call_lock(call_mutex_);
    ResetPendingState();

    req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
    const auto identity_id = req.header.identity.id;
    current_request_id_.store(identity_id, std::memory_order_release);

    req_puber_->publish(req);

    std::shared_ptr<const Response> response;
    {
      std::unique_lock<std::mutex> response_lock(response_mutex_);
      if (!response_cv_.wait_for(response_lock, timeout, [this]() {
            return response_ready_;
          })) {
        current_request_id_.store(0, std::memory_order_release);
        ResetPendingStateLocked();
        return UT_ROBOT_TASK_TIMEOUT;
      }
      response = received_response_;
    }

    current_request_id_.store(0, std::memory_order_release);
    ResetPendingState();

    if (!response) {
      return UT_ROBOT_TASK_UNKNOWN_ERROR;
    }

    if (response->header.status.code != 0) {
      std::cout << "error code: " << response->header.status.code << std::endl;
      return response->header.status.code;
    }

    try {
      js = nlohmann::json::parse(response->data.data());
    } catch (const nlohmann::detail::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to parse RPC response JSON: %s",
                   e.what());
      return UT_ROBOT_TASK_UNKNOWN_ERROR;
    }
    return UT_ROBOT_SUCCESS;
  }

  int32_t Call(Request req, nlohmann::json& js) {
    return Call(std::move(req), js, std::chrono::seconds(5));
  }

  int32_t Call(Request req) {
    nlohmann::json js;
    return Call(std::move(req), js, std::chrono::seconds(5));
  }

  int32_t Call(Request req, std::chrono::milliseconds timeout) {
    nlohmann::json js;
    return Call(std::move(req), js, timeout);
  }

 private:
  static rclcpp::SubscriptionOptions CreateSubscriptionOptions(
      const rclcpp::CallbackGroup::SharedPtr& callback_group) {
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group;
    return options;
  }

  void OnResponseReceived(const std::shared_ptr<const Response>& data) {
    if (!data) {
      return;
    }

    const uint64_t response_id = data->header.identity.id;
    const uint64_t expected_id =
        current_request_id_.load(std::memory_order_acquire);
    if (expected_id == 0 || response_id != expected_id) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(response_mutex_);
      if (response_id != current_request_id_.load(std::memory_order_acquire)) {
        return;
      }
      if (response_ready_) {
        return;
      }
      received_response_ = data;
      response_ready_ = true;
    }
    response_cv_.notify_one();
  }

  void ResetPendingState() {
    std::lock_guard<std::mutex> lock(response_mutex_);
    ResetPendingStateLocked();
  }

  void ResetPendingStateLocked() {
    received_response_.reset();
    response_ready_ = false;
  }
};
