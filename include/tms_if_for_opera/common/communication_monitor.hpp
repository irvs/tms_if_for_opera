#ifndef TMS_IF_FOR_OPERA_COMMUNICATION_MONITOR_HPP_
#define TMS_IF_FOR_OPERA_COMMUNICATION_MONITOR_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

/**
 * Monitors freshness of one robot-state topic.
 * Supported monitor_type: pose, joint_state (position is accepted as pose alias).
 * Supported pose message types: geometry_msgs/msg/PoseStamped,
 *                               nav_msgs/msg/Odometry.
 */
class CommunicationMonitor
{
public:
  explicit CommunicationMonitor(rclcpp::Node * node) : node_(node)
  {
    node_->declare_parameter<bool>("monitor_enabled", false);
    node_->declare_parameter<std::string>("monitor_type", "pose");
    node_->declare_parameter<std::string>("monitor_msg_type", "geometry_msgs/msg/PoseStamped");
    node_->declare_parameter<std::string>("monitor_topic", "");
    node_->declare_parameter<double>("monitor_timeout", 1.0);
    node_->declare_parameter<double>("monitor_check_period", 0.1);

    enabled_ = node_->get_parameter("monitor_enabled").as_bool();
    monitor_type_ = node_->get_parameter("monitor_type").as_string();
    message_type_ = node_->get_parameter("monitor_msg_type").as_string();
    topic_ = node_->get_parameter("monitor_topic").as_string();
    timeout_sec_ = node_->get_parameter("monitor_timeout").as_double();
    check_period_sec_ = node_->get_parameter("monitor_check_period").as_double();

    if (!enabled_) {
      RCLCPP_INFO(node_->get_logger(), "Communication monitoring is disabled");
      return;
    }
    if (topic_.empty()) {
      throw std::runtime_error("monitor_enabled=true but monitor_topic is empty");
    }
    if (timeout_sec_ <= 0.0 || check_period_sec_ <= 0.0) {
      throw std::runtime_error("monitor_timeout and monitor_check_period must be > 0");
    }

    const auto qos = rclcpp::SensorDataQoS();
    if (monitor_type_ == "joint_state") {
      joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        topic_, qos, [this](sensor_msgs::msg::JointState::ConstSharedPtr) { mark_received(); });
    } else if (monitor_type_ == "pose" || monitor_type_ == "position") {
      if (message_type_ == "geometry_msgs/msg/PoseStamped") {
        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          topic_, qos, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr) { mark_received(); });
      } else if (message_type_ == "nav_msgs/msg/Odometry") {
        odometry_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
          topic_, qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr) { mark_received(); });
      } else {
        throw std::runtime_error("Unsupported monitor_msg_type: " + message_type_);
      }
    } else {
      throw std::runtime_error("Unsupported monitor_type: " + monitor_type_ +
                               " (use pose or joint_state)");
    }

    RCLCPP_INFO(node_->get_logger(),
      "Communication monitor: type=%s, msg=%s, topic=%s, timeout=%.3f sec",
      monitor_type_.c_str(), message_type_.c_str(), topic_.c_str(), timeout_sec_);

    timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(check_period_sec_),
      std::bind(&CommunicationMonitor::check_connection, this));
  }

  bool is_connected() const
  {
    if (!enabled_) return true;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_) return false;
    return std::chrono::duration<double>(Clock::now() - last_received_).count() <= timeout_sec_;
  }

  void set_disconnect_callback(std::function<void()> callback)
  {
    disconnect_callback_ = std::move(callback);
  }

  bool enabled() const { return enabled_; }

private:
  using Clock = std::chrono::steady_clock;

  void mark_received()
  {
    bool first_message = false;
    bool was_connected = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      first_message = !received_;
      last_received_ = Clock::now();
      received_ = true;
      was_connected = was_connected_.exchange(true);
    }
    if (first_message) {
      RCLCPP_INFO(node_->get_logger(), "First robot-state message received: %s", topic_.c_str());
    } else if (!was_connected) {
      RCLCPP_INFO(node_->get_logger(), "Robot state topic recovered: %s", topic_.c_str());
    }
  }

  void check_connection()
  {
    if (!enabled_) return;
    const bool connected = is_connected();
    const bool previous = was_connected_.exchange(connected);

    if (connected && !previous) {
      RCLCPP_INFO(node_->get_logger(), "Robot state topic recovered: %s", topic_.c_str());
    } else if (!connected && previous) {
      RCLCPP_ERROR(node_->get_logger(), "Robot communication timeout: %s (%.3f sec)",
                   topic_.c_str(), timeout_sec_);
      if (disconnect_callback_) disconnect_callback_();
    }
  }

  rclcpp::Node * node_;
  bool enabled_{false};
  std::string monitor_type_;
  std::string message_type_;
  std::string topic_;
  double timeout_sec_{1.0};
  double check_period_sec_{0.1};
  mutable std::mutex mutex_;
  Clock::time_point last_received_{};
  bool received_{false};
  std::atomic<bool> was_connected_{false};
  std::function<void()> disconnect_callback_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

#endif  // TMS_IF_FOR_OPERA_COMMUNICATION_MONITOR_HPP_
