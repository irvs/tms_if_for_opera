#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "tms_msg_ts/action/tms_ts_backhoe_trajectory_execution.hpp"

class BackhoeTrajectoryExecutionActionClient : public rclcpp::Node
{
public:
  using BackhoeTrajectoryExecution = tms_msg_ts::action::TmsTsBackhoeTrajectoryExecution;
  using GoalHandleBackhoeTrajectoryExecution = rclcpp_action::ClientGoalHandle<BackhoeTrajectoryExecution>;

  explicit BackhoeTrajectoryExecutionActionClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("backhoe_trajectory_execution_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<BackhoeTrajectoryExecution>(this, "backhoe_trajectory_execution");

    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                           std::bind(&BackhoeTrajectoryExecutionActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = BackhoeTrajectoryExecution::Goal();
    goal_msg.trajectory.joint_names = { "swing_joint", "boom_joint", "arm_joint", "bucket_joint", "bucket_end_joint" };

    trajectory_msgs::msg::JointTrajectoryPoint point1;
    point1.positions = { 0.0, 0.0, 1.7, 0.0, 0.0 };
    goal_msg.trajectory.points.push_back(point1);

    trajectory_msgs::msg::JointTrajectoryPoint point2;
    point2.positions = { 1.7, 0.0, 1.7, 0.0, 0.0 };
    goal_msg.trajectory.points.push_back(point2);

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<BackhoeTrajectoryExecution>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&BackhoeTrajectoryExecutionActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&BackhoeTrajectoryExecutionActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&BackhoeTrajectoryExecutionActionClient::result_callback, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<BackhoeTrajectoryExecution>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleBackhoeTrajectoryExecution::SharedPtr& goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(const GoalHandleBackhoeTrajectoryExecution::SharedPtr,
                         const std::shared_ptr<const GoalHandleBackhoeTrajectoryExecution::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Feedback received: ";

    ss << feedback->state << " ";

    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleBackhoeTrajectoryExecution::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal was succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    // std::stringstream ss;
    // ss << "Result received: ";
    // ss << result.result->error_code << " ";
    // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<BackhoeTrajectoryExecutionActionClient>();
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}