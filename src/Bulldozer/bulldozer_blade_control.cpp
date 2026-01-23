#include <thread>
#include <utility>

#include "tms_if_for_opera/Bulldozer/bulldozer_blade_control.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

BulldozerBladeControl::BulldozerBladeControl()
: rclcpp::Node("tms_if_bulldozer_blade_control_node")
{
  // 受け口（このノードが提供するアクション名）
  action_server_ = rclcpp_action::create_server<Action>(
    this,
    "tms_rp_set_bulldozer_blade",
    std::bind(&BulldozerBladeControl::handle_goal, this, _1, _2),
    std::bind(&BulldozerBladeControl::handle_cancel, this, _1),
    std::bind(&BulldozerBladeControl::handle_accepted, this, _1));

  // 中継先（実機/シミュレータ側が提供しているアクション名）
  action_client_ = rclcpp_action::create_client<Action>(this, "set_bulldozer_blade");
}

rclcpp_action::GoalResponse BulldozerBladeControl::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Action::Goal> goal)
{
  // Goal の最低限チェック（仕様通り）
  if (goal->joint_name.empty()) {
    RCLCPP_WARN(this->get_logger(), "Reject: joint_name is empty");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->goal_position.empty()) {
    RCLCPP_WARN(this->get_logger(), "Reject: goal_position is empty (required)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->joint_name.size() != goal->goal_position.size()) {
    RCLCPP_WARN(this->get_logger(),
      "Reject: size mismatch joint_name(%zu) vs goal_position(%zu)",
      goal->joint_name.size(), goal->goal_position.size());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->control_type == 1 && !goal->velocity.empty() &&
      goal->velocity.size() != goal->joint_name.size())
  {
    RCLCPP_WARN(this->get_logger(), "Reject: velocity size mismatch");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->control_type == 2 && !goal->effort.empty() &&
      goal->effort.size() != goal->joint_name.size())
  {
    RCLCPP_WARN(this->get_logger(), "Reject: effort size mismatch");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Accepted goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BulldozerBladeControl::handle_cancel(
  const std::shared_ptr<ServerGoalHandle> /*server_goal_handle*/)
{
  RCLCPP_INFO(this->get_logger(), "Cancel requested (server side)");

  // client 側へキャンセル中継
  try {
    if (client_future_goal_handle_.valid() &&
        client_future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto client_goal_handle = client_future_goal_handle_.get();
      if (client_goal_handle) {
        action_client_->async_cancel_goal(client_goal_handle);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in cancel: %s", e.what());
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void BulldozerBladeControl::handle_accepted(const std::shared_ptr<ServerGoalHandle> goal_handle)
{
  std::thread{std::bind(&BulldozerBladeControl::execute, this, _1), goal_handle}.detach();
}

void BulldozerBladeControl::execute(const std::shared_ptr<ServerGoalHandle> server_goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "BulldozerBladeControl executing...");

  // 中継先がいなければ abort
  auto result_to_server = std::make_shared<Action::Result>();
  result_to_server->success = false;

  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "Downstream action server not available");
    if (server_goal_handle->is_active()) {
      server_goal_handle->abort(result_to_server);
    }
    return;
  }

  // server 側で受けた Goal を client 側へ中継
  Action::Goal goal_msg{};
  goal_msg = *server_goal_handle->get_goal();  // 丸ごとコピー

  typename Client::SendGoalOptions send_goal_options;

  send_goal_options.goal_response_callback =
    [this](const typename ClientGoalHandle::SharedPtr & gh) {
      goal_response_callback(gh);
    };

  // downstream feedback を upstream feedback としてそのまま流す
  send_goal_options.feedback_callback =
    [server_goal_handle](typename ClientGoalHandle::SharedPtr /*gh*/,
                         const std::shared_ptr<const Action::Feedback> feedback)
    {
      if (!server_goal_handle->is_active()) {
        return;
      }
      auto fb = std::make_shared<Action::Feedback>();
      fb->current_error = feedback->current_error;
      server_goal_handle->publish_feedback(fb);
    };

  send_goal_options.result_callback =
    [this, server_goal_handle](const WrappedResult & wrapped) {
      result_callback(server_goal_handle, wrapped);
    };

  RCLCPP_INFO(this->get_logger(), "Relaying goal to downstream...");
  client_future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
}

void BulldozerBladeControl::goal_response_callback(const typename ClientGoalHandle::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Downstream goal rejected");
  } else {
    RCLCPP_INFO(this->get_logger(), "Downstream goal accepted");
  }
}

void BulldozerBladeControl::result_callback(
  const std::shared_ptr<ServerGoalHandle> server_goal_handle,
  const WrappedResult & result)
{
  if (!server_goal_handle->is_active()) {
    RCLCPP_WARN(this->get_logger(), "Server goal is not active anymore");
    return;
  }

  auto result_to_server = std::make_shared<Action::Result>();
  result_to_server->success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      server_goal_handle->succeed(result_to_server);
      break;

    case rclcpp_action::ResultCode::ABORTED:
      server_goal_handle->abort(result_to_server);
      break;

    case rclcpp_action::ResultCode::CANCELED:
      server_goal_handle->canceled(result_to_server);
      break;

    default:
      server_goal_handle->abort(result_to_server);
      break;
  }
}


int main(int argc, char* argv[])
{
    // Initialize Google's logging library.
    //   google::InitGoogleLogging(argv[0]);
    //   google::InstallFailureSignalHandler();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BulldozerBladeControl>());
    rclcpp::shutdown();
    return 0;
}
