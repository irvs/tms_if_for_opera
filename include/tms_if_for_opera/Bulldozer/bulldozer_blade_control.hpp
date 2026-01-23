#ifndef BULLDOZER_BLADE_CONTROL_HPP
#define BULLDOZER_BLADE_CONTROL_HPP

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tms_msg_rp/action/tms_rp_bulldozer_blade.hpp"

class BulldozerBladeControl : public rclcpp::Node
{
public:
  using Action = tms_msg_rp::action::TmsRpBulldozerBlade;

  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using Client = rclcpp_action::Client<Action>;
  using WrappedResult = Client::WrappedResult;

  BulldozerBladeControl();

private:
  // ---- server side (受け口) ----
  rclcpp_action::Server<Action>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Action::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<ServerGoalHandle> server_goal_handle);

  void handle_accepted(const std::shared_ptr<ServerGoalHandle> goal_handle);
  void execute(const std::shared_ptr<ServerGoalHandle> server_goal_handle);

  // ---- client side (中継先) ----
  Client::SharedPtr action_client_;
  std::shared_future<typename ClientGoalHandle::SharedPtr> client_future_goal_handle_;

  void goal_response_callback(const typename ClientGoalHandle::SharedPtr & goal_handle);

  void result_callback(
    const std::shared_ptr<ServerGoalHandle> server_goal_handle,
    const WrappedResult & result);
};

#endif  // BULLDOZER_BLADE_CONTROL_HPP
