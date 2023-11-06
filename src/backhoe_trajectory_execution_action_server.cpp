#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp_components/register_node_macro.hpp"

#include "tms_if_for_opera/visibility_control.h"
#include "tms_msg_ts/action/tms_ts_backhoe_trajectory_execution.hpp"

/** Moveit! **/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>
/*****/

namespace tms_if_for_opera
{
class BackhoeTrajectoryExecutionActionServer : public rclcpp::Node
{
public:
  using BackhoeTrajectoryExecution = tms_msg_ts::action::TmsTsBackhoeTrajectoryExecution;
  using GoalHandleBackhoeTrajectoryExecution = rclcpp_action::ServerGoalHandle<BackhoeTrajectoryExecution>;

  explicit BackhoeTrajectoryExecutionActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("backhoe_trajectory_execution_action_server", options), planning_group("manipulator")
  {
    /* Create server */
    RCLCPP_INFO(this->get_logger(), "Create server.");  // debug
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<BackhoeTrajectoryExecution>(
        this, "backhoe_trajectory_execution",
        std::bind(&BackhoeTrajectoryExecutionActionServer::handle_goal, this, _1, _2),
        std::bind(&BackhoeTrajectoryExecutionActionServer::handle_cancel, this, _1),
        std::bind(&BackhoeTrajectoryExecutionActionServer::handle_accepted, this, _1));
    /****/

    /* Setup movegroup interface */
    // thisをshared_ptrに変換する良さげな方法が見つからないため、渋々ノード生成
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    move_group_node_ = rclcpp::Node::make_shared("backhoe_trajectory_execution_action_server_move_group");

    // robot の状態監視のため
    executor.add_node(move_group_node_);
    std::thread([this]() { executor.spin(); }).detach();

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    /*** debug ***/

    // RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    // RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
    // RCLCPP_INFO(this->get_logger(), "Available Planning Groups:");
    // std::copy(move_group_->getJointModelGroupNames().begin(), move_group_->getJointModelGroupNames().end(),
    //           std::ostream_iterator<std::string>(std::cout, ", "));
    // RCLCPP_INFO(this->get_logger(), "Joint Names:");
    // std::copy(move_group_->getJointNames().begin(), move_group_->getJointNames().end(),
    //           std::ostream_iterator<std::string>(std::cout, ", "));

    /******/
  }

private:
  rclcpp_action::Server<BackhoeTrajectoryExecution>::SharedPtr action_server_;
  rclcpp::Node::SharedPtr move_group_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  const std::string planning_group;
  rclcpp::executors::SingleThreadedExecutor executor;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const BackhoeTrajectoryExecution::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleBackhoeTrajectoryExecution> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleBackhoeTrajectoryExecution> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "handle_accepted() start.");
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&BackhoeTrajectoryExecutionActionServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<GoalHandleBackhoeTrajectoryExecution> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<BackhoeTrajectoryExecution::Feedback>();
    auto result = std::make_shared<BackhoeTrajectoryExecution::Result>();

    for (const auto& point : goal->trajectory.points)
    {
      std::map<std::string, double> target_joints;
      for (size_t i = 0; i < goal->trajectory.joint_names.size() && i < point.positions.size(); ++i)
      {
        target_joints[goal->trajectory.joint_names[i]] = point.positions[i];
      }
      move_group_->setJointValueTarget(target_joints);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success)
      {
        auto moveit_result = move_group_->execute(my_plan);

        // Check the result of the execution.
        if (moveit_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
          // Create a result message.
          result->error_code.val = 1;
          RCLCPP_INFO(this->get_logger(), "Execution succeeded");
        }
        else
        {
          result->error_code.val = 9999;
          RCLCPP_ERROR(this->get_logger(), "Execution failed");
          break;
        }
      }
      else
      {
        result->error_code.val = 9999;
        RCLCPP_ERROR(this->get_logger(), "Planning failed");
        break;
      }
    }

    // Pass the result to the goal handle.
    if (result->error_code.val == 1)
    {
      goal_handle->succeed(result);
    }
    else
    {
      goal_handle->abort(result);
    }
  }
};
}  // namespace tms_if_for_opera

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tms_if_for_opera::BackhoeTrajectoryExecutionActionServer>());
  rclcpp::shutdown();
  return 0;
}