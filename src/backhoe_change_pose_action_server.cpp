#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp_components/register_node_macro.hpp"

#include "tms_if_for_opera/visibility_control.h"
#include "tms_msg_ts/action/tms_ts_backhoe_change_pose.hpp"

/** Moveit! **/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>
/*****/

#include <excavator_ik/excavator_ik.hpp>

namespace tms_if_for_opera
{
class BackhoeChangePoseActionServer : public rclcpp::Node
{
public:
  using BackhoeChangePose = tms_msg_ts::action::TmsTsBackhoeChangePose;
  using GoalHandleBackhoeChangePose = rclcpp_action::ServerGoalHandle<BackhoeChangePose>;

  explicit BackhoeChangePoseActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("backhoe_change_pose_action_server", options)
    , planning_group("manipulator")  // TODO: Fix not to set planning_group here
    , excavator_ik_("/home/common/pwri_ws/src/zx200_ros2/zx200_description/urdf/zx200.urdf")  // TODO: Fix not to set
                                                                                              // urdf_path here

  {
    /* Create server */
    RCLCPP_INFO(this->get_logger(), "Create server.");  // debug
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<BackhoeChangePose>(
        this, "backhoe_change_pose", std::bind(&BackhoeChangePoseActionServer::handle_goal, this, _1, _2),
        std::bind(&BackhoeChangePoseActionServer::handle_cancel, this, _1),
        std::bind(&BackhoeChangePoseActionServer::handle_accepted, this, _1));
    /****/

    /* Setup movegroup interface */
    // thisをshared_ptrに変換する良さげな方法が見つからないため、渋々ノード生成
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    move_group_node_ = rclcpp::Node::make_shared(std::string(this->get_name()) + "_move_group");

    // robot の状態監視のため
    executor.add_node(move_group_node_);
    std::thread([this]() { executor.spin(); }).detach();

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Get robot info
    joint_names_ = move_group_->getJointNames();

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
  rclcpp_action::Server<BackhoeChangePose>::SharedPtr action_server_;
  rclcpp::Node::SharedPtr move_group_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  const std::string planning_group;
  rclcpp::executors::SingleThreadedExecutor executor;
  // std::vector<double> current_joint_values_;
  std::vector<std::string> joint_names_;
  std::map<std::string, double> current_joint_values_;
  // std::map<std::string, double> target_joint_values_;
  ExcavatorIK excavator_ik_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const BackhoeChangePose::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleBackhoeChangePose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleBackhoeChangePose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "handle_accepted() start.");
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&BackhoeChangePoseActionServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<GoalHandleBackhoeChangePose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<BackhoeChangePose::Feedback>();
    auto result = std::make_shared<BackhoeChangePose::Result>();

    feedback->state = "IDLE";
    goal_handle->publish_feedback(feedback);

    // Get current joint values
    std::vector<double> joint_values = move_group_->getCurrentJointValues();
    for (size_t i = 0; i < joint_names_.size() && i < joint_values.size(); i++)
    {
      current_joint_values_[joint_names_[i]] = joint_values[i];
      // target_joint_values_[joint_names_[i]] = joint_values[i];
    }

    // Planning
    // TODO: Fix to connect wayponts smoothly
    if (goal->trajectory.points.size() > 0)
    {
      for (const auto& point : goal->trajectory.points)
      {
        std::map<std::string, double> target_joint_values;
        for (size_t i = 0; i < goal->trajectory.joint_names.size() && i < point.positions.size(); ++i)
        {
          if (fabs(point.positions[i]) > 2.0 * M_PI)
          {
            target_joint_values[goal->trajectory.joint_names[i]] =
                current_joint_values_[goal->trajectory.joint_names[i]];
          }
          else
          {
            target_joint_values[goal->trajectory.joint_names[i]] = point.positions[i];
          }
        }
        move_group_->setJointValueTarget(target_joint_values);

        feedback->state = "PLANNING";
        goal_handle->publish_feedback(feedback);

        if (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
          feedback->state = "SUCCEEDED";
          goal_handle->publish_feedback(feedback);
          result->error_code.val = 1;
          // goal_handle->succeed(result);
        }
        else
        {  // Failed
          feedback->state = "ABORTED";
          goal_handle->publish_feedback(feedback);
          result->error_code.val = 9999;
          goal_handle->abort(result);

          break;
        }
      }
    }
    else if (goal->pose_sequence.size() > 0)
    {
      for (const auto& pose : goal->pose_sequence)
      {
        move_group_->setPoseTarget(pose);

        feedback->state = "PLANNING";
        goal_handle->publish_feedback(feedback);

        if (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
          feedback->state = "SUCCEEDED";
          goal_handle->publish_feedback(feedback);
          result->error_code.val = 1;
          // goal_handle->succeed(result);
        }
        else
        {  // Failed
          feedback->state = "ABORTED";
          goal_handle->publish_feedback(feedback);
          result->error_code.val = 9999;
          goal_handle->abort(result);

          break;
        }
      }
    }
    else if (goal->position_with_angle_sequence.size() > 0)
    {
      for (const auto& point : goal->position_with_angle_sequence)
      {
        std::vector<double> target_joint_values(joint_names_.size(), 0.0);
        if (excavator_ik_.inverseKinematics4Dof(point.position.x, point.position.y, point.position.z, point.theta_w,
                                                target_joint_values) == -1)
        {
          RCLCPP_INFO(this->get_logger(), "Failed to calculate inverse kinematics");
          feedback->state = "ABORTED";
          result->error_code.val = 9999;
          break;
        }

        move_group_->setJointValueTarget(target_joint_values);

        feedback->state = "PLANNING";
        goal_handle->publish_feedback(feedback);

        if (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
          feedback->state = "SUCCEEDED";
          goal_handle->publish_feedback(feedback);
          result->error_code.val = 1;
          // goal_handle->succeed(result);
        }
        else
        {  // Failed
          feedback->state = "ABORTED";
          goal_handle->publish_feedback(feedback);
          result->error_code.val = 9999;
          goal_handle->abort(result);

          break;
        }
      }
    }

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // feedback->state = "PLANNING";
    // goal_handle->publish_feedback(feedback);

    // bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if (success)
    // {
    //   feedback->state = "EXECUTING";
    //   goal_handle->publish_feedback(feedback);

    //   auto moveit_result = move_group_->execute(my_plan);

    //   // Check the result of the execution.
    //   if (moveit_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    //   {
    //     // Create a result message.
    //     result->error_code.val = 1;
    //     RCLCPP_INFO(this->get_logger(), "Execution succeeded");

    //     feedback->state = "SUCCEEDED";
    //     goal_handle->publish_feedback(feedback);
    //   }
    //   else
    //   {
    //     result->error_code.val = 9999;
    //     RCLCPP_ERROR(this->get_logger(), "Execution failed");

    //     feedback->state = "FAILED";
    //     goal_handle->publish_feedback(feedback);
    //     break;
    //   }
    // }
    // else
    // {
    //   feedback->state = "ABORTED";
    //   goal_handle->publish_feedback(feedback);
    // }

    // If execution was successful, set the result of the action and mark it as succeeded.
    if (result->error_code.val == 1)
    {
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      goal_handle->succeed(result);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal aborted");
      goal_handle->abort(result);
    }
  }
};
}  // namespace tms_if_for_opera

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tms_if_for_opera::BackhoeChangePoseActionServer>());
  rclcpp::shutdown();
  return 0;
}