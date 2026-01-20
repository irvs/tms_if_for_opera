#ifndef EXCAVATOR_CHANGE_POSE_EXECUTE_FROM_PLAN_HPP_
#define EXCAVATOR_CHANGE_POSE_EXECUTE_FROM_PLAN_HPP_

#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"

#include "tms_msg_rp/action/tms_rp_excavator_change_pose_execute.hpp"

/** Moveit! **/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_msgs/msg/link_padding.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>

#include <moveit/macros/console_colors.h>
#include <moveit/robot_state/robot_state.h>
/*****/

#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>

#include <fstream>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/string.hpp>

namespace tms_if_for_opera
{
class ExcavatorChangePoseExecuteFromPlanActionServer : public rclcpp::Node
{
public:
  using ExcavatorChangePoseExecuteFromPlan = tms_msg_rp::action::TmsRpExcavatorChangePoseExecute;
  using GoalHandleExcavatorChangePoseExecuteFromPlan = rclcpp_action::ServerGoalHandle<ExcavatorChangePoseExecuteFromPlan>;

  explicit ExcavatorChangePoseExecuteFromPlanActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  std::string planning_group_;
  std::string robot_description_;

  rclcpp_action::Server<ExcavatorChangePoseExecuteFromPlan>::SharedPtr action_server_;
  rclcpp::Node::SharedPtr move_group_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Options> move_group_options_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::vector<std::string> joint_names_;
  moveit::core::RobotStatePtr robot_state_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emg_stop_publisher_;  // for emg stop

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const ExcavatorChangePoseExecuteFromPlan::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExcavatorChangePoseExecuteFromPlan> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleExcavatorChangePoseExecuteFromPlan> goal_handle);
  void execute(const std::shared_ptr<GoalHandleExcavatorChangePoseExecuteFromPlan> goal_handle);

  double getDoubleValue(const bsoncxx::document::element& element);
};
}  // namespace tms_if_for_opera

#endif  // EXCAVATOR_CHANGE_POSE_EXECUTE_FROM_PLAN_HPP_