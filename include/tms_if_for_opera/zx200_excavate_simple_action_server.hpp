#ifndef ZX200_EXCAVATE_SIMPLE_ACTION_SERVER_HPP_
#define ZX200_EXCAVATE_SIMPLE_ACTION_SERVER_HPP_

#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "shape_msgs/msg/solid_primitive.hpp"
// #include "rclcpp_components/register_node_macro.hpp"

#include "tms_if_for_opera/visibility_control.h"
#include "tms_msg_rp/action/tms_rp_zx200_excavate_simple.hpp"

/** Moveit! **/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>
#include <moveit/robot_state/robot_state.h>
/*****/

#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>

// #include "excavator_ik/excavator_ik.hpp"

namespace tms_if_for_opera
{
class Zx200ExcavateSimpleActionServer : public rclcpp::Node
{
public:
  using Zx200ExcavateSimple = tms_msg_rp::action::TmsRpZx200ExcavateSimple;
  using GoalHandleZx200ExcavateSimple = rclcpp_action::ServerGoalHandle<Zx200ExcavateSimple>;

  explicit Zx200ExcavateSimpleActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  std::string planning_group_;
  std::string robot_description_;
  std::string collision_object_component_name_;

  rclcpp_action::Server<Zx200ExcavateSimple>::SharedPtr action_server_;
  rclcpp::Node::SharedPtr move_group_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  // std::vector<double> current_joint_values_;
  std::vector<std::string> joint_names_;
  std::map<std::string, double> current_joint_values_;
  // std::map<std::string, double> target_joint_values_;
  // ExcavatorIK excavator_ik_;
  moveit::core::RobotStatePtr robot_state_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const Zx200ExcavateSimple::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleZx200ExcavateSimple> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleZx200ExcavateSimple> goal_handle);
  void execute(const std::shared_ptr<GoalHandleZx200ExcavateSimple> goal_handle);

  void apply_collision_objects_from_db(const std::string& component_name);
  double getDoubleValue(const bsoncxx::document::element& element);
};
}  // namespace tms_if_for_opera

#endif  // ZX200_EXCAVATE_SIMPLE_ACTION_SERVER_HPP_