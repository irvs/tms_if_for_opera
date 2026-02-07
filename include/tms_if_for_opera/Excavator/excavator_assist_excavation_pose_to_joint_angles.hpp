#ifndef EXCAVATOR_ASSIST_EXCAVATION_POSE_TO_JOINT_ANGLES_HPP_
#define EXCAVATOR_ASSIST_EXCAVATION_POSE_TO_JOINT_ANGLES_HPP_

#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tms_msg_rp/action/tms_rp_excavator_assist.hpp"
#include "tms_msg_rp/msg/tms_rp_excavator_joint_values.hpp"
#include "tms_msg_rp/msg/tms_rp_excavator_position_with_angle.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "tms_if_for_opera/Excavator/lib/excavator_pose_converter.hpp"

namespace tms_if_for_opera
{
class ExcavatorAssistExcavationPoseToJointAnglesActionServer : public rclcpp::Node
{
public:
  using ExcavatorAssist = tms_msg_rp::action::TmsRpExcavatorAssist;
  using GoalHandleExcavatorAssist = rclcpp_action::ServerGoalHandle<ExcavatorAssist>;

  explicit ExcavatorAssistExcavationPoseToJointAnglesActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  std::string planning_group_;

  rclcpp_action::Server<ExcavatorAssist>::SharedPtr action_server_;
  rclcpp::Node::SharedPtr move_group_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Options> move_group_options_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  
  ExcavatorPoseConverter pose_converter_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const ExcavatorAssist::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExcavatorAssist> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleExcavatorAssist> goal_handle);
  void execute(const std::shared_ptr<GoalHandleExcavatorAssist> goal_handle);
};
}  // namespace tms_if_for_opera

#endif  // EXCAVATOR_ASSIST_EXCAVATION_POSE_TO_JOINT_ANGLES_HPP_