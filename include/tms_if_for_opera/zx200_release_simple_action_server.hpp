#ifndef ZX200_RELEASE_SIMPLE_ACTION_SERVER_HPP_
#define ZX200_RELEASE_SIMPLE_ACTION_SERVER_HPP_

#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
// #include "shape_msgs/msg/solid_primitive.hpp"
// #include "rclcpp_components/register_node_macro.hpp"

#include "tms_if_for_opera/visibility_control.h"
#include "tms_msg_rp/action/tms_rp_zx200_release_simple.hpp"

/** Moveit! **/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_msgs/msg/link_padding.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

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

// #include "tms_if_for_opera/excavator_ik.hpp"

#include <fstream>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/string.hpp>

namespace tms_if_for_opera
{
class Zx200ReleaseSimpleActionServer : public rclcpp::Node
{
public:
  using Zx200ReleaseSimple = tms_msg_rp::action::TmsRpZx200ReleaseSimple;
  using GoalHandleZx200ReleaseSimple = rclcpp_action::ServerGoalHandle<Zx200ReleaseSimple>;

  explicit Zx200ReleaseSimpleActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  std::string planning_group_;
  std::string robot_description_;
  std::string collision_object_record_name_;
  std::vector<std::string> collision_object_dump_record_name_;

  rclcpp_action::Server<Zx200ReleaseSimple>::SharedPtr action_server_;
  rclcpp::Node::SharedPtr move_group_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Options> move_group_options_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  // std::vector<double> current_joint_values_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  std::map<std::string, double> current_joint_values_;
  // std::map<std::string, double> target_joint_values_;
  // ExcavatorIK excavator_ik_;
  moveit::core::RobotStatePtr robot_state_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emg_stop_publisher_;  // for emg stop

  // TF2関連
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // 複数ロボットのrobot_description管理
  std::map<std::string, std::string> other_robot_descriptions_;
  std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> robot_description_subs_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const Zx200ReleaseSimple::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleZx200ReleaseSimple> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleZx200ReleaseSimple> goal_handle);
  void execute(const std::shared_ptr<GoalHandleZx200ReleaseSimple> goal_handle);

  void apply_collision_objects_from_db(const std::string& record_name);
  void apply_collision_objects_mesh_from_db(const std::vector<std::string>& record_names);
  double getDoubleValue(const bsoncxx::document::element& element);

  bool load_urdf_from_file(const std::string& robot_name);
  void apply_collision_objects_from_robot_description_and_tf(
    const std::string& other_robot_base_frame, 
    const std::string& collision_object_prefix);
};
}  // namespace tms_if_for_opera

#endif  // ZX200_RELEASE_SIMPLE_ACTION_SERVER_HPP_