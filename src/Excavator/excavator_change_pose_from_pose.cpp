#include "tms_if_for_opera/Excavator/excavator_change_pose_from_pose.hpp"

// #include <moveit_msgs/msg/constraints.hpp>
// #include <moveit_msgs/msg/orientation_constraint.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <urdf/model.h>
#include <geometric_shapes/shape_operations.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
using std::cout;

using namespace tms_if_for_opera;

ExcavatorChangePoseFromPoseActionServer::ExcavatorChangePoseFromPoseActionServer(const rclcpp::NodeOptions& options)
  : Node("tms_if_for_opera_excavator_change_pose_from_pose", options)
{
  this->declare_parameter<std::string>("robot_description", "");
  this->get_parameter("robot_description", robot_description_);
  RCLCPP_INFO(this->get_logger(), "Robot description: %s", robot_description_.c_str());
  excavator_ik_.loadURDF(robot_description_);

  this->declare_parameter<std::string>("planning_group", "");
  this->get_parameter("planning_group", planning_group_);
  RCLCPP_INFO(this->get_logger(), "Planning group: %s", planning_group_.c_str());

  std::string namespace_param = this->get_namespace();
  RCLCPP_INFO(this->get_logger(), "Node namespace: %s", namespace_param.c_str());

  this->declare_parameter<std::string>("collision_object_record_name", "");
  this->get_parameter("collision_object_record_name", collision_object_record_name_);
  RCLCPP_INFO(this->get_logger(), "Collision object record name: %s", collision_object_record_name_.c_str());

  this->declare_parameter<std::string>("collision_object_dump_record_name", "");
  std::string dump_record_names_str;
  this->get_parameter("collision_object_dump_record_name", dump_record_names_str);
  
  // コンマ区切りの文字列を配列に変換
  if (!dump_record_names_str.empty()) {
    std::stringstream ss(dump_record_names_str);
    std::string item;
    while (std::getline(ss, item, ',')) {
      // 前後の空白を削除
      item.erase(0, item.find_first_not_of(" \t"));
      item.erase(item.find_last_not_of(" \t") + 1);
      if (!item.empty()) {
        collision_object_dump_record_name_.push_back(item);
      }
    }
  }
  RCLCPP_INFO(this->get_logger(), "Collision object dump record name count: %zu", collision_object_dump_record_name_.size());

  /* Create server */
  RCLCPP_INFO(this->get_logger(), "Create server.");  // debug
  using namespace std::placeholders;

  action_server_ = rclcpp_action::create_server<ExcavatorChangePoseFromPose>(
      this, "tms_rp_excavator_change_pose_from_pose", std::bind(&ExcavatorChangePoseFromPoseActionServer::handle_goal, this, _1, _2),
      std::bind(&ExcavatorChangePoseFromPoseActionServer::handle_cancel, this, _1),
      std::bind(&ExcavatorChangePoseFromPoseActionServer::handle_accepted, this, _1));
  /****/

  /* Setup movegroup interface */
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  move_group_node_ = rclcpp::Node::make_shared(std::string(this->get_name()) + "_move_group");

  // robot の状態監視のため
  executor_.add_node(move_group_node_);
  std::thread([this]() { executor_.spin(); }).detach();

  // move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, planning_group_);
  move_group_options_ = std::make_shared<moveit::planning_interface::MoveGroupInterface::Options>(planning_group_, "robot_description", namespace_param);
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, *move_group_options_);

  move_group_->setMaxVelocityScalingFactor(1.0);
  move_group_->setMaxAccelerationScalingFactor(1.0);
  move_group_->setNumPlanningAttempts(100);
  move_group_->setPlanningTime(60.0);
  move_group_->setPlannerId("RRTConnectkConfigDefault");

  // Get robot info
  joint_names_ = move_group_->getJointNames();

  // For FK
  robot_state_ = std::make_shared<moveit::core::RobotState>(move_group_->getRobotModel());

  // Init DB connection
  mongocxx::instance instance{};

  // For emg stop
  this->emg_stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>("emg_stop", 10);
}

rclcpp_action::GoalResponse ExcavatorChangePoseFromPoseActionServer::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                                     std::shared_ptr<const ExcavatorChangePoseFromPose::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ExcavatorChangePoseFromPoseActionServer::handle_cancel(const std::shared_ptr<GoalHandleExcavatorChangePoseFromPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Publishing EMG stop signal to Excavator.");

  // 実機用非常停止
  std_msgs::msg::Bool msg;
  msg.data = true;
  this->emg_stop_publisher_->publish(msg);
  // move_group停止
  move_group_->stop();

  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExcavatorChangePoseFromPoseActionServer::handle_accepted(const std::shared_ptr<GoalHandleExcavatorChangePoseFromPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "handle_accepted() start.");
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{ std::bind(&ExcavatorChangePoseFromPoseActionServer::execute, this, _1), goal_handle }.detach();
}

void ExcavatorChangePoseFromPoseActionServer::execute(const std::shared_ptr<GoalHandleExcavatorChangePoseFromPose> goal_handle)
{

  // Execute goal
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ExcavatorChangePoseFromPose::Feedback>();
  auto result = std::make_shared<ExcavatorChangePoseFromPose::Result>();

  feedback->state = "IDLE";
  goal_handle->publish_feedback(feedback);

  // 入力パラメータを取得
  const auto& point = goal->position_with_angle_sequence[0];
  double x = point.position.x;
  double y = point.position.y;
  double z = point.position.z;
  double theta_w = point.theta_w;

  RCLCPP_INFO(this->get_logger(), "Input: x=%.6f, y=%.6f, z=%.6f, theta_w=%.6f", x, y, z, theta_w);

  // Apply constraints if provided
  if (!goal->constraints.joint_constraints.empty() ||
      !goal->constraints.position_constraints.empty() ||
      !goal->constraints.orientation_constraints.empty() ||
      !goal->constraints.visibility_constraints.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Applying path constraints (joint: %zu, position: %zu, orientation: %zu, visibility: %zu)",
                goal->constraints.joint_constraints.size(),
                goal->constraints.position_constraints.size(),
                goal->constraints.orientation_constraints.size(),
                goal->constraints.visibility_constraints.size());
    move_group_->setPathConstraints(goal->constraints);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "No constraints provided, clearing existing constraints");
    move_group_->clearPathConstraints();
  }

  // Apply planning scene if provided
  if (!goal->planning_scene.name.empty() || !goal->planning_scene.world.collision_objects.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Applying planning scene");
    planning_scene_interface_.applyPlanningScene(goal->planning_scene);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "No planning scene provided");
  }

  // Convert to Pose
  Pose target_pose;
  geometry_msgs::msg::Pose moveit_pose;
  pose_converter_.convertToXYZQuaternion(x, y, z, theta_w, target_pose);
  moveit_pose.position.x = target_pose.x;
  moveit_pose.position.y = target_pose.y;
  moveit_pose.position.z = target_pose.z;
  moveit_pose.orientation.x = target_pose.qx;
  moveit_pose.orientation.y = target_pose.qy;
  moveit_pose.orientation.z = target_pose.qz;
  moveit_pose.orientation.w = target_pose.qw;

  move_group_->setPoseTarget(moveit_pose);

  feedback->state = "PLANNING";
  goal_handle->publish_feedback(feedback);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    RCLCPP_INFO(this->get_logger(), "Planning succeeded");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Planning failed");
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;

    // Clear constraints before aborting
    move_group_->clearPathConstraints();

    // Abort the action
    goal_handle->abort(result);
    return;
  }

  // Execute
  if (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    feedback->state = "SUCCEEDED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 1;
    
    // Clear constraints after success
    move_group_->clearPathConstraints();
    
    // Succeed the action
    goal_handle->succeed(result);
  }
  else
  {  // Failed
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;

    // Clear constraints before aborting
    move_group_->clearPathConstraints();

    // Abort the action
    goal_handle->abort(result);
  }

}

double ExcavatorChangePoseFromPoseActionServer::getDoubleValue(const bsoncxx::document::element& element)
{
  if (element.type() == bsoncxx::type::k_double)
  {
    return element.get_double().value;
  }
  else if (element.type() == bsoncxx::type::k_int32)
  {
    return static_cast<double>(element.get_int32().value);
  }
  else
  {
    throw std::runtime_error("Unsupported type");
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExcavatorChangePoseFromPoseActionServer>());
  rclcpp::shutdown();
  return 0;
}