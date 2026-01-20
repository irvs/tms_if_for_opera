#include "tms_if_for_opera/Excavator/excavator_change_pose_execute_from_plan.hpp"

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

ExcavatorChangePoseExecuteFromPlanActionServer::ExcavatorChangePoseExecuteFromPlanActionServer(const rclcpp::NodeOptions& options)
  : Node("tms_if_for_opera_excavator_change_pose_execute_from_plan", options)
{
  this->declare_parameter<std::string>("planning_group", "");
  this->get_parameter("planning_group", planning_group_);
  RCLCPP_INFO(this->get_logger(), "Planning group: %s", planning_group_.c_str());

  std::string namespace_param = this->get_namespace();
  RCLCPP_INFO(this->get_logger(), "Node namespace: %s", namespace_param.c_str());

  /* Create server */
  RCLCPP_INFO(this->get_logger(), "Create server.");
  using namespace std::placeholders;

  action_server_ = rclcpp_action::create_server<ExcavatorChangePoseExecuteFromPlan>(
      this, "tms_rp_excavator_change_pose_execute_from_joint_values", 
      std::bind(&ExcavatorChangePoseExecuteFromPlanActionServer::handle_goal, this, _1, _2),
      std::bind(&ExcavatorChangePoseExecuteFromPlanActionServer::handle_cancel, this, _1),
      std::bind(&ExcavatorChangePoseExecuteFromPlanActionServer::handle_accepted, this, _1));

  /* Setup movegroup interface */
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  move_group_node_ = rclcpp::Node::make_shared(std::string(this->get_name()) + "_move_group");

  // robot の状態監視のため
  executor_.add_node(move_group_node_);
  std::thread([this]() { executor_.spin(); }).detach();

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

rclcpp_action::GoalResponse ExcavatorChangePoseExecuteFromPlanActionServer::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                                     std::shared_ptr<const ExcavatorChangePoseExecuteFromPlan::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ExcavatorChangePoseExecuteFromPlanActionServer::handle_cancel(const std::shared_ptr<GoalHandleExcavatorChangePoseExecuteFromPlan> goal_handle)
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

void ExcavatorChangePoseExecuteFromPlanActionServer::handle_accepted(const std::shared_ptr<GoalHandleExcavatorChangePoseExecuteFromPlan> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "handle_accepted() start.");
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{ std::bind(&ExcavatorChangePoseExecuteFromPlanActionServer::execute, this, _1), goal_handle }.detach();
}

void ExcavatorChangePoseExecuteFromPlanActionServer::execute(const std::shared_ptr<GoalHandleExcavatorChangePoseExecuteFromPlan> goal_handle)
{
  // Execute goal
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ExcavatorChangePoseExecuteFromPlan::Feedback>();
  auto result = std::make_shared<ExcavatorChangePoseExecuteFromPlan::Result>();

  feedback->state = "IDLE";
  goal_handle->publish_feedback(feedback);

  // planが空でないかチェック
  if (goal->plan.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Received empty plan");
    feedback->state = "FAILED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = -1;
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Received %zu plan(s) to execute", goal->plan.size());

  // 各プランを順番に実行
  for (size_t i = 0; i < goal->plan.size(); ++i) {
    const auto& robot_trajectory = goal->plan[i];
    
    RCLCPP_INFO(this->get_logger(), "Executing plan %zu/%zu", i + 1, goal->plan.size());
    RCLCPP_INFO(this->get_logger(), "  Joint trajectory points: %zu", robot_trajectory.joint_trajectory.points.size());
    RCLCPP_INFO(this->get_logger(), "  Multi-DOF trajectory points: %zu", robot_trajectory.multi_dof_joint_trajectory.points.size());

    // キャンセルチェック
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal was canceled");
      feedback->state = "CANCELED";
      goal_handle->publish_feedback(feedback);
      result->error_code.val = -1;
      goal_handle->canceled(result);
      return;
    }

    feedback->state = "EXECUTING";
    goal_handle->publish_feedback(feedback);

    // MoveIt planを作成
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = robot_trajectory;

    // プランを実行
    RCLCPP_INFO(this->get_logger(), "Starting execution of plan %zu", i + 1);
    moveit::core::MoveItErrorCode execute_result = move_group_->execute(plan);

    if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute plan %zu: %s", 
                   i + 1, 
                   moveit::core::error_code_to_string(execute_result).c_str());
      feedback->state = "FAILED";
      goal_handle->publish_feedback(feedback);
      result->error_code.val = execute_result.val;
      goal_handle->abort(result);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully executed plan %zu", i + 1);
  }

  // 成功
  RCLCPP_INFO(this->get_logger(), "All plans executed successfully");
  feedback->state = "SUCCEEDED";
  goal_handle->publish_feedback(feedback);
  result->error_code.val = 1;
  
  // Succeed the action
  goal_handle->succeed(result);
}

double ExcavatorChangePoseExecuteFromPlanActionServer::getDoubleValue(const bsoncxx::document::element& element)
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
  rclcpp::spin(std::make_shared<ExcavatorChangePoseExecuteFromPlanActionServer>());
  rclcpp::shutdown();
  return 0;
}