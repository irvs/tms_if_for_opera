#include "tms_if_for_opera/Excavator/excavator_change_pose_plan_from_poses.hpp"

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
  : Node("tms_if_for_opera_excavator_change_pose_from_poses", options)
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
      this, "tms_rp_excavator_change_pose_plan_from_poses", std::bind(&ExcavatorChangePoseFromPoseActionServer::handle_goal, this, _1, _2),
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
  move_group_->setGoalPositionTolerance(0.1);
  move_group_->setGoalOrientationTolerance(0.1);
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
  RCLCPP_INFO(this->get_logger(), "Executing goal (Plan generation only)");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ExcavatorChangePoseFromPose::Feedback>();
  auto result = std::make_shared<ExcavatorChangePoseFromPose::Result>();

  feedback->state = "IDLE";
  goal_handle->publish_feedback(feedback);

  // 入力パラメータのサイズをチェック
  if (goal->position_with_angle_sequence.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "No waypoints provided");
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Number of waypoints: %zu", goal->position_with_angle_sequence.size());

  // 全ての座標変換を先に実行
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  RCLCPP_INFO(this->get_logger(), "Converting all waypoints to MoveIt poses");
  for (size_t i = 0; i < goal->position_with_angle_sequence.size(); ++i)
  {
    const auto& point = goal->position_with_angle_sequence[i];
    RCLCPP_INFO(this->get_logger(), "Waypoint %zu: x=%.6f, y=%.6f, z=%.6f, theta_w=%.6f", 
                i, point.position.x, point.position.y, point.position.z, point.theta_w);

    Pose target_pose;
    geometry_msgs::msg::Pose moveit_pose;
    pose_converter_.convertToXYZQuaternion(point.position.x, point.position.y, 
                                           point.position.z, point.theta_w, target_pose);
    moveit_pose.position.x = target_pose.x;
    moveit_pose.position.y = target_pose.y;
    moveit_pose.position.z = target_pose.z;
    moveit_pose.orientation.x = target_pose.qx;
    moveit_pose.orientation.y = target_pose.qy;
    moveit_pose.orientation.z = target_pose.qz;
    moveit_pose.orientation.w = target_pose.qw;

    waypoints.push_back(moveit_pose);
  }

  move_group_->clearPathConstraints();
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
  }

  // Planning sceneの初期化: 既存のcollision objectsをクリア
  RCLCPP_INFO(this->get_logger(), "Initializing planning scene");
  std::map<std::string, moveit_msgs::msg::CollisionObject> known_objects = planning_scene_interface_.getObjects();
  RCLCPP_INFO(this->get_logger(), "Current planning scene has %zu collision objects", known_objects.size());
  if (!known_objects.empty())
  {
    std::vector<std::string> object_ids;
    for (const auto& obj : known_objects)
    {
      object_ids.push_back(obj.first);
    }
    planning_scene_interface_.removeCollisionObjects(object_ids);
    RCLCPP_INFO(this->get_logger(), "Cleared %zu collision objects", object_ids.size());
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  RCLCPP_INFO(this->get_logger(), "Clearing octomap");
  moveit_msgs::msg::PlanningScene clear_octomap_scene;
  clear_octomap_scene.is_diff = true;
  clear_octomap_scene.world.octomap.octomap.header.frame_id = move_group_->getPlanningFrame();
  clear_octomap_scene.world.octomap.octomap.binary = true;
  clear_octomap_scene.world.octomap.octomap.id = "OcTree";
  clear_octomap_scene.world.octomap.octomap.data.clear();
  planning_scene_interface_.applyPlanningScene(clear_octomap_scene);
  RCLCPP_INFO(this->get_logger(), "Octomap cleared");
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  // 新しいplanning sceneを適用（goalで指定されている場合）
  if (!goal->planning_scene.world.collision_objects.empty() ||
      !goal->planning_scene.world.octomap.octomap.data.empty() ||
      !goal->planning_scene.link_padding.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Applying new planning scene (collision_objects: %zu, octomap: %s, link_padding: %zu)",
                goal->planning_scene.world.collision_objects.size(),
                goal->planning_scene.world.octomap.octomap.data.empty() ? "empty" : "provided",
                goal->planning_scene.link_padding.size());
    planning_scene_interface_.applyPlanningScene(goal->planning_scene);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "No new planning scene provided");
  }

  // 最初の点へのPlan生成（実行はしない）
  RCLCPP_INFO(this->get_logger(), "Planning to first waypoint");
  move_group_->setPoseTarget(waypoints[0]);

  feedback->state = "PLANNING";
  goal_handle->publish_feedback(feedback);

  // Plan to first waypoint
  moveit::planning_interface::MoveGroupInterface::Plan first_plan;
  bool success = (move_group_->plan(first_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success)
  {
    RCLCPP_ERROR(this->get_logger(), "Planning to first waypoint failed");
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Planning to first waypoint succeeded");

  // 最初のPlanをresultに追加
  result->plan.push_back(first_plan.trajectory_);

  // 2点目以降がある場合、Cartesian Pathで計画
  if (waypoints.size() > 1)
  {
    RCLCPP_INFO(this->get_logger(), "Planning Cartesian path for remaining waypoints");

    // Cartesian Pathを計算
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;  // 1cm刻み
    const double jump_threshold = 0.0;  // ジャンプ閾値（0.0で無効化）
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(this->get_logger(), "Cartesian path planned (%.2f%% achieved)", fraction * 100.0);

    if (fraction < 0.95)  // 95%未満の場合はエラー
    {
      RCLCPP_ERROR(this->get_logger(), "Cartesian path planning failed: only %.2f%% of path achieved", fraction * 100.0);
      feedback->state = "ABORTED";
      goal_handle->publish_feedback(feedback);
      result->error_code.val = 9999;
      goal_handle->abort(result);
      return;
    }

    // Cartesian PathをresultにPlanとして追加
    result->plan.push_back(trajectory);
    RCLCPP_INFO(this->get_logger(), "Cartesian path planning succeeded");
  }

  // 成功（Planのみ生成、実行はしない）
  feedback->state = "SUCCEEDED";
  goal_handle->publish_feedback(feedback);
  result->error_code.val = 1;
  
  RCLCPP_INFO(this->get_logger(), "Plan generation completed. Total plans: %zu", result->plan.size());
  
  // Succeed the action
  goal_handle->succeed(result);
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