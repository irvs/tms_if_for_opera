#include "tms_if_for_opera/Excavator/excavator_change_pose_plan_from_joint_values.hpp"

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

ExcavatorChangePosePlanFromJointValuesActionServer::ExcavatorChangePosePlanFromJointValuesActionServer(const rclcpp::NodeOptions& options)
  : Node("tms_if_for_opera_excavator_change_pose_plan_from_joint_values", options)
{
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

  action_server_ = rclcpp_action::create_server<ExcavatorChangePosePlanFromJointValues>(
      this, "tms_rp_excavator_change_pose_plan_from_joint_values", 
      std::bind(&ExcavatorChangePosePlanFromJointValuesActionServer::handle_goal, this, _1, _2),
      std::bind(&ExcavatorChangePosePlanFromJointValuesActionServer::handle_cancel, this, _1),
      std::bind(&ExcavatorChangePosePlanFromJointValuesActionServer::handle_accepted, this, _1));
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

rclcpp_action::GoalResponse ExcavatorChangePosePlanFromJointValuesActionServer::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                                     std::shared_ptr<const ExcavatorChangePosePlanFromJointValues::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ExcavatorChangePosePlanFromJointValuesActionServer::handle_cancel(const std::shared_ptr<GoalHandleExcavatorChangePosePlanFromJointValues> goal_handle)
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

void ExcavatorChangePosePlanFromJointValuesActionServer::handle_accepted(const std::shared_ptr<GoalHandleExcavatorChangePosePlanFromJointValues> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "handle_accepted() start.");
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{ std::bind(&ExcavatorChangePosePlanFromJointValuesActionServer::execute, this, _1), goal_handle }.detach();
}

void ExcavatorChangePosePlanFromJointValuesActionServer::execute(const std::shared_ptr<GoalHandleExcavatorChangePosePlanFromJointValues> goal_handle)
{

  // Execute goal
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ExcavatorChangePosePlanFromJointValues::Feedback>();
  auto result = std::make_shared<ExcavatorChangePosePlanFromJointValues::Result>();

  feedback->state = "IDLE";
  goal_handle->publish_feedback(feedback);

  // 入力パラメータのサイズをチェック
  if (goal->joint_values_sequence.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "No joint values provided");
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Number of joint value sets: %zu", goal->joint_values_sequence.size());

  // Apply constraints if provided
  move_group_->clearPathConstraints();
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

  feedback->state = "PLANNING";
  goal_handle->publish_feedback(feedback);

  // 各joint_valuesセットを順番に実行
  std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;
  
  // 最初のPlanは現在のロボット状態から生成するが、2番目以降は前のPlanの最終状態から生成
  moveit::core::RobotStatePtr start_state = move_group_->getCurrentState(10.0);
  if (!start_state) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get current robot state");
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
    goal_handle->abort(result);
    return;
  }
  
  for (size_t i = 0; i < goal->joint_values_sequence.size(); ++i)
  {
    const auto& joint_value = goal->joint_values_sequence[i];
    
    RCLCPP_INFO(this->get_logger(), "Moving to joint values set %zu", i);
    
    // joint_namesとjoint_valuesのサイズチェック
    if (joint_value.joint_names.size() != joint_value.joint_values.size())
    {
      RCLCPP_ERROR(this->get_logger(), "Joint names and values size mismatch at index %zu", i);
      feedback->state = "ABORTED";
      goal_handle->publish_feedback(feedback);
      result->error_code.val = 9999;
      goal_handle->abort(result);
      return;
    }
    
    // ジョイント値をログ出力
    for (size_t j = 0; j < joint_value.joint_names.size(); ++j)
    {
      RCLCPP_INFO(this->get_logger(), "  %s: %f", 
                  joint_value.joint_names[j].c_str(), 
                  joint_value.joint_values[j]);
    }
    
    // ジョイント値をマップに変換
    std::map<std::string, double> joint_values_map;
    for (size_t j = 0; j < joint_value.joint_names.size(); ++j)
    {
      joint_values_map[joint_value.joint_names[j]] = joint_value.joint_values[j];
    }
    
    // 開始状態を設定（2番目以降は前のPlanの最終状態から）
    move_group_->setStartState(*start_state);
    
    // MoveGroupにジョイント目標を設定
    move_group_->setJointValueTarget(joint_values_map);
    
    // プランニング
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (!success)
    {
      RCLCPP_ERROR(this->get_logger(), "Planning to joint values set %zu failed", i);
      feedback->state = "ABORTED";
      goal_handle->publish_feedback(feedback);
      result->error_code.val = 9999;
      goal_handle->abort(result);
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Planning to joint values set %zu succeeded", i);
    
    // デバッグ: Planの最初と最後のポイントをログ出力
    if (!plan.trajectory_.joint_trajectory.points.empty()) {
      const auto& first_point = plan.trajectory_.joint_trajectory.points.front();
      const auto& last_point = plan.trajectory_.joint_trajectory.points.back();
      
      RCLCPP_INFO(this->get_logger(), "  Plan has %zu points", plan.trajectory_.joint_trajectory.points.size());
      RCLCPP_INFO(this->get_logger(), "  First point positions:");
      for (size_t j = 0; j < first_point.positions.size() && j < plan.trajectory_.joint_trajectory.joint_names.size(); ++j) {
        RCLCPP_INFO(this->get_logger(), "    %s: %.6f", 
                    plan.trajectory_.joint_trajectory.joint_names[j].c_str(), 
                    first_point.positions[j]);
      }
      RCLCPP_INFO(this->get_logger(), "  Last point positions:");
      for (size_t j = 0; j < last_point.positions.size() && j < plan.trajectory_.joint_trajectory.joint_names.size(); ++j) {
        RCLCPP_INFO(this->get_logger(), "    %s: %.6f", 
                    plan.trajectory_.joint_trajectory.joint_names[j].c_str(), 
                    last_point.positions[j]);
      }
      
      // 次のPlanの開始状態として、このPlanの最終状態を設定
      for (size_t j = 0; j < plan.trajectory_.joint_trajectory.joint_names.size(); ++j) {
        const auto& joint_name = plan.trajectory_.joint_trajectory.joint_names[j];
        start_state->setVariablePosition(joint_name, last_point.positions[j]);
      }
      start_state->update();
    }
    
    trajectories.push_back(plan.trajectory_);
  }

  // 成功
  feedback->state = "SUCCEEDED";
  goal_handle->publish_feedback(feedback);
  result->error_code.val = 1;
  result->plan = trajectories;
  
  // Succeed the action
  goal_handle->succeed(result);
}

double ExcavatorChangePosePlanFromJointValuesActionServer::getDoubleValue(const bsoncxx::document::element& element)
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
  rclcpp::spin(std::make_shared<ExcavatorChangePosePlanFromJointValuesActionServer>());
  rclcpp::shutdown();
  return 0;
}