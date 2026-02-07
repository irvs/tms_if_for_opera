#include "tms_if_for_opera/Excavator/excavator_assist_excavation_pose_to_joint_angles.hpp"

#include <moveit_msgs/srv/get_position_ik.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>

using namespace tms_if_for_opera;

ExcavatorAssistExcavationPoseToJointAnglesActionServer::ExcavatorAssistExcavationPoseToJointAnglesActionServer(const rclcpp::NodeOptions& options)
  : Node("tms_if_for_opera_excavator_assist_excavation_pose_to_joint_angles", options)
{
  this->declare_parameter<std::string>("planning_group", "");
  this->get_parameter("planning_group", planning_group_);
  RCLCPP_INFO(this->get_logger(), "Planning group: %s", planning_group_.c_str());

  std::string namespace_param = this->get_namespace();
  RCLCPP_INFO(this->get_logger(), "Node namespace: %s", namespace_param.c_str());

  /* Create action server */
  RCLCPP_INFO(this->get_logger(), "Creating action server...");
  using namespace std::placeholders;

  action_server_ = rclcpp_action::create_server<ExcavatorAssist>(
      this, "tms_rp_excavator_assist_excavation_pose_to_joint_angles", 
      std::bind(&ExcavatorAssistExcavationPoseToJointAnglesActionServer::handle_goal, this, _1, _2),
      std::bind(&ExcavatorAssistExcavationPoseToJointAnglesActionServer::handle_cancel, this, _1),
      std::bind(&ExcavatorAssistExcavationPoseToJointAnglesActionServer::handle_accepted, this, _1));

  /* Setup MoveGroup interface */
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  
  // 親ノードのパラメータをmove_group_node_にコピー
  auto param_names = this->list_parameters({}, 0).names;
  for (const auto& param_name : param_names)
  {
    rclcpp::Parameter param = this->get_parameter(param_name);
    node_options.append_parameter_override(param_name, param.get_parameter_value());
  }
  
  move_group_node_ = rclcpp::Node::make_shared(std::string(this->get_name()) + "_move_group", namespace_param, node_options);

  // Robot状態監視のため
  executor_.add_node(move_group_node_);
  std::thread([this]() { executor_.spin(); }).detach();

  move_group_options_ = std::make_shared<moveit::planning_interface::MoveGroupInterface::Options>(
      planning_group_, "robot_description", namespace_param);
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, *move_group_options_);

  RCLCPP_INFO(this->get_logger(), "Action server ready.");
}

rclcpp_action::GoalResponse ExcavatorAssistExcavationPoseToJointAnglesActionServer::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const ExcavatorAssist::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExcavatorAssistExcavationPoseToJointAnglesActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleExcavatorAssist> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExcavatorAssistExcavationPoseToJointAnglesActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleExcavatorAssist> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Goal accepted, starting execution...");
  using namespace std::placeholders;
  std::thread{ std::bind(&ExcavatorAssistExcavationPoseToJointAnglesActionServer::execute, this, _1), goal_handle }.detach();
}

void ExcavatorAssistExcavationPoseToJointAnglesActionServer::execute(
    const std::shared_ptr<GoalHandleExcavatorAssist> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  move_group_->setGoalPositionTolerance(0.1);
  move_group_->setGoalOrientationTolerance(0.1);

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ExcavatorAssist::Feedback>();
  auto result = std::make_shared<ExcavatorAssist::Result>();

  feedback->state = "PROCESSING";
  goal_handle->publish_feedback(feedback);

  // 入力データの取得
  double x = goal->position_with_angle.position.x;
  double y = goal->position_with_angle.position.y;
  double z = goal->position_with_angle.position.z;
  double theta_w = goal->position_with_angle.theta_w;

  RCLCPP_INFO(this->get_logger(), "Target: x=%.3f, y=%.3f, z=%.3f, theta_w=%.3f", x, y, z, theta_w);

  // pose_converterを使ってxyz, theta_wを7つの変数（xyz + quaternion）に変換
  Pose target_pose;
  pose_converter_.convertToXYZQuaternion(x, y, z, theta_w, target_pose);

  RCLCPP_INFO(this->get_logger(), "Converted to pose: [%.3f, %.3f, %.3f] quat: [%.3f, %.3f, %.3f, %.3f]", 
              target_pose.x, target_pose.y, target_pose.z, 
              target_pose.qx, target_pose.qy, target_pose.qz, target_pose.qw);

  // 目標姿勢を設定
  geometry_msgs::msg::Pose target_pose_msg;
  target_pose_msg.position.x = target_pose.x;
  target_pose_msg.position.y = target_pose.y;
  target_pose_msg.position.z = target_pose.z;
  target_pose_msg.orientation.x = target_pose.qx;
  target_pose_msg.orientation.y = target_pose.qy;
  target_pose_msg.orientation.z = target_pose.qz;
  target_pose_msg.orientation.w = target_pose.qw;

  // setPoseTargetを使用してMoveGroupにプランニングさせる
  RCLCPP_INFO(this->get_logger(), "Setting pose target...");
  bool set_target_success = move_group_->setPoseTarget(target_pose_msg);
  
  if (!set_target_success)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to set pose target");
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Planning to target pose...");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::MoveItErrorCode plan_result = move_group_->plan(plan);
  
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan to target position (error: %d)", plan_result.val);
    RCLCPP_ERROR(this->get_logger(), "Target pose: pos=[%.3f, %.3f, %.3f] orient=[%.3f, %.3f, %.3f, %.3f]",
                 target_pose_msg.position.x, target_pose_msg.position.y, target_pose_msg.position.z,
                 target_pose_msg.orientation.x, target_pose_msg.orientation.y, 
                 target_pose_msg.orientation.z, target_pose_msg.orientation.w);
    
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Planning succeeded!");

  // プランから最終的な関節値を取得
  const auto& trajectory = plan.trajectory_.joint_trajectory;
  if (trajectory.points.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Planned trajectory is empty");
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
    goal_handle->abort(result);
    return;
  }

  // 最後のウェイポイントから関節値を取得
  const auto& last_point = trajectory.points.back();
  std::vector<double> joint_values = last_point.positions;
  const std::vector<std::string>& joint_names = trajectory.joint_names;
  
  RCLCPP_INFO(this->get_logger(), "Goal position joint values:");
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(this->get_logger(), "  %s: %.6f rad (%.2f deg)", 
                joint_names[i].c_str(), joint_values[i], joint_values[i] * 180.0 / M_PI);
  }

  // arm_jointのインデックスを見つける
  int arm_joint_idx = -1;
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    if (joint_names[i] == "arm_joint")
    {
      arm_joint_idx = i;
      break;
    }
  }

  if (arm_joint_idx == -1)
  {
    RCLCPP_ERROR(this->get_logger(), "arm_joint not found in trajectory");
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
    goal_handle->abort(result);
    return;
  }

  double original_arm_angle = joint_values[arm_joint_idx];
  std::vector<double> original_joint_values = joint_values;
  
  RCLCPP_INFO(this->get_logger(), "Original arm_joint angle: %.3f rad (%.1f deg)", 
              original_arm_angle, original_arm_angle * 180.0 / M_PI);

  // 現在のロボット状態を取得
  moveit::core::RobotStatePtr current_state_for_search = move_group_->getCurrentState(10.0);
  if (!current_state_for_search)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get current robot state");
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
    goal_handle->abort(result);
    return;
  }

  // arm_jointの最大値を取得
  const moveit::core::JointModel* arm_joint_model = 
      current_state_for_search->getRobotModel()->getJointModel("arm_joint");
  
  if (!arm_joint_model)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get arm_joint model");
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
    goal_handle->abort(result);
    return;
  }

  const moveit::core::JointModel::Bounds& arm_joint_bounds = arm_joint_model->getVariableBounds();
  double arm_joint_max_limit = arm_joint_bounds[0].max_position_;
  double arm_joint_min_limit = arm_joint_bounds[0].min_position_;
  
  RCLCPP_INFO(this->get_logger(), "arm_joint limits: min=%.3f rad (%.1f deg), max=%.3f rad (%.1f deg)", 
              arm_joint_min_limit, arm_joint_min_limit * 180.0 / M_PI,
              arm_joint_max_limit, arm_joint_max_limit * 180.0 / M_PI);

  // 2分探索でarm_jointの角度をできるだけ大きくする（引く）
  // プランニングが通る最大のarm_joint角度を探す
  double search_min = original_arm_angle;      // 元の角度
  double search_max = arm_joint_max_limit;     // 関節の最大値
  double search_precision = 0.02;              // 約1.1度の精度
  double best_arm_angle = original_arm_angle;
  std::vector<double> best_joint_values = original_joint_values;

  RCLCPP_INFO(this->get_logger(), "Starting binary search for arm_joint angle (range: %.3f to %.3f rad)", 
              search_min, search_max);
  RCLCPP_INFO(this->get_logger(), "Maximum possible retraction: %.3f rad (%.1f deg)",
              search_max - search_min, (search_max - search_min) * 180.0 / M_PI);

  // 2分探索
  int iteration = 0;
  while (search_max - search_min > search_precision)
  {
    double mid = (search_min + search_max) / 2.0;
    iteration++;

    RCLCPP_INFO(this->get_logger(), "Iteration %d: Testing arm_joint angle %.3f rad (%.1f deg)", 
                iteration, mid, mid * 180.0 / M_PI);

    // arm_jointの角度を変更した目標関節値を作成
    std::vector<double> test_joint_values = original_joint_values;
    test_joint_values[arm_joint_idx] = mid;

    // 目標関節値を設定
    move_group_->setJointValueTarget(test_joint_values);

    // プランニングを実行
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode plan_result = move_group_->plan(plan);

    if (plan_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      // プランが成功した場合、さらにアームを引けるか試す
      best_arm_angle = mid;
      best_joint_values = test_joint_values;
      search_min = mid;
      
      RCLCPP_INFO(this->get_logger(), "  -> Planning succeeded! Retracted arm_joint by %.3f rad (%.1f deg)", 
                  mid - original_arm_angle, (mid - original_arm_angle) * 180.0 / M_PI);
    }
    else
    {
      // プランが失敗した場合、引きすぎなので範囲を狭める
      search_max = mid;
      RCLCPP_DEBUG(this->get_logger(), "  -> Planning failed (error: %d)", plan_result.val);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Binary search completed in %d iterations", iteration);
  RCLCPP_INFO(this->get_logger(), "Best arm_joint angle: %.3f rad (%.1f deg)", 
              best_arm_angle, best_arm_angle * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "Arm retraction: %.3f rad (%.1f deg)", 
              best_arm_angle - original_arm_angle, 
              (best_arm_angle - original_arm_angle) * 180.0 / M_PI);

  // 結果を複数の関節角度として返す（元の姿勢と引いた姿勢の2つ）
  result->joint_values.resize(2);

  // 1つ目: 元の位置のIK解
  result->joint_values[0].joint_names = joint_names;
  result->joint_values[0].joint_values = original_joint_values;
  
  RCLCPP_INFO(this->get_logger(), "Original position joint values:");
  for (size_t i = 0; i < result->joint_values[0].joint_names.size(); ++i)
  {
    RCLCPP_INFO(this->get_logger(), "  %s: %.6f rad (%.2f deg)", 
                result->joint_values[0].joint_names[i].c_str(), 
                result->joint_values[0].joint_values[i],
                result->joint_values[0].joint_values[i] * 180.0 / M_PI);
  }

  // 2つ目: アームを引いた位置の関節値
  result->joint_values[1].joint_names = joint_names;
  result->joint_values[1].joint_values = best_joint_values;

  RCLCPP_INFO(this->get_logger(), "Retracted position joint values:");
  for (size_t i = 0; i < result->joint_values[1].joint_names.size(); ++i)
  {
    RCLCPP_INFO(this->get_logger(), "  %s: %.6f rad (%.2f deg)", 
                result->joint_values[1].joint_names[i].c_str(), 
                result->joint_values[1].joint_values[i],
                result->joint_values[1].joint_values[i] * 180.0 / M_PI);
  }

  // === 実際にロボットを動かす ===
  
  // 1. 元の位置に移動
  RCLCPP_INFO(this->get_logger(), "Executing motion to original position...");
  move_group_->setJointValueTarget(original_joint_values);
  
  moveit::planning_interface::MoveGroupInterface::Plan plan_original;
  moveit::core::MoveItErrorCode plan_result_original = move_group_->plan(plan_original);
  
  if (plan_result_original != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan to original position (error: %d)", plan_result_original.val);
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
    goal_handle->abort(result);
    return;
  }
  
  moveit::core::MoveItErrorCode execute_result_original = move_group_->execute(plan_original);
  if (execute_result_original != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute motion to original position (error: %d)", execute_result_original.val);
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
    goal_handle->abort(result);
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Successfully moved to original position");
  
  // 2. 引いた位置に移動
  RCLCPP_INFO(this->get_logger(), "Executing motion to retracted position...");
  move_group_->setJointValueTarget(best_joint_values);
  
  moveit::planning_interface::MoveGroupInterface::Plan plan_retracted;
  moveit::core::MoveItErrorCode plan_result_retracted = move_group_->plan(plan_retracted);
  
  if (plan_result_retracted != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan to retracted position (error: %d)", plan_result_retracted.val);
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
    goal_handle->abort(result);
    return;
  }
  
  moveit::core::MoveItErrorCode execute_result_retracted = move_group_->execute(plan_retracted);
  if (execute_result_retracted != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute motion to retracted position (error: %d)", execute_result_retracted.val);
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
    goal_handle->abort(result);
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Successfully moved to retracted position");

  // 成功
  feedback->state = "SUCCEEDED";
  goal_handle->publish_feedback(feedback);
  result->error_code.val = 1;

  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Goal succeeded with %zu joint value sets", result->joint_values.size());
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExcavatorAssistExcavationPoseToJointAnglesActionServer>());
  rclcpp::shutdown();
  return 0;
}