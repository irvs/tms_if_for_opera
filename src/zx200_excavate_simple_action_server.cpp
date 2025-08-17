#include "tms_if_for_opera/zx200_excavate_simple_action_server.hpp"

// #include <moveit_msgs/msg/constraints.hpp>
// #include <moveit_msgs/msg/orientation_constraint.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/robot_state/robot_state.h>
#include <sstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <urdf/model.h>
#include <geometric_shapes/shape_operations.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

using namespace tms_if_for_opera;

Zx200ExcavateSimpleActionServer::Zx200ExcavateSimpleActionServer(const rclcpp::NodeOptions& options)
  : Node("tms_if_for_opera_zx200_excavate_simple", options)
{
  this->declare_parameter<std::string>("robot_description", "");
  this->get_parameter("robot_description", robot_description_);
  RCLCPP_INFO(this->get_logger(), "Robot description: %s", robot_description_.c_str());
  excavator_ik_.loadURDF(robot_description_);

  this->declare_parameter<std::string>("planning_group", "");
  this->get_parameter("planning_group", planning_group_);
  RCLCPP_INFO(this->get_logger(), "Planning group: %s", planning_group_.c_str());

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

  action_server_ = rclcpp_action::create_server<Zx200ExcavateSimple>(
      this, "tms_rp_zx200_excavate_simple", std::bind(&Zx200ExcavateSimpleActionServer::handle_goal, this, _1, _2),
      std::bind(&Zx200ExcavateSimpleActionServer::handle_cancel, this, _1),
      std::bind(&Zx200ExcavateSimpleActionServer::handle_accepted, this, _1));
  /****/

  /* Setup movegroup interface */
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  move_group_node_ = rclcpp::Node::make_shared(std::string(this->get_name()) + "_move_group");

  // robot の状態監視のため
  executor_.add_node(move_group_node_);
  std::thread([this]() { executor_.spin(); }).detach();

  // move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, planning_group_);
  move_group_options_ = std::make_shared<moveit::planning_interface::MoveGroupInterface::Options>(planning_group_, "robot_description", "/zx200");
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, *move_group_options_);
  
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  move_group_->setMaxVelocityScalingFactor(1.0);
  move_group_->setMaxAccelerationScalingFactor(1.0);
  move_group_->setNumPlanningAttempts(10);
  move_group_->setPlanningTime(10.0);
  move_group_->setPlannerId("RRTConnectkConfigDefault");

  // Get robot info
  joint_names_ = move_group_->getJointNames();

  // For FK
  robot_state_ = std::make_shared<moveit::core::RobotState>(move_group_->getRobotModel());

  // Init DB connection
  mongocxx::instance instance{};

  // For emg stop
  this->emg_stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/zx200/emg_stop", 10);

  // TF2の初期化
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 複数ロボットのコリジョン設定パラメータ
  this->declare_parameter<std::string>("other_robots_config", "");
  std::string other_robots_config_str;
  this->get_parameter("other_robots_config", other_robots_config_str);

  // collision_objectsフォルダのパス設定
  this->declare_parameter<std::string>("collision_objects_base_path", "/home/common/3_SIP/tms_fujita_ws/src/tms_if_for_opera/collision_objects");
  std::string collision_objects_base_path;
  this->get_parameter("collision_objects_base_path", collision_objects_base_path);

  // 複数ロボットの設定を解析
  if (!other_robots_config_str.empty()) {
    std::stringstream ss(other_robots_config_str);
    std::string robot_name;
    while (std::getline(ss, robot_name, ',')) {
      robot_name.erase(0, robot_name.find_first_not_of(" \t"));
      robot_name.erase(robot_name.find_last_not_of(" \t") + 1);
      
      if (!robot_name.empty()) {
        if (load_urdf_from_file(robot_name)) {
          RCLCPP_INFO(this->get_logger(), "Loaded URDF for robot %s", robot_name.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to load URDF for robot %s", robot_name.c_str());
        }
      }
    }
  }
  RCLCPP_INFO(this->get_logger(), "Configured %zu other robots for collision detection", other_robot_descriptions_.size());
}

rclcpp_action::GoalResponse Zx200ExcavateSimpleActionServer::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Zx200ExcavateSimple::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Zx200ExcavateSimpleActionServer::handle_cancel(const std::shared_ptr<GoalHandleZx200ExcavateSimple> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Publishing EMG stop signal to ZX200.");

  // 実機用非常停止
  std_msgs::msg::Bool msg;
  msg.data = true;
  this->emg_stop_publisher_->publish(msg);
  // move_group停止
  move_group_->stop();

  // auto result = std::make_shared<Zx200ExcavateSimple::Result>();
  // result->error_code.val = 9999;
  // goal_handle->abort(result);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Zx200ExcavateSimpleActionServer::handle_accepted(const std::shared_ptr<GoalHandleZx200ExcavateSimple> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "handle_accepted() start.");
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{ std::bind(&Zx200ExcavateSimpleActionServer::execute, this, _1), goal_handle }.detach();
}

void Zx200ExcavateSimpleActionServer::execute(const std::shared_ptr<GoalHandleZx200ExcavateSimple> goal_handle)
{
  // Start to execute goal
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Zx200ExcavateSimple::Feedback>();
  auto result = std::make_shared<Zx200ExcavateSimple::Result>();

  // Function for error handling
  auto handle_error = [&](const std::string& message) {
    if (goal_handle->is_active())
    {
      result->error_code.val = 9999;
      goal_handle->abort(result);
      RCLCPP_INFO(this->get_logger(), message.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal is not active");
    }
  };

  // Apply collision object
  apply_collision_objects_from_db(collision_object_record_name_);
  apply_collision_objects_mesh_from_db(collision_object_dump_record_name_);
  
  // 全ての他ロボットのコリジョンを追加
  for (const auto& robot_pair : other_robot_descriptions_) {
    const std::string& robot_name = robot_pair.first;
    if (!robot_pair.second.empty()) {
      apply_collision_objects_from_robot_description_and_tf(robot_name + "_tf/base_link", robot_name);
    }
  }

  // Get link info
  link_names_ = move_group_->getLinkNames();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  // PlanningScene メッセージの作成
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;  // 差分更新
  // パディングを設定するための LinkPadding メッセージ
  moveit_msgs::msg::LinkPadding padding_msg;
  std::vector<std::string> padding_link_list = {"boom_link", "arm_link", "bucket_link", "bucket_end_link"};
  for (size_t i = 0; i < link_names_.size(); i++)
  {
      // リンク名が padding_link_list に含まれているかチェック
      if (std::find(padding_link_list.begin(), padding_link_list.end(), link_names_[i]) == padding_link_list.end())
      {
          continue;  // 含まれていなければスキップ
      }
      RCLCPP_INFO(this->get_logger(), "Link Name: %s", link_names_[i].c_str());
      // 新しく padding_msg を作成
      moveit_msgs::msg::LinkPadding padding_msg;
      padding_msg.link_name = link_names_[i];  // パディングを適用するリンク
      padding_msg.padding = 0.25;
      // パディングリストに追加
      planning_scene_msg.link_padding.push_back(padding_msg);
  }
  planning_scene_interface_.applyPlanningScene(planning_scene_msg);

  // Clear constraints
  // move_group_->clearPathConstraints();

  double radians = atan2(goal->position_with_angle.position.y, goal->position_with_angle.position.x);
  // radians = M_PI - radians;
  double offset = goal->position_with_angle.offset;
  RCLCPP_INFO(this->get_logger(), "%f", radians);


  const double step = 0.01;
  // const double theta_w = goal->position_with_angle.theta_w;
  const double theta_w = 0.0;
  const double theta_min = 0.0;
  const double theta_max = M_PI;
  std::vector<double> target_joint_values(joint_names_.size(), 0.0);

  bool found = false;
  // double theta_offset = 0.1;
  double best_theta = theta_w;

  for (double delta = 0.0; delta <= theta_max; delta += step) {
      // 1) θw + Δ を試す
      double cand1 = theta_w + delta;
      if (cand1 <= theta_max) {
          if (excavator_ik_.inverseKinematics4Dof(
                  goal->position_with_angle.position.x + offset*cos(radians),
                  goal->position_with_angle.position.y + offset*sin(radians),
                  goal->position_with_angle.position.z - 0.3,
                  cand1,
                  target_joint_values) == 0)
          {
              best_theta = cand1;
              found = true;
              break;
          }
      }
      // 2) θw - Δ を試す
      // double cand2 = theta_w - delta;
      // if (delta > 0.0 && cand2 >= theta_min) {
      //     if (excavator_ik_.inverseKinematics4Dof(
      //             goal->position_with_angle.position.x + offset*cos(radians),
      //             goal->position_with_angle.position.y + offset*sin(radians),
      //             goal->position_with_angle.position.z - 0.3,
      //             cand2,
      //             target_joint_values) == 0)
      //     {
      //         best_theta = cand2;
      //         found = true;
      //         break;
      //     }
      // }
  }

  if (found) {

  } else {
      handle_error("Failed to calculate inverse kinematics near theta_w");
      return;
  }


  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
  std::vector<std::vector<double>> joint_targets;

  // Swingだけ先に動かす
  std::vector<double> swing_joint_values(joint_names_.size(), 0.0);
  swing_joint_values = move_group_->getCurrentJointValues();
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (joint_names_[i] == "swing_joint") {
      swing_joint_values[i] = target_joint_values[i];
    }
  }

  joint_targets.push_back(swing_joint_values);
  
  joint_targets.push_back(target_joint_values);
  
  // arm_joint を追加で動かした状態
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (joint_names_[i] == "arm_joint") {
      target_joint_values[i] += 0.1;
    } else if (joint_names_[i] == "bucket_joint") {
      target_joint_values[i] = 1.7;
    }
  }
  joint_targets.push_back(target_joint_values);
  
  // // bucket_joint を追加で動かした状態
  // for (size_t i = 0; i < joint_names_.size(); i++) {
  //   if (joint_names_[i] == "bucket_joint") {
  //     target_joint_values[i] = 2.2;
  //   }
  // }
  // joint_targets.push_back(target_joint_values);
  
  // while(!joint_targets.empty()){
    // 最初の状態を取得
    moveit::core::RobotState start_state(*move_group_->getCurrentState());
    
    bool planning_success = true;
    
    for (const auto& joints : joint_targets) {
      move_group_->setStartState(start_state);
      move_group_->setJointValueTarget(joints);
    
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_->plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan to one of the waypoints");
        planning_success = false;
        break;
      }
    
      plans.push_back(plan);  // 成功したら保持
    
      // 次の出発点を更新（まだexecuteしてないけど、stateは変える）
      start_state.setJointGroupPositions(
        move_group_->getRobotModel()->getJointModelGroup(move_group_->getName()), joints);
      start_state.update();
    }
    
    if (!planning_success) {
      RCLCPP_WARN(this->get_logger(), "Aborting execution due to planning failure.");
      result->error_code.val = static_cast<int>(rclcpp_action::ResultCode::ABORTED);
      goal_handle->abort(result);
      return;
    }
    
  //   bool all_success = true;
  //   // 成功した場合、順番に実行
  //   for (const auto& plan : plans) {
  //     moveit::core::MoveItErrorCode exec_result = move_group_->execute(plan);
    
  //     if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
  //       continue;  // 正常に完了
  //     } else if (exec_result == moveit::core::MoveItErrorCode::CONTROL_FAILED) {
  //       RCLCPP_WARN(this->get_logger(), "Execution timed out, but continuing to next trajectory...");
  //       joint_targets.erase(joint_targets.begin());
  //       plans.clear();  // 前のプランは破棄
  //       all_success = false;
  //       break;  // タイムアウトだが無視して次へ
  //     } else {
  //       RCLCPP_ERROR(this->get_logger(), "Execution failed with error code: %d", exec_result.val);
  //       result->error_code.val = static_cast<int>(rclcpp_action::ResultCode::ABORTED);
  //       goal_handle->abort(result);
  //       return;
  //     }
  //   }

  //   if (all_success) {
  //     RCLCPP_INFO(this->get_logger(), "All trajectories executed successfully.");
  //     result->error_code.val = static_cast<int>(rclcpp_action::ResultCode::SUCCEEDED);
  //     goal_handle->succeed(result);
  //     return;
  //   }
  // }

  // RCLCPP_INFO(this->get_logger(), "All trajectories executed successfully.");
  // result->error_code.val = static_cast<int>(rclcpp_action::ResultCode::SUCCEEDED);
  // goal_handle->succeed(result);
  

  // // 成功した場合、順番に実行
  for (const auto& plan : plans) {
    if (!move_group_->execute(plan)) {
      RCLCPP_ERROR(this->get_logger(), "Execution failed for one of the trajectories");
      result->error_code.val = static_cast<int>(rclcpp_action::ResultCode::ABORTED);
      goal_handle->abort(result);
      return;
    }
  }

  // // Execute
  // feedback->state = "EXECUTING";
  // goal_handle->publish_feedback(feedback);
  // if (move_group_->execute(combined_trajectory) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  // {
  //   handle_error("Failed to execute");
  //   return;
  // }

  // RCLCPP_INFO(this->get_logger(), "Move to a position 1 m higher than the target excavation position succeeded");

  // // std::vector<double> target_joint_values2(joint_names_.size(), 0.0);
  // // /*** Move to a target excavation position ***/
  // // for (double i = 2.0; i > 0.01; i -= 0.01)
  // // {
  // //   if (excavator_ik_.inverseKinematics4Dof(goal->position_with_angle.position.x - offset*cos(radians), goal->position_with_angle.position.y + offset*sin(radians),
  // //                                           goal->position_with_angle.position.z + 0.5, i, target_joint_values2) == 0)
  // //   {
  // //     // RCLCPP_INFO(this->get_logger(), "%f", i);
  // //     break;
  // //   }
  // //   // RCLCPP_INFO(this->get_logger(), "%f", i);
  // //   // if(i >= 0.01)
  // //   // {
  // //   //   handle_error("Failed to calculate inverse kinematics");
  // //   //   return;
  // //   // }
  // // }

  // // move_group_->setJointValueTarget(target_joint_values2);

  // /*** Exacvate ***/
  // // Get current joint values
  // target_joint_values = move_group_->getCurrentJointValues();
  // for (size_t i = 0; i < joint_names_.size(); i++)
  // {
  //   if (joint_names_[i] == "arm_joint")
  //   {
  //     target_joint_values[i] = target_joint_values[i] + 0.3;  // 掘削時のarmの目標角度[rad]
  //     break;
  //   }
  // }

  // move_group_->setJointValueTarget(target_joint_values);

  // // Plan
  // feedback->state = "PLANNING";
  // goal_handle->publish_feedback(feedback);
  // if (move_group_->plan(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  // {
  //   handle_error("Failed to plan");
  //   return;
  // }

  // // Execute
  // feedback->state = "EXECUTING";
  // goal_handle->publish_feedback(feedback);
  // if (move_group_->execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  // {
  //   handle_error("Failed to execute");
  //   return;
  // }

  // RCLCPP_INFO(this->get_logger(), "Move to target excavation position succeeded");

  // // std::vector<double> target_joint_values3(joint_names_.size(), 0.0);
  // // /*** Move to a target excavation position ***/
  // // for (double i = 3.0; i > 0.01; i -= 0.01)
  // // {
  // //   if (excavator_ik_.inverseKinematics4Dof(goal->position_with_angle.position.x - 2.0*offset*cos(radians), goal->position_with_angle.position.y + 2.0*offset*sin(radians),
  // //                                           goal->position_with_angle.position.z + 1.5, i, target_joint_values3) == 0)
  // //   {
  // //     // RCLCPP_INFO(this->get_logger(), "%f", i);
  // //     break;
  // //   }
  // //   // RCLCPP_INFO(this->get_logger(), "%f", i);
  // //   // if(i >= 0.01)
  // //   // {
  // //   //   handle_error("Failed to calculate inverse kinematics");
  // //   //   return;
  // //   // }
  // // }

  // // move_group_->setJointValueTarget(target_joint_values3);

  // // // Plan
  // // feedback->state = "PLANNING";
  // // goal_handle->publish_feedback(feedback);
  // // if (move_group_->plan(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  // // {
  // //   handle_error("Failed to plan");
  // //   return;
  // // }

  // // // Execute
  // // feedback->state = "EXECUTING";
  // // goal_handle->publish_feedback(feedback);
  // // if (move_group_->execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  // // {
  // //   handle_error("Failed to execute");
  // //   return;
  // // }

  // // /*** Exacvate ***/
  // // Get current joint values
  // target_joint_values = move_group_->getCurrentJointValues();
  // for (size_t i = 0; i < joint_names_.size(); i++)
  // {
  //   if (joint_names_[i] == "bucket_joint")
  //   {
  //     target_joint_values[i] = target_joint_values[i] + 0.8;  // 掘削時のバケットの目標角度[rad]
  //     break;
  //   }
  // }

  // move_group_->setJointValueTarget(target_joint_values);

  // // Plan
  // feedback->state = "PLANNING";
  // goal_handle->publish_feedback(feedback);
  // if (move_group_->plan(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  // {
  //   handle_error("Failed to plan");
  //   return;
  // }

  // // Execute
  // feedback->state = "EXECUTING";
  // goal_handle->publish_feedback(feedback);
  // if (move_group_->execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  // {
  //   handle_error("Failed to execute");
  //   return;
  // }

  // If execution was successful, set the result of the action and mark it as succeeded.
  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  result->error_code.val = 1;
  goal_handle->succeed(result);
}

void Zx200ExcavateSimpleActionServer::apply_collision_objects_from_db(const std::string& record_name)
{
  // Load collision objects from DB
  // RCLCPP_INFO(this->get_logger(), "Loading collision objects from DB");

  mongocxx::client client{ mongocxx::uri{ "mongodb://localhost:27017" } };
  mongocxx::database db = client["rostmsdb"];
  mongocxx::collection collection = db["parameter"];
  bsoncxx::builder::stream::document filter_builder;
  filter_builder << "record_name" << record_name;
  auto filter = filter_builder.view();
  auto result = collection.find_one(filter);

  if ((record_name != "") && !result)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get collision objects from DB");
    return;
  }
  else if (record_name == "")
  {
    RCLCPP_INFO(this->get_logger(), "No collision objects to load");
    return;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Succeeded to get collision objects from DB");
  }

  // Remove all collision objects
  // std::vector<std::string> object_ids = planning_scene_interface_.getKnownObjectNames();
  // planning_scene_interface_.removeCollisionObjects(object_ids);

  // Apply collision objects
  auto collision_objects = result->view()["collision_objects"].get_array().value;
  for (auto&& co : collision_objects)
  {
    moveit_msgs::msg::CollisionObject co_msg;
    co_msg.header.frame_id = move_group_->getPlanningFrame();
    co_msg.id = co["id"].get_utf8().value.to_string();

    // Get collision object type
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = co["primitive_type"].get_int32().value;
    for (auto dimension : co["dimensions"].get_array().value)
    {
      if (dimension.type() == bsoncxx::type::k_double)
      {
        primitive.dimensions.push_back(dimension.get_double().value);
      }
      else if (dimension.type() == bsoncxx::type::k_int32)
      {
        primitive.dimensions.push_back(static_cast<double>(dimension.get_int32().value));
      }
      else
      {
        throw std::runtime_error("Unsupported type");
      }
    }

    co_msg.primitives.push_back(primitive);

    // Get collision object pose
    geometry_msgs::msg::Pose pose;
    pose.position.x = getDoubleValue(co["pose"]["position"]["x"]);
    pose.position.y = getDoubleValue(co["pose"]["position"]["y"]);
    pose.position.z = getDoubleValue(co["pose"]["position"]["z"]);
    pose.orientation.x = getDoubleValue(co["pose"]["orientation"]["x"]);
    pose.orientation.y = getDoubleValue(co["pose"]["orientation"]["y"]);
    pose.orientation.z = getDoubleValue(co["pose"]["orientation"]["z"]);
    pose.orientation.w = getDoubleValue(co["pose"]["orientation"]["w"]);

    co_msg.primitive_poses.push_back(pose);

    co_msg.operation = co_msg.ADD;

    planning_scene_interface_.applyCollisionObject(co_msg);
  }
}

void Zx200ExcavateSimpleActionServer::apply_collision_objects_mesh_from_db(const std::vector<std::string>& record_names)
{
  for (const auto& record_name : record_names)
  {
    // Load collision objects from DB
    mongocxx::client client{ mongocxx::uri{ "mongodb://localhost:27017" } };
    mongocxx::database db = client["rostmsdb"];
    mongocxx::collection collection = db["parameter"];
    bsoncxx::builder::stream::document filter_builder;
    filter_builder << "record_name" << record_name;
    auto filter = filter_builder.view();
    auto result = collection.find_one(filter);

    if ((record_name != "") && !result)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get collision objects from DB for record: %s", record_name.c_str());
      continue;
    }
    else if (record_name == "")
    {
      RCLCPP_INFO(this->get_logger(), "No collision objects to load");
      continue;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Succeeded to get collision objects from DB for record: %s", record_name.c_str());
    }

    auto collision_objects_dump = result->view();
    moveit_msgs::msg::CollisionObject co_dump_msg;
    co_dump_msg.header.frame_id = move_group_->getPlanningFrame();
    co_dump_msg.id = record_name + "_mesh"; // 一意のIDを付与

    // Apply collision objects
    auto mesh_binary = collision_objects_dump["data"].get_binary();
    std::string temp_mesh_path = "/tmp/temp_dump_mesh_" + record_name + ".dae";
    std::ofstream ofs(temp_mesh_path, std::ios::binary);
    ofs.write(reinterpret_cast<const char*>(mesh_binary.bytes), mesh_binary.size);
    ofs.close();
    shapes::Mesh *m = shapes::createMeshFromResource("file://" + temp_mesh_path);
    if (!m)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from temporary file for record: %s", record_name.c_str());
      continue;
    }
    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(m, shape_msg);
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

    // Get collision object pose
    geometry_msgs::msg::Pose mesh_pose;
    mesh_pose.position.x = collision_objects_dump["x"].get_double().value;
    mesh_pose.position.y = collision_objects_dump["y"].get_double().value;
    mesh_pose.position.z = collision_objects_dump["z"].get_double().value;
    mesh_pose.orientation.x = collision_objects_dump["qx"].get_double().value;
    mesh_pose.orientation.y = collision_objects_dump["qy"].get_double().value;
    mesh_pose.orientation.z = collision_objects_dump["qz"].get_double().value;
    mesh_pose.orientation.w = collision_objects_dump["qw"].get_double().value;

    // CollisionObjectにメッシュを追加
    co_dump_msg.meshes.push_back(mesh_msg);
    co_dump_msg.mesh_poses.push_back(mesh_pose);
    co_dump_msg.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface_.applyCollisionObject(co_dump_msg);
  }
}

void Zx200ExcavateSimpleActionServer::apply_collision_objects_from_robot_description_and_tf(
  const std::string& other_robot_base_frame, 
  const std::string& collision_object_prefix)
{
  // ロボット名からdescriptionを取得
  auto it = other_robot_descriptions_.find(collision_object_prefix);
  if (it == other_robot_descriptions_.end() || it->second.empty()) {
    RCLCPP_WARN(this->get_logger(), "Robot description not available for %s", collision_object_prefix.c_str());
    return;
  }

  const std::string& robot_description = it->second;

  // URDFモデルの解析
  urdf::Model other_robot_model;
  if (!other_robot_model.initString(robot_description)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF for robot %s", collision_object_prefix.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Processing collision objects for robot %s", collision_object_prefix.c_str());
  
  // 各リンクのコリジョン形状を処理
  int processed_links = 0;
  int successful_links = 0;
  
  for (const auto& link_pair : other_robot_model.links_) {
    const auto& link = link_pair.second;
    
    if (!link->collision_array.empty()) {
      RCLCPP_INFO(this->get_logger(), "Link %s has %zu collision objects", link->name.c_str(), link->collision_array.size());
      
      for (size_t i = 0; i < link->collision_array.size(); ++i) {
        const auto& collision = link->collision_array[i];
        processed_links++;
        
        try {
          // TFから該当リンクの位置を取得 ({robot_name}_tf/{link_name}形式)
          geometry_msgs::msg::TransformStamped transform;
          std::string tf_frame_name = collision_object_prefix + "_tf/" + link->name;
          transform = tf_buffer_->lookupTransform(
            move_group_->getPlanningFrame(), 
            tf_frame_name,
            tf2::TimePointZero, 
            tf2::durationFromSec(1.0));

          RCLCPP_INFO(this->get_logger(), "Successfully found TF for link %s (frame: %s)", 
                     link->name.c_str(), tf_frame_name.c_str());
          successful_links++;

          // CollisionObjectを作成
          moveit_msgs::msg::CollisionObject co_msg;
          co_msg.header.frame_id = move_group_->getPlanningFrame();
          co_msg.id = collision_object_prefix + "_" + link->name + "_" + std::to_string(i);

          // 形状の種類に応じて処理
          if (collision->geometry->type == urdf::Geometry::BOX) {
            auto box = std::dynamic_pointer_cast<urdf::Box>(collision->geometry);
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {box->dim.x, box->dim.y, box->dim.z};
            co_msg.primitives.push_back(primitive);

            geometry_msgs::msg::Pose pose;
            pose.position.x = transform.transform.translation.x + collision->origin.position.x;
            pose.position.y = transform.transform.translation.y + collision->origin.position.y;
            pose.position.z = transform.transform.translation.z + collision->origin.position.z;
            
            // 回転の合成
            tf2::Quaternion tf_quat(transform.transform.rotation.x, transform.transform.rotation.y,
                                   transform.transform.rotation.z, transform.transform.rotation.w);
            tf2::Quaternion collision_quat(collision->origin.rotation.x, collision->origin.rotation.y,
                                         collision->origin.rotation.z, collision->origin.rotation.w);
            tf2::Quaternion combined_quat = tf_quat * collision_quat;
            
            pose.orientation.x = combined_quat.x();
            pose.orientation.y = combined_quat.y();
            pose.orientation.z = combined_quat.z();
            pose.orientation.w = combined_quat.w();
            
            co_msg.primitive_poses.push_back(pose);
          }
          else if (collision->geometry->type == urdf::Geometry::CYLINDER) {
            auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(collision->geometry);
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.CYLINDER;
            primitive.dimensions = {cylinder->length, cylinder->radius};
            co_msg.primitives.push_back(primitive);

            geometry_msgs::msg::Pose pose;
            pose.position.x = transform.transform.translation.x + collision->origin.position.x;
            pose.position.y = transform.transform.translation.y + collision->origin.position.y;
            pose.position.z = transform.transform.translation.z + collision->origin.position.z;
            
            tf2::Quaternion tf_quat(transform.transform.rotation.x, transform.transform.rotation.y,
                                   transform.transform.rotation.z, transform.transform.rotation.w);
            tf2::Quaternion collision_quat(collision->origin.rotation.x, collision->origin.rotation.y,
                                         collision->origin.rotation.z, collision->origin.rotation.w);
            tf2::Quaternion combined_quat = tf_quat * collision_quat;
            
            pose.orientation.x = combined_quat.x();
            pose.orientation.y = combined_quat.y();
            pose.orientation.z = combined_quat.z();
            pose.orientation.w = combined_quat.w();
            
            co_msg.primitive_poses.push_back(pose);
          }
          else if (collision->geometry->type == urdf::Geometry::SPHERE) {
            auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(collision->geometry);
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.SPHERE;
            primitive.dimensions = {sphere->radius};
            co_msg.primitives.push_back(primitive);

            geometry_msgs::msg::Pose pose;
            pose.position.x = transform.transform.translation.x + collision->origin.position.x;
            pose.position.y = transform.transform.translation.y + collision->origin.position.y;
            pose.position.z = transform.transform.translation.z + collision->origin.position.z;
            
            tf2::Quaternion tf_quat(transform.transform.rotation.x, transform.transform.rotation.y,
                                   transform.transform.rotation.z, transform.transform.rotation.w);
            tf2::Quaternion collision_quat(collision->origin.rotation.x, collision->origin.rotation.y,
                                         collision->origin.rotation.z, collision->origin.rotation.w);
            tf2::Quaternion combined_quat = tf_quat * collision_quat;
            
            pose.orientation.x = combined_quat.x();
            pose.orientation.y = combined_quat.y();
            pose.orientation.z = combined_quat.z();
            pose.orientation.w = combined_quat.w();
            
            co_msg.primitive_poses.push_back(pose);
          }
          else if (collision->geometry->type == urdf::Geometry::MESH) {
            auto mesh_geom = std::dynamic_pointer_cast<urdf::Mesh>(collision->geometry);
            shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_geom->filename);
            if (mesh) {
              shape_msgs::msg::Mesh mesh_msg;
              shapes::ShapeMsg shape_msg;
              shapes::constructMsgFromShape(mesh, shape_msg);
              mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
              
              geometry_msgs::msg::Pose pose;
              pose.position.x = transform.transform.translation.x + collision->origin.position.x;
              pose.position.y = transform.transform.translation.y + collision->origin.position.y;
              pose.position.z = transform.transform.translation.z + collision->origin.position.z;
              
              tf2::Quaternion tf_quat(transform.transform.rotation.x, transform.transform.rotation.y,
                                     transform.transform.rotation.z, transform.transform.rotation.w);
              tf2::Quaternion collision_quat(collision->origin.rotation.x, collision->origin.rotation.y,
                                           collision->origin.rotation.z, collision->origin.rotation.w);
              tf2::Quaternion combined_quat = tf_quat * collision_quat;
              
              pose.orientation.x = combined_quat.x();
              pose.orientation.y = combined_quat.y();
              pose.orientation.z = combined_quat.z();
              pose.orientation.w = combined_quat.w();
              
              co_msg.meshes.push_back(mesh_msg);
              co_msg.mesh_poses.push_back(pose);
              delete mesh;
            }
          }

          co_msg.operation = moveit_msgs::msg::CollisionObject::ADD;
          planning_scene_interface_.applyCollisionObject(co_msg);
          
        } catch (tf2::TransformException& ex) {
          RCLCPP_WARN(this->get_logger(), "Could not get transform for link %s (frame: %s): %s", 
                     link->name.c_str(), (collision_object_prefix + "_tf/" + link->name).c_str(), ex.what());
          continue;
        }
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Link %s has no collision objects", link->name.c_str());
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Applied collision objects from robot description and TF for %s: %d/%d links successful", 
             collision_object_prefix.c_str(), successful_links, processed_links);
}

bool Zx200ExcavateSimpleActionServer::load_urdf_from_file(const std::string& robot_name)
{
  try {
    // tms_if_for_operaパッケージのパスを取得
    std::string package_path = ament_index_cpp::get_package_share_directory("tms_if_for_opera");
    std::string urdf_path = package_path + "/collision_objects/" + robot_name + "_description/urdf/" + robot_name + ".urdf";
    
    std::ifstream file(urdf_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open URDF file: %s", urdf_path.c_str());
      return false;
    }

    // URDFファイルの内容を読み込み
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string urdf_content = buffer.str();
    file.close();

    // package://パスを実際のファイルパスに置換
    // package://mst110cr_description/meshes/ -> file:///path/to/tms_if_for_opera/collision_objects/mst110cr_description/meshes/
    std::string package_prefix = "package://" + robot_name + "_description/";
    std::string actual_path = "file://" + package_path + "/collision_objects/" + robot_name + "_description/";
    
    size_t pos = 0;
    while ((pos = urdf_content.find(package_prefix, pos)) != std::string::npos) {
      urdf_content.replace(pos, package_prefix.length(), actual_path);
      pos += actual_path.length();
    }
    
    // URDFパーサーでテスト
    urdf::Model test_model;
    if (!test_model.initString(urdf_content)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF for %s", robot_name.c_str());
      return false;
    }
    
    other_robot_descriptions_[robot_name] = urdf_content;
    RCLCPP_INFO(this->get_logger(), "Successfully loaded URDF for %s from %s", robot_name.c_str(), urdf_path.c_str());
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading URDF for %s: %s", robot_name.c_str(), e.what());
    return false;
  }
}

double Zx200ExcavateSimpleActionServer::getDoubleValue(const bsoncxx::document::element& element)
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
  rclcpp::spin(std::make_shared<Zx200ExcavateSimpleActionServer>());
  rclcpp::shutdown();
  return 0;
}