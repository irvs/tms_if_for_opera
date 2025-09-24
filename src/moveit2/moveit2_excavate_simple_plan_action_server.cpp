#include "tms_if_for_opera/moveit2/moveit2_excavate_simple_action_server.hpp"

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

Moveit2ExcavateSimpleActionServer::Moveit2ExcavateSimpleActionServer(const rclcpp::NodeOptions& options)
  : Node("tms_if_for_opera_moveit2_excavate_simple_plan", options)
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

  action_server_ = rclcpp_action::create_server<ExcavatorExcavateSimple>(
      this, "tms_rp_excavator_excavate_simple_plan", std::bind(&Moveit2ExcavateSimpleActionServer::handle_goal, this, _1, _2),
      std::bind(&Moveit2ExcavateSimpleActionServer::handle_cancel, this, _1),
      std::bind(&Moveit2ExcavateSimpleActionServer::handle_accepted, this, _1));
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
  this->emg_stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>("emg_stop", 10);

 }

rclcpp_action::GoalResponse Moveit2ExcavateSimpleActionServer::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ExcavatorExcavateSimple::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Moveit2ExcavateSimpleActionServer::handle_cancel(const std::shared_ptr<GoalHandleExcavatorExcavateSimple> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Publishing EMG stop signal to Moveit2.");

  // 実機用非常停止
  std_msgs::msg::Bool msg;
  msg.data = true;
  this->emg_stop_publisher_->publish(msg);
  // move_group停止
  move_group_->stop();

  // auto result = std::make_shared<ExcavatorExcavateSimple::Result>();
  // result->error_code.val = 9999;
  // goal_handle->abort(result);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Moveit2ExcavateSimpleActionServer::handle_accepted(const std::shared_ptr<GoalHandleExcavatorExcavateSimple> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "handle_accepted() start.");
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{ std::bind(&Moveit2ExcavateSimpleActionServer::execute, this, _1), goal_handle }.detach();
}

void Moveit2ExcavateSimpleActionServer::execute(const std::shared_ptr<GoalHandleExcavatorExcavateSimple> goal_handle)
{
  // Start to execute goal
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ExcavatorExcavateSimple::Feedback>();
  auto result = std::make_shared<ExcavatorExcavateSimple::Result>();

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
  const double theta_w = goal->position_with_angle.theta_w;
  // const double theta_w = 0.0;
  const double theta_min = 0.0;
  const double theta_max = M_PI;
  std::vector<double> target_joint_values(joint_names_.size(), 0.0);

  bool found = false;
  double theta_offset = 0.1;
  // double best_theta = theta_w - theta_offset;
  double best_theta = theta_w;

  for (double delta = 0.0; delta <= theta_max; delta += step) {
      // 1) θw + Δ を試す
      double cand1 = theta_w + delta;
      if (cand1 <= theta_max) {
          if (excavator_ik_.inverseKinematics4Dof(
                  goal->position_with_angle.position.x + offset*cos(radians),
                  goal->position_with_angle.position.y + offset*sin(radians),
                  goal->position_with_angle.position.z - 1.0,
                  cand1,
                  target_joint_values) == 0)
          {
              best_theta = cand1;
              found = true;
              break;
          }
      }
      // 2) θw - Δ を試す
      double cand2 = theta_w - delta;
      if (delta > 0.0 && cand2 >= theta_min) {
          if (excavator_ik_.inverseKinematics4Dof(
                  goal->position_with_angle.position.x + offset*cos(radians),
                  goal->position_with_angle.position.y + offset*sin(radians),
                  goal->position_with_angle.position.z - 1.0,
                  cand2,
                  target_joint_values) == 0)
          {
              best_theta = cand2;
              found = true;
              break;
          }
      }
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
  
  // // 成功した場合、順番に実行
  // for (const auto& plan : plans) {
  //   if (!move_group_->execute(plan)) {
  //     RCLCPP_ERROR(this->get_logger(), "Execution failed for one of the trajectories");
  //     result->error_code.val = moveit::core::MoveItErrorCode::CONTROL_FAILED;
  //     goal_handle->abort(result);
  //     return;
  //   }
  // }

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
  RCLCPP_INFO(this->get_logger(), "Plan succeeded");
  result->error_code.val = 1;
  goal_handle->succeed(result);
}

void Moveit2ExcavateSimpleActionServer::apply_collision_objects_from_db(const std::string& record_name)
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

void Moveit2ExcavateSimpleActionServer::apply_collision_objects_mesh_from_db(const std::vector<std::string>& record_names)
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


double Moveit2ExcavateSimpleActionServer::getDoubleValue(const bsoncxx::document::element& element)
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
  rclcpp::spin(std::make_shared<Moveit2ExcavateSimpleActionServer>());
  rclcpp::shutdown();
  return 0;
}