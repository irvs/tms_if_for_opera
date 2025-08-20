#include "tms_if_for_opera/zx200_change_pose_action_server.hpp"

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

using namespace tms_if_for_opera;

Zx200ChangePoseActionServer::Zx200ChangePoseActionServer(const rclcpp::NodeOptions& options)
  : Node("tms_if_for_opera_zx200_change_pose", options)
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

  action_server_ = rclcpp_action::create_server<Zx200ChangePose>(
      this, "tms_rp_zx200_change_pose_plan", std::bind(&Zx200ChangePoseActionServer::handle_goal, this, _1, _2),
      std::bind(&Zx200ChangePoseActionServer::handle_cancel, this, _1),
      std::bind(&Zx200ChangePoseActionServer::handle_accepted, this, _1));
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

  // TF2の初期化
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // For emg stop
  this->emg_stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/zx200/emg_stop", 10);

  // 複数ロボットのコリジョン設定パラメータ (robot_name1,robot_name2,...)
  this->declare_parameter<std::string>("other_robots_config", "");
  std::string other_robots_config_str;
  this->get_parameter("other_robots_config", other_robots_config_str);

  // collision_objectsフォルダのパス設定
  this->declare_parameter<std::string>("collision_objects_base_path", "/home/common/3_SIP/tms_fujita_ws/src/tms_if_for_opera/collision_objects");
  std::string collision_objects_base_path;
  this->get_parameter("collision_objects_base_path", collision_objects_base_path);

  // 複数ロボットの設定を解析 (例: "mst110cr,robot2,excavator")
  if (!other_robots_config_str.empty()) {
    std::stringstream ss(other_robots_config_str);
    std::string robot_name;
    while (std::getline(ss, robot_name, ',')) {
      // 前後の空白を削除
      robot_name.erase(0, robot_name.find_first_not_of(" \t"));
      robot_name.erase(robot_name.find_last_not_of(" \t") + 1);
      
      if (!robot_name.empty()) {
        // URDFファイルを読み込み
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

rclcpp_action::GoalResponse Zx200ChangePoseActionServer::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                                     std::shared_ptr<const Zx200ChangePose::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Zx200ChangePoseActionServer::handle_cancel(const std::shared_ptr<GoalHandleZx200ChangePose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Publishing EMG stop signal to ZX200.");

  // 実機用非常停止
  std_msgs::msg::Bool msg;
  msg.data = true;
  this->emg_stop_publisher_->publish(msg);
  // move_group停止
  move_group_->stop();

  // auto result = std::make_shared<Zx200ChangePose::Result>();
  // result->error_code.val = 9999;
  // goal_handle->abort(result);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Zx200ChangePoseActionServer::handle_accepted(const std::shared_ptr<GoalHandleZx200ChangePose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "handle_accepted() start.");
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{ std::bind(&Zx200ChangePoseActionServer::execute, this, _1), goal_handle }.detach();
}

void Zx200ChangePoseActionServer::execute(const std::shared_ptr<GoalHandleZx200ChangePose> goal_handle)
{
  // Apply collision object
  apply_collision_objects_from_db(collision_object_record_name_);
  apply_collision_objects_mesh_from_db(collision_object_dump_record_name_);
  
  // 全ての他ロボットのコリジョンを追加
  for (const auto& robot_pair : other_robot_descriptions_) {
    const std::string& robot_name = robot_pair.first;
    if (!robot_pair.second.empty()) {
      // base_frameも{robot_name}_tf形式に修正
      apply_collision_objects_from_robot_description_and_tf(robot_name + "_2/base_link", robot_name);
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

  // Execute goal
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Zx200ChangePose::Feedback>();
  auto result = std::make_shared<Zx200ChangePose::Result>();

  feedback->state = "IDLE";
  goal_handle->publish_feedback(feedback);

  // Get current joint values
  std::vector<double> joint_values = move_group_->getCurrentJointValues();
  for (size_t i = 0; i < joint_names_.size() && i < joint_values.size(); i++)
  {
    current_joint_values_[joint_names_[i]] = joint_values[i];
    // target_joint_values_[joint_names_[i]] = joint_values[i];
  }

  // Planning
  // TODO: Fix to connect wayponts smoothly
  if (goal->trajectory.points.size() > 0 && goal->pose_sequence.size() == 0 &&
      goal->position_with_angle_sequence.size() == 0)
  {
    for (const auto& point : goal->trajectory.points)
    {
      std::map<std::string, double> target_joint_values;
      for (size_t i = 0; i < goal->trajectory.joint_names.size() && i < point.positions.size(); ++i)
      {
        if (fabs(point.positions[i]) > 2.0 * M_PI)
        {
          target_joint_values[goal->trajectory.joint_names[i]] = current_joint_values_[goal->trajectory.joint_names[i]];
        }
        else
        {
          target_joint_values[goal->trajectory.joint_names[i]] = point.positions[i];
        }
      }
      move_group_->setJointValueTarget(target_joint_values);

      feedback->state = "PLANNING";
      goal_handle->publish_feedback(feedback);

      if (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        feedback->state = "SUCCEEDED";
        goal_handle->publish_feedback(feedback);
        result->error_code.val = 1;
        // goal_handle->succeed(result);
      }
      else
      {  // Failed
        feedback->state = "ABORTED";
        goal_handle->publish_feedback(feedback);
        result->error_code.val = 9999;

        break;
      }
    }
  }
  else if (goal->pose_sequence.size() > 0 && goal->trajectory.points.size() == 0 &&
           goal->position_with_angle_sequence.size() == 0)
  {
    for (const auto& pose : goal->pose_sequence)
    {
      move_group_->setPoseTarget(pose);

      feedback->state = "PLANNING";
      goal_handle->publish_feedback(feedback);

      if (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        feedback->state = "SUCCEEDED";
        goal_handle->publish_feedback(feedback);
        result->error_code.val = 1;
        // goal_handle->succeed(result);
      }
      else
      {  // Failed
        feedback->state = "ABORTED";
        goal_handle->publish_feedback(feedback);
        result->error_code.val = 9999;

        break;
      }
    }
  }
  else if (goal->position_with_angle_sequence.size() > 0 && goal->trajectory.points.size() == 0 &&
           goal->pose_sequence.size() == 0)
  {
    for (const auto& point : goal->position_with_angle_sequence)
    {
      // Get end effector pose to use pose/position constraint
      std::vector<double> target_joint_values(joint_names_.size(), 0.0);
      if (excavator_ik_.inverseKinematics4Dof(point.position.x, point.position.y, point.position.z, point.theta_w,
                                              target_joint_values) == -1)
      {
        RCLCPP_INFO(this->get_logger(), "Failed to calculate inverse kinematics");
        feedback->state = "ABORTED";
        result->error_code.val = 9999;
        break;
      }

      // // Set pose constraint
      // // Check if constraint exists
      // if (goal->constraints.joint_constraints.empty() && goal->constraints.position_constraints.empty() &&
      //     goal->constraints.orientation_constraints.empty() && goal->constraints.visibility_constraints.empty())
      // {
      //   RCLCPP_INFO(this->get_logger(), "Constraints do not exist");
      // }
      // else
      // {
      //   RCLCPP_INFO(this->get_logger(), "Constraints exist");

      //   // TODO: Add error handling
      //   //       - Use constraint in joint space
      //   //       - Constraint is not for end effector
      //   if (goal->constraints.orientation_constraints.size() > 0)
      //   {
      //     RCLCPP_INFO(this->get_logger(), "Orientation constraint exists");
      //     auto current_pose = move_group_->getCurrentPose();
      //     moveit_msgs::msg::Constraints pose_constraints;
      //     moveit_msgs::msg::OrientationConstraint ocm;
      //     ocm.header.frame_id = move_group_->getPoseReferenceFrame();  // Replace with your base link name
      //     ocm.link_name = move_group_->getEndEffectorLink();
      //     // Specify the desired orientation
      //     ocm.orientation = current_pose.pose.orientation;
      //     ocm.absolute_x_axis_tolerance = goal->constraints.orientation_constraints[0].absolute_x_axis_tolerance;
      //     ocm.absolute_y_axis_tolerance = goal->constraints.orientation_constraints[0].absolute_y_axis_tolerance;
      //     ocm.absolute_z_axis_tolerance = goal->constraints.orientation_constraints[0].absolute_z_axis_tolerance;
      //     ocm.weight = goal->constraints.orientation_constraints[0].weight;
      //     pose_constraints.orientation_constraints.emplace_back(ocm);
      //     move_group_->setPathConstraints(pose_constraints);
      //   }
      // }

      // Set target pose
      // robot_state_->setJointGroupPositions(move_group_->getName(), target_joint_values);
      // robot_state_->update();
      // Eigen::Isometry3d end_effector_state = robot_state_->getGlobalLinkTransform(move_group_->getEndEffectorLink());
      // move_group_->setPoseTarget(end_effector_state);

      move_group_->setJointValueTarget(target_joint_values);

      feedback->state = "PLANNING";
      goal_handle->publish_feedback(feedback);
      result->error_code.val = 1;

      // if (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      // {
      //   feedback->state = "SUCCEEDED";
      //   goal_handle->publish_feedback(feedback);
      //   result->error_code.val = 1;
      //   // goal_handle->succeed(result);
      // }
      // else
      // {  // Failed
      //   feedback->state = "ABORTED";
      //   goal_handle->publish_feedback(feedback);
      //   result->error_code.val = 9999;

      //   break;
      // }
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "No or too much input.");
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
  }

  // If execution was successful, set the result of the action and mark it as succeeded.
  if (result->error_code.val == 1)
  {
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    goal_handle->succeed(result);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Goal aborted");
    goal_handle->abort(result);
  }
}

void Zx200ChangePoseActionServer::apply_collision_objects_from_db(const std::string& record_name)
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

void Zx200ChangePoseActionServer::apply_collision_objects_mesh_from_db(const std::vector<std::string>& record_names)
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

void Zx200ChangePoseActionServer::apply_collision_objects_from_robot_description_and_tf(
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
          // TFから該当リンクの位置を取得 ({robot_name}_2/{link_name}形式)
          geometry_msgs::msg::TransformStamped transform;
          std::string tf_frame_name = collision_object_prefix + "_2/" + link->name;
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
                     link->name.c_str(), (collision_object_prefix + "_2/" + link->name).c_str(), ex.what());
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

double Zx200ChangePoseActionServer::getDoubleValue(const bsoncxx::document::element& element)
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

bool Zx200ChangePoseActionServer::load_urdf_from_file(const std::string& robot_name)
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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Zx200ChangePoseActionServer>());
  rclcpp::shutdown();
  return 0;
}