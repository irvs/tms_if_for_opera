#include "tms_if_for_opera/zx200_excavate_simple_action_server.hpp"

// #include <moveit_msgs/msg/constraints.hpp>
// #include <moveit_msgs/msg/orientation_constraint.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

  this->declare_parameter<std::string>("collision_object_component_name", "");
  this->get_parameter("collision_object_component_name", collision_object_component_name_);
  RCLCPP_INFO(this->get_logger(), "Collision object component name: %s", collision_object_component_name_.c_str());

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

  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, planning_group_);
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
  apply_collision_objects_from_db(collision_object_component_name_);
  apply_collision_objects_ic120_from_db("collision_object_ic120");

  // Clear constraints
  // move_group_->clearPathConstraints();

  /*** Move to a position 1 m higher than the target excavation position ***/
  // Get target joint values
  std::vector<double> target_joint_values(joint_names_.size(), 0.0);
  for (double i = 0.0; i < M_PI / 3.0; i += 0.01)
  {
    if (excavator_ik_.inverseKinematics4Dof(goal->position_with_angle.position.x, goal->position_with_angle.position.y,
                                            goal->position_with_angle.position.z + 2.0, i, target_joint_values) == 0)
    {
      break;
    }
    if(i >= M_PI / 3.0 - 0.01)
    {
      handle_error("Failed to calculate inverse kinematics");
      return;
    }
  }

  move_group_->setJointValueTarget(target_joint_values);

  // Plan
  feedback->state = "PLANNING";
  goal_handle->publish_feedback(feedback);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  if (move_group_->plan(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    handle_error("Failed to plan");
    return;
  }

  // Execute
  feedback->state = "EXECUTING";
  goal_handle->publish_feedback(feedback);
  if (move_group_->execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    handle_error("Failed to execute");
    return;
  }

  /*** Move to a target excavation position ***/
  for (double i = 0.0; i < M_PI / 3.0; i += 0.01)
  {
    if (excavator_ik_.inverseKinematics4Dof(goal->position_with_angle.position.x, goal->position_with_angle.position.y,
                                            goal->position_with_angle.position.z, i, target_joint_values) == 0)
    {
      break;
    }

    if(i >= M_PI / 3.0 - 0.01)
    {
      handle_error("Failed to calculate inverse kinematics");
      return;
    }
  }

  move_group_->setJointValueTarget(target_joint_values);

  // Plan
  feedback->state = "PLANNING";
  goal_handle->publish_feedback(feedback);
  if (move_group_->plan(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    handle_error("Failed to plan");
    return;
  }

  // Execute
  feedback->state = "EXECUTING";
  goal_handle->publish_feedback(feedback);
  if (move_group_->execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    handle_error("Failed to execute");
    return;
  }

  /*** Exacvate ***/
  // Get current joint values
  target_joint_values = move_group_->getCurrentJointValues();
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    if (joint_names_[i] == "bucket_joint")
    {
      target_joint_values[i] = 1.92;  // 掘削時のバケットの目標角度[rad]
      break;
    }
  }

  move_group_->setJointValueTarget(target_joint_values);

  // Plan
  feedback->state = "PLANNING";
  goal_handle->publish_feedback(feedback);
  if (move_group_->plan(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    handle_error("Failed to plan");
    return;
  }

  // Execute
  feedback->state = "EXECUTING";
  goal_handle->publish_feedback(feedback);
  if (move_group_->execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    handle_error("Failed to execute");
    return;
  }

  // If execution was successful, set the result of the action and mark it as succeeded.
  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  result->error_code.val = 1;
  goal_handle->succeed(result);
}

void Zx200ExcavateSimpleActionServer::apply_collision_objects_from_db(const std::string& component_name)
{
  // Load collision objects from DB
  // RCLCPP_INFO(this->get_logger(), "Loading collision objects from DB");

  mongocxx::client client{ mongocxx::uri{ "mongodb://localhost:27017" } };
  mongocxx::database db = client["rostmsdb"];
  mongocxx::collection collection = db["parameter"];
  bsoncxx::builder::stream::document filter_builder;
  filter_builder << "component_name" << component_name;
  auto filter = filter_builder.view();
  auto result = collection.find_one(filter);

  if ((component_name != "") && !result)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get collision objects from DB");
    return;
  }
  else if (component_name == "")
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

void Zx200ExcavateSimpleActionServer::apply_collision_objects_ic120_from_db(const std::string& component_name)
{
  // Load collision objects from DB
  // RCLCPP_INFO(this->get_logger(), "Loading collision objects from DB");

  mongocxx::client client{ mongocxx::uri{ "mongodb://localhost:27017" } };
  mongocxx::database db = client["rostmsdb"];
  mongocxx::collection collection = db["parameter"];
  bsoncxx::builder::stream::document filter_builder;
  filter_builder << "component_name" << component_name;
  auto filter = filter_builder.view();
  auto result = collection.find_one(filter);

  if ((component_name != "") && !result)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get collision objects from DB");
    return;
  }
  else if (component_name == "")
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

  auto collision_objects_ic120 = result->view();
  moveit_msgs::msg::CollisionObject co_ic120_msg;
  // RCLCPP_INFO(this->get_logger(), "Retrieved document: %s", bsoncxx::to_json(collision_objects_ic120).c_str());
  co_ic120_msg.header.frame_id = move_group_->getPlanningFrame();
  // co_ic120_msg.id = collision_objects_ic120["_id"].get_utf8().value.to_string();

  // Apply collision objects
  shapes::Mesh *m = shapes::createMeshFromResource("package://tms_if_for_opera/collision_objects/ic120/ic120.dae");
  if (!m)
  {
      RCLCPP_ERROR(this->get_logger(), "Failed to load mesh file!");
      return;
  }
  shape_msgs::msg::Mesh mesh_msg;
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(m, shape_msg);
  mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

  // Get collision object pose
  geometry_msgs::msg::Pose mesh_pose;
  mesh_pose.position.x = collision_objects_ic120["x"].get_double().value;
  mesh_pose.position.y = collision_objects_ic120["y"].get_double().value;
  mesh_pose.position.z = collision_objects_ic120["z"].get_double().value;
  mesh_pose.orientation.x = collision_objects_ic120["qx"].get_double().value;
  mesh_pose.orientation.y = collision_objects_ic120["qy"].get_double().value;
  mesh_pose.orientation.z = collision_objects_ic120["qz"].get_double().value;
  mesh_pose.orientation.w = collision_objects_ic120["qw"].get_double().value;

  // CollisionObjectにメッシュを追加
  co_ic120_msg.meshes.push_back(mesh_msg);
  co_ic120_msg.mesh_poses.push_back(mesh_pose);
  co_ic120_msg.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Planning Sceneにオブジェクトを追加
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(co_ic120_msg);

  planning_scene_interface_.applyCollisionObject(co_ic120_msg);

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