#include "tms_if_for_opera/backhoe_change_pose_action_server.hpp"

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

using namespace tms_if_for_opera;

BackhoeChangePoseActionServer::BackhoeChangePoseActionServer(const rclcpp::NodeOptions& options)
  : Node("backhoe_change_pose_action_server", options)
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

  action_server_ = rclcpp_action::create_server<BackhoeChangePose>(
      this, "backhoe_change_pose", std::bind(&BackhoeChangePoseActionServer::handle_goal, this, _1, _2),
      std::bind(&BackhoeChangePoseActionServer::handle_cancel, this, _1),
      std::bind(&BackhoeChangePoseActionServer::handle_accepted, this, _1));
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

  // Get robot info
  joint_names_ = move_group_->getJointNames();

  // For FK
  robot_state_ = std::make_shared<moveit::core::RobotState>(move_group_->getRobotModel());

  // Init DB connection
  mongocxx::instance instance{};

  /*** debug ***/

  // RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
  // RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
  // RCLCPP_INFO(this->get_logger(), "Available Planning Groups:");
  // std::copy(move_group_->getJointModelGroupNames().begin(), move_group_->getJointModelGroupNames().end(),
  //           std::ostream_iterator<std::string>(std::cout, ", "));
  // RCLCPP_INFO(this->get_logger(), "Joint Names:");
  // std::copy(move_group_->getJointNames().begin(), move_group_->getJointNames().end(),
  //           std::ostream_iterator<std::string>(std::cout, ", "));

  /******/
}

rclcpp_action::GoalResponse BackhoeChangePoseActionServer::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const BackhoeChangePose::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
BackhoeChangePoseActionServer::handle_cancel(const std::shared_ptr<GoalHandleBackhoeChangePose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void BackhoeChangePoseActionServer::handle_accepted(const std::shared_ptr<GoalHandleBackhoeChangePose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "handle_accepted() start.");
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{ std::bind(&BackhoeChangePoseActionServer::execute, this, _1), goal_handle }.detach();
}

void BackhoeChangePoseActionServer::execute(const std::shared_ptr<GoalHandleBackhoeChangePose> goal_handle)
{
  // Apply collision object
  apply_collision_objects_from_db(collision_object_component_name_);

  // Execute goal
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<BackhoeChangePose::Feedback>();
  auto result = std::make_shared<BackhoeChangePose::Result>();

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
        goal_handle->abort(result);

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
        goal_handle->abort(result);

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

      robot_state_->setJointGroupPositions(move_group_->getName(), target_joint_values);
      robot_state_->update();
      Eigen::Isometry3d end_effector_state = robot_state_->getGlobalLinkTransform(move_group_->getEndEffectorLink());

      // Set pose constraint
      // moveit_msgs::msg::Constraints pose_constraints;
      // moveit_msgs::msg::OrientationConstraint ocm;

      // ocm.link_name = "end_effector_link";  // Replace with your end effector link name
      // ocm.header.frame_id = "base_link";    // Replace with your base link name
      // ocm.orientation.w = 1.0;              // Specify the desired orientation
      // ocm.absolute_x_axis_tolerance = 0.1;
      // ocm.absolute_y_axis_tolerance = 0.1;
      // ocm.absolute_z_axis_tolerance = 0.1;
      // ocm.weight = 1.0;

      // pose_constraints.orientation_constraints.push_back(ocm);

      // move_group_->setPathConstraints(pose_constraints);

      // Set target pose
      move_group_->setPoseTarget(end_effector_state);
      // move_group_->setJointValueTarget(target_joint_values);

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
        goal_handle->abort(result);

        break;
      }
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "No or too much input.");
    feedback->state = "ABORTED";
    goal_handle->publish_feedback(feedback);
    result->error_code.val = 9999;
    goal_handle->abort(result);
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

void BackhoeChangePoseActionServer::apply_collision_objects_from_db(const std::string& component_name)
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
  std::vector<std::string> object_ids = planning_scene_interface_.getKnownObjectNames();
  planning_scene_interface_.removeCollisionObjects(object_ids);

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

double BackhoeChangePoseActionServer::getDoubleValue(const bsoncxx::document::element& element)
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
  rclcpp::spin(std::make_shared<BackhoeChangePoseActionServer>());
  rclcpp::shutdown();
  return 0;
}