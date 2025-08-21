#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <urdf/model.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <thread>
#include <set>

class CollisionUpdater : public rclcpp::Node
{
public:
  CollisionUpdater()
  : Node("collision_updater")
  {
    // パラメータ宣言
    this->declare_parameter<std::string>("planning_group", "manipulator");
    this->declare_parameter<std::string>(
      "collision_objects_base_path",
      "/home/common/3_SIP/tms_fujita_ws/src/tms_if_for_opera/collision_objects");
    this->declare_parameter<std::string>("other_robots_config", "");

    this->get_parameter("planning_group", planning_group_);
    this->get_parameter("collision_objects_base_path", collision_objects_base_path_);
    this->get_parameter("other_robots_config", other_robots_config_str_);

    RCLCPP_INFO(this->get_logger(), "Planning group: %s", planning_group_.c_str());

    // TF初期化
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // MoveGroupInterfaceの初期化
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    move_group_node_ =
      rclcpp::Node::make_shared(std::string(this->get_name()) + "_move_group");

    executor_.add_node(move_group_node_);
    std::thread([this]() { executor_.spin(); }).detach();

    move_group_options_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface::Options>(
        planning_group_, "robot_description", "/zx200");

    move_group_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        move_group_node_, *move_group_options_);

    planning_frame_ = move_group_->getPlanningFrame();
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", planning_frame_.c_str());

    // 他ロボット設定をロード
    parse_and_load_other_robots();

    // 定期更新タイマー (例: 1Hz)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CollisionUpdater::update, this));
  }

private:
  // 他ロボット設定の読み込み
  void parse_and_load_other_robots()
  {
    if (other_robots_config_str_.empty()) return;

    std::stringstream ss(other_robots_config_str_);
    std::string robot_name;
    std::set<std::string> loaded_types;

    while (std::getline(ss, robot_name, ',')) {
      robot_name.erase(0, robot_name.find_first_not_of(" \t"));
      robot_name.erase(robot_name.find_last_not_of(" \t") + 1);

      if (!robot_name.empty()) {
        std::string robot_type = robot_name;
        size_t underscore_pos = robot_name.find('_');
        if (underscore_pos != std::string::npos) {
          robot_type = robot_name.substr(0, underscore_pos);
        }

        // robot_type が未ロードなら読み込む
        if (loaded_types.find(robot_type) == loaded_types.end()) {
          if (load_urdf_from_file(robot_type)) {
            loaded_types.insert(robot_type);
            RCLCPP_INFO(this->get_logger(),
                        "Loaded URDF for robot type: %s",
                        robot_type.c_str());
          } else {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to load URDF for robot type: %s",
                        robot_type.c_str());
          }
        }

        // robot_name と robot_type の対応を記録
        robot_to_type_map_[robot_name] = robot_type;
      }
    }
  }

  // URDFロード
  bool load_urdf_from_file(const std::string& robot_type)
  {
    try {
      std::string package_path =
        ament_index_cpp::get_package_share_directory("tms_if_for_opera");
      std::string urdf_path =
        package_path + "/collision_objects/" + robot_type + "_description/urdf/" +
        robot_type + ".urdf";

      std::ifstream file(urdf_path);
      if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open URDF file: %s", urdf_path.c_str());
        return false;
      }

      std::stringstream buffer;
      buffer << file.rdbuf();
      std::string urdf_content = buffer.str();
      file.close();

      // package:// を file:// に置換
      std::string package_prefix = "package://" + robot_type + "_description/";
      std::string actual_path =
        "file://" + package_path + "/collision_objects/" + robot_type + "_description/";

      size_t pos = 0;
      while ((pos = urdf_content.find(package_prefix, pos)) != std::string::npos) {
        urdf_content.replace(pos, package_prefix.length(), actual_path);
        pos += actual_path.length();
      }

      urdf::Model test_model;
      if (!test_model.initString(urdf_content)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF for %s", robot_type.c_str());
        return false;
      }

      robot_type_descriptions_[robot_type] = urdf_content;
      return true;

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading URDF for %s: %s",
                   robot_type.c_str(), e.what());
      return false;
    }
  }

  // 定期更新処理
  void update()
  {
    for (const auto& pair : robot_to_type_map_) {
      const std::string& robot_name = pair.first;
      const std::string& robot_type = pair.second;
      apply_collision_objects_from_robot_description_and_tf(robot_name, robot_type);
    }
  }

  // URDF + TF から CollisionObject を生成して apply
  void apply_collision_objects_from_robot_description_and_tf(
    const std::string& robot_name,
    const std::string& robot_type)
  {
    auto it = robot_type_descriptions_.find(robot_type);
    if (it == robot_type_descriptions_.end() || it->second.empty()) {
      RCLCPP_WARN(this->get_logger(), "No URDF for type %s", robot_type.c_str());
      return;
    }

    const std::string& robot_description = it->second;
    urdf::Model model;
    if (!model.initString(robot_description)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to parse URDF for type %s", robot_type.c_str());
      return;
    }

    for (const auto& link_pair : model.links_) {
      const auto& link = link_pair.second;
      if (link->collision_array.empty()) continue;

      for (size_t i = 0; i < link->collision_array.size(); ++i) {
        auto collision = link->collision_array[i];
        try {
          geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(
              planning_frame_,
              robot_name + "/" + link->name,
              tf2::TimePointZero,
              tf2::durationFromSec(0.5));

          moveit_msgs::msg::CollisionObject co_msg;
          co_msg.header.frame_id = planning_frame_;
          co_msg.id = robot_name + "_" + link->name + "_" + std::to_string(i);

          // === 形状ごとの処理 ===
          if (collision->geometry->type == urdf::Geometry::BOX) {
            auto box = std::dynamic_pointer_cast<urdf::Box>(collision->geometry);
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {box->dim.x, box->dim.y, box->dim.z};
            co_msg.primitives.push_back(primitive);

            geometry_msgs::msg::Pose pose;
            fill_pose_with_tf_and_origin(pose, transform, collision->origin);
            co_msg.primitive_poses.push_back(pose);

          } else if (collision->geometry->type == urdf::Geometry::CYLINDER) {
            auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(collision->geometry);
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.CYLINDER;
            primitive.dimensions = {cylinder->length, cylinder->radius};
            co_msg.primitives.push_back(primitive);

            geometry_msgs::msg::Pose pose;
            fill_pose_with_tf_and_origin(pose, transform, collision->origin);
            co_msg.primitive_poses.push_back(pose);

          } else if (collision->geometry->type == urdf::Geometry::SPHERE) {
            auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(collision->geometry);
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.SPHERE;
            primitive.dimensions = {sphere->radius};
            co_msg.primitives.push_back(primitive);

            geometry_msgs::msg::Pose pose;
            fill_pose_with_tf_and_origin(pose, transform, collision->origin);
            co_msg.primitive_poses.push_back(pose);

          } else if (collision->geometry->type == urdf::Geometry::MESH) {
            auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(collision->geometry);
            shapes::Mesh* m = shapes::createMeshFromResource(mesh->filename);
            if (m) {
              shape_msgs::msg::Mesh mesh_msg;
              shapes::ShapeMsg shape_msg;
              if (shapes::constructMsgFromShape(m, shape_msg)) {
                mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
                co_msg.meshes.push_back(mesh_msg);

                geometry_msgs::msg::Pose pose;
                fill_pose_with_tf_and_origin(pose, transform, collision->origin);
                co_msg.mesh_poses.push_back(pose);
              }
              delete m;
            }
          }

          co_msg.operation = moveit_msgs::msg::CollisionObject::ADD;
          planning_scene_interface_.applyCollisionObject(co_msg);

        } catch (tf2::TransformException& ex) {
          RCLCPP_WARN(this->get_logger(),
                      "TF not found for %s/%s: %s",
                      robot_name.c_str(),
                      link->name.c_str(),
                      ex.what());
        }
      }
    }
  }

  // Pose 計算を共通化
  void fill_pose_with_tf_and_origin(geometry_msgs::msg::Pose& pose,
                                    const geometry_msgs::msg::TransformStamped& transform,
                                    const urdf::Pose& origin)
  {
    pose.position.x = transform.transform.translation.x + origin.position.x;
    pose.position.y = transform.transform.translation.y + origin.position.y;
    pose.position.z = transform.transform.translation.z + origin.position.z;

    tf2::Quaternion tf_quat(transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w);
    tf2::Quaternion origin_quat(origin.rotation.x,
                                origin.rotation.y,
                                origin.rotation.z,
                                origin.rotation.w);
    tf2::Quaternion combined_quat = tf_quat * origin_quat;

    pose.orientation.x = combined_quat.x();
    pose.orientation.y = combined_quat.y();
    pose.orientation.z = combined_quat.z();
    pose.orientation.w = combined_quat.w();
  }

  // メンバ
  std::string planning_group_;
  std::string collision_objects_base_path_;
  std::string other_robots_config_str_;

  std::string planning_frame_;

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr move_group_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Options> move_group_options_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // robot_type ごとのURDF文字列
  std::map<std::string, std::string> robot_type_descriptions_;
  // robot_name → robot_type の対応
  std::map<std::string, std::string> robot_to_type_map_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollisionUpdater>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
