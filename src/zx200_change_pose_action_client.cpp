#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tms_msg_rp/action/tms_rp_zx200_change_pose.hpp"

class Zx200ChangePoseActionClient : public rclcpp::Node
{
public:
  using Zx200ChangePose = tms_msg_rp::action::TmsRpZx200ChangePose;
  using GoalHandleZx200ChangePose = rclcpp_action::ClientGoalHandle<Zx200ChangePose>;

  explicit Zx200ChangePoseActionClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("zx200_change_pose_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Zx200ChangePose>(this, "tms_rp_zx200_change_pose");

    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                           std::bind(&Zx200ChangePoseActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Action server is available");

    auto goal_msg = Zx200ChangePose::Goal();
    goal_msg.position_with_angle_sequence.resize(1);
    // Confirm to move
    // goal_msg.position_with_angle_sequence[0].position.x = 6.5003;
    // goal_msg.position_with_angle_sequence[0].position.y = 5.2609;
    // goal_msg.position_with_angle_sequence[0].position.z = 2.5797;
    // goal_msg.position_with_angle_sequence[0].theta_w = 0.5;
    // For test to use constraints
    // if (pose_num == 0)
    // {
    //   goal_msg.position_with_angle_sequence[0].position.x = 5.5003;
    //   goal_msg.position_with_angle_sequence[0].position.y = 4.2609;
    //   goal_msg.position_with_angle_sequence[0].position.z = 1.5797;
    //   goal_msg.position_with_angle_sequence[0].theta_w = M_PI / 2.0;
    // }
    // else
    // {
    //   goal_msg.position_with_angle_sequence[0].position.x = 6.3724;
    //   goal_msg.position_with_angle_sequence[0].position.y = -0.086909;
    //   goal_msg.position_with_angle_sequence[0].position.z = 1.3456;
    //   goal_msg.position_with_angle_sequence[0].theta_w = M_PI / 2.0;
    // }
    goal_msg.position_with_angle_sequence[0].position.x = 1.2068;
    goal_msg.position_with_angle_sequence[0].position.y = -5.5729;
    goal_msg.position_with_angle_sequence[0].position.z = 2.7381;
    goal_msg.position_with_angle_sequence[0].theta_w = 180.0 * M_PI / 180.0;

    // moveit_msgs::msg::Constraints pose_constraints;
    // moveit_msgs::msg::OrientationConstraint ocm;
    // ocm.link_name = "bucket_end_link";  // Replace with your end effector link name
    // ocm.header.frame_id = "base_link";  // Replace with your base link name
    // Specify the desired orientation
    // Eigen::Quaterniond eigen_quaternion(current_end_effector_state.rotation());
    // geometry_msgs::msg::Quaternion geometry_quaternion;
    // geometry_quaternion.x = eigen_quaternion.x();
    // geometry_quaternion.y = eigen_quaternion.y();
    // geometry_quaternion.z = eigen_quaternion.z();
    // geometry_quaternion.w = eigen_quaternion.w();
    // ocm.orientation = geometry_quaternion;
    // ocm.absolute_x_axis_tolerance = 2.0;   // fail at 0.423599
    // ocm.absolute_y_axis_tolerance = 2.0;   // fail at 0.023599
    // ocm.absolute_z_axis_tolerance = M_PI;  // fail at 0.423599
    // ocm.weight = 1.0;
    // goal_msg.constraints.orientation_constraints.push_back(ocm);

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Zx200ChangePose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&Zx200ChangePoseActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&Zx200ChangePoseActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&Zx200ChangePoseActionClient::result_callback, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

    // debug
    // home pose
    // Eigen::Quaterniond q(0.41981, -5.5067e-06, 0.90761, 2.5471e-06);
    // after rotation
    // Eigen::Quaterniond q(0.39964, -0.30216, 0.85381, 0.14143);
    // Eigen::Vector3d euler_angles = q.toRotationMatrix().eulerAngles(2, 1, 0);  // roll, pitch, yaw
    // double roll = euler_angles[0];
    // double pitch = euler_angles[1];
    // double yaw = euler_angles[2];
    // RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
  }

private:
  rclcpp_action::Client<Zx200ChangePose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  double tolerance_ = M_PI / 6.0;
  int pose_num = 0;
  int count_ = 0;

  void goal_response_callback(const GoalHandleZx200ChangePose::SharedPtr& goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(const GoalHandleZx200ChangePose::SharedPtr,
                         const std::shared_ptr<const GoalHandleZx200ChangePose::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Feedback received: ";

    ss << feedback->state << " ";

    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleZx200ChangePose::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal was succeeded");
        tolerance_ -= 0.1;
        pose_num = (pose_num + 1) % 2;
        // count_ += 10;
        // RCLCPP_INFO(this->get_logger(), "tolerance: %f", tolerance_);
        // send_goal();
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        // rclcpp::shutdown();
        // return;
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        // rclcpp::shutdown();
        // return;
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        // rclcpp::shutdown();
        return;
    }
    // count_ += 10;
    // RCLCPP_INFO(this->get_logger(), "count: %d", count_);
    // send_goal();
    // std::stringstream ss;
    // ss << "Result received: ";
    // ss << result.result->error_code << " ";
    // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    // rclcpp::shutdown();
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<Zx200ChangePoseActionClient>();
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}