// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.

//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//      http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vector>
#include "tms_if_for_opera/Excavator/excavator_follow_straight.hpp"
// #include <glog/logging.h>

using std::placeholders::_1;
using std::placeholders::_2;

ExcavatorFollowStraight::ExcavatorFollowStraight() : rclcpp::Node("tms_if_follow_straight_node")
{
    this->declare_parameter<std::string>( "behavior_tree", "" );

    this->action_server_ = rclcpp_action::create_server<FollowStraight>(
        this, "tms_rp_navigate_follow_straight",
        std::bind(&ExcavatorFollowStraight::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ExcavatorFollowStraight::handle_cancel, this, std::placeholders::_1),
        std::bind(&ExcavatorFollowStraight::handle_accepted, this, std::placeholders::_1));

    
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
}

rclcpp_action::GoalResponse ExcavatorFollowStraight::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowStraight::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExcavatorFollowStraight::handle_cancel(const std::shared_ptr<GoalHandleFollowStraight> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel tms_if_follow_straight_node node");
    if (client_future_goal_handle_.valid() &&
        client_future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        auto goal_handle = client_future_goal_handle_.get();
        action_client_->async_cancel_goal(goal_handle);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ExcavatorFollowStraight::handle_accepted(const std::shared_ptr<GoalHandleFollowStraight> goal_handle)
{
    using namespace std::placeholders;
    std::thread{ std::bind(&ExcavatorFollowStraight::execute, this, _1), goal_handle }.detach();
}

void ExcavatorFollowStraight::execute(const std::shared_ptr<GoalHandleFollowStraight> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "tms_if_for_opera(tms_if_follow_straight_node) is executing...");
    current_goal_handle_ = goal_handle;

    auto result = std::make_shared<FollowStraight::Result>();


    // ============================================================
    // Check NavigateToPose server
    // ============================================================
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server is not available");

      result->result = false;
      goal_handle->abort(result);

      return;
    }

    // ============================================================
    // Get FollowStraight goal
    // ============================================================
    auto received_goal = goal_handle->get_goal();

    RCLCPP_INFO(this->get_logger(), "Received FollowStraight Goal: x=%.3f, y=%.3f", received_goal->goal.pose.position.x, received_goal->goal.pose.position.y);

    // ============================================================
    // NavigateToPose Goal
    // ============================================================
    NavigateToPose::Goal nav_goal;


    // ------------------------------------------------------------
    // Goal Pose
    // ------------------------------------------------------------
    nav_goal.pose = received_goal->goal;

    RCLCPP_INFO(this->get_logger(), "Sending NavigateToPose Goal: x=%.3f, y=%.3f", nav_goal.pose.pose.position.x, nav_goal.pose.pose.position.y);


    // ============================================================
    // Behavior Tree
    // ============================================================

    // nav_goal.behavior_tree ="/home/common/ros2-tms-for-construction_ws/src/opera/zx200/zx200_ros2/zx200_straight_navigation/linear_path_controller/straight_navigation.xml";

    const std::string behavior_tree = this->get_parameter("behavior_tree").as_string(); 
    
    if (behavior_tree.empty()) {
      RCLCPP_ERROR( this->get_logger(), "Parameter 'behavior_tree' is empty"); 
      result->result = false; goal_handle->abort(result); return; 
    } 
      
    nav_goal.behavior_tree = behavior_tree;

    RCLCPP_INFO(this->get_logger(), "Sending NavigateToPose goal");
    RCLCPP_INFO(this->get_logger(), "Behavior Tree: %s", nav_goal.behavior_tree.c_str());

  // ============================================================
  // Send Goal Options
  // ============================================================
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback = [this](const GoalHandleNavigateToPose::SharedPtr & handle){goal_response_callback(handle);};
  send_goal_options.feedback_callback = [this](const GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> )
    {
      // 必要ならここでFollowStraightのFeedbackを送る
      RCLCPP_DEBUG(
        this->get_logger(),
        "NavigateToPose feedback received");
    };

  send_goal_options.result_callback = [this, goal_handle](const GoalHandleNavigateToPose::WrappedResult & result){result_callback(goal_handle, result);};

  // ============================================================
  // Send NavigateToPose
  // ============================================================

  //Goal をサーバ�?�に送信
  RCLCPP_INFO(this->get_logger(), "Sending goal");
  client_future_goal_handle_ = action_client_->async_send_goal(nav_goal, send_goal_options);
}



void ExcavatorFollowStraight::goal_response_callback(const GoalHandleNavigateToPose::SharedPtr& goal_handle)
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


void ExcavatorFollowStraight::feedback_callback(
    const std::shared_ptr<GoalHandleFollowStraight> goal_handle,
    const std::shared_ptr<const FollowStraight::Feedback> feedback)
{
    auto feedback_to_st_node = std::make_shared<FollowStraight::Feedback>();
    *feedback_to_st_node = *feedback;

    // アクティブなゴールハンドルにフィードバックを送信
    if (current_goal_handle_ && current_goal_handle_->is_active()) {
        current_goal_handle_->publish_feedback(feedback_to_st_node);
    }
}


//result
void ExcavatorFollowStraight::result_callback(const std::shared_ptr<GoalHandleFollowStraight> goal_handle,
                                             const GoalHandleNavigateToPose::WrappedResult& result)
{
  if (!goal_handle->is_active())
  {
    RCLCPP_WARN(this->get_logger(), "Attempted to succeed an already succeeded goal");
    return;
  }

  auto result_to_st_node = std::make_shared<FollowStraight::Result>();
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      goal_handle->succeed(result_to_st_node);
      RCLCPP_INFO(this->get_logger(), "tms if excavator execution is succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      goal_handle->abort(result_to_st_node);
      RCLCPP_INFO(this->get_logger(), "tms if excavator execution is aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      goal_handle->canceled(result_to_st_node);
      RCLCPP_INFO(this->get_logger(), "tms if excavator execution is canceled");
      break;
    default:
      goal_handle->abort(result_to_st_node);
      RCLCPP_INFO(this->get_logger(), "Unknown result code");
      break;
  }
}

int main(int argc, char* argv[])
{
    // Initialize Google's logging library.
    //   google::InitGoogleLogging(argv[0]);
    //   google::InstallFailureSignalHandler();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExcavatorFollowStraight>());
    rclcpp::shutdown();
    return 0;
}
