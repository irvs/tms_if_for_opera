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
#include "tms_if_for_opera/navigation2/navigation2_navigate_anywhere_deg.hpp"
// #include <glog/logging.h>

using std::placeholders::_1;
using std::placeholders::_2;

Navigation2NavigateAnywhereDeg::Navigation2NavigateAnywhereDeg() : rclcpp::Node("tms_if_navigate_anywhere_deg_node")
{
    this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
        this, "tms_rp_navigate_anywhere_deg",
        std::bind(&Navigation2NavigateAnywhereDeg::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Navigation2NavigateAnywhereDeg::handle_cancel, this, std::placeholders::_1),
        std::bind(&Navigation2NavigateAnywhereDeg::handle_accepted, this, std::placeholders::_1));

    
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
}

rclcpp_action::GoalResponse Navigation2NavigateAnywhereDeg::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const NavigateToPose::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Navigation2NavigateAnywhereDeg::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel tms_if_navigate_anywhere_deg_node node");
    if (client_future_goal_handle_.valid() &&
        client_future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        auto goal_handle = client_future_goal_handle_.get();
        action_client_->async_cancel_goal(goal_handle);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Navigation2NavigateAnywhereDeg::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    using namespace std::placeholders;
    std::thread{ std::bind(&Navigation2NavigateAnywhereDeg::execute, this, _1), goal_handle }.detach();
}

void Navigation2NavigateAnywhereDeg::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "tms_if_for_opera(tms_if_navigate_anywhere_deg_node) is executing...");
    current_goal_handle_ = goal_handle;

    auto result = std::make_shared<NavigateToPose::Result>();
    auto handle_error = [&](const std::string& message) {
        if (goal_handle->is_active())
        {
        goal_handle->abort(result);
        RCLCPP_INFO(this->get_logger(), message.c_str());
        }
        else
        {
        RCLCPP_INFO(this->get_logger(), "Goal is not active");
        }
    };

    auto received_goal = goal_handle->get_goal();
    auto goal_msg = *received_goal;

    //進捗状況を表示するFeedbackコールバックを設�?
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const auto& goal_handle) { goal_response_callback(goal_handle); };
    send_goal_options.feedback_callback = [this](const auto tmp, const auto feedback) {
        feedback_callback(tmp, feedback);
    };
    send_goal_options.result_callback = [this, goal_handle](const auto& result) { result_callback(goal_handle, result); };

    //Goal をサーバ�?�に送信
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    client_future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
}

void Navigation2NavigateAnywhereDeg::goal_response_callback(const GoalHandleNavigateToPose::SharedPtr& goal_handle)
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

  
void Navigation2NavigateAnywhereDeg::feedback_callback(
    const GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const GoalHandleNavigateToPose::Feedback> feedback)
{
  auto feedback_to_st_node = std::make_shared<NavigateToPose::Feedback>();
  *feedback_to_st_node = *feedback;
  
  // アクティブなゴールハンドルにフィードバックを送信
  if (current_goal_handle_ && current_goal_handle_->is_active()) {
    current_goal_handle_->publish_feedback(feedback_to_st_node);
  }
}


//result
void Navigation2NavigateAnywhereDeg::result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                                             const GoalHandleNavigateToPose::WrappedResult& result)
{
  if (!goal_handle->is_active())
  {
    RCLCPP_WARN(this->get_logger(), "Attempted to succeed an already succeeded goal");
    return;
  }

  auto result_to_st_node = std::make_shared<NavigateToPose::Result>();
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      goal_handle->succeed(result_to_st_node);
      RCLCPP_INFO(this->get_logger(), "tms if navigation2 execution is succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      goal_handle->abort(result_to_st_node);
      RCLCPP_INFO(this->get_logger(), "tms if navigation2 execution is aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      goal_handle->canceled(result_to_st_node);
      RCLCPP_INFO(this->get_logger(), "tms if navigation2 execution is canceled");
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
    rclcpp::spin(std::make_shared<Navigation2NavigateAnywhereDeg>());
    rclcpp::shutdown();
    return 0;
}
