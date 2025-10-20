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
#include "tms_if_for_opera/others/crawlerdump_release_soil.hpp"
// #include <glog/logging.h>

using std::placeholders::_1;
using std::placeholders::_2;

CrawlerDumpReleaseSoil::CrawlerDumpReleaseSoil() : rclcpp::Node("tms_if_crawlerdump_release_soil_node")
{
    this->action_server_ = rclcpp_action::create_server<tms_msg_rp::action::TmsRpCrawlerDumpDumpAngle>(
        this, "tms_rp_set_dump_angle",
        std::bind(&CrawlerDumpReleaseSoil::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CrawlerDumpReleaseSoil::handle_cancel, this, std::placeholders::_1),
        std::bind(&CrawlerDumpReleaseSoil::handle_accepted, this, std::placeholders::_1));

    
    action_client_ = rclcpp_action::create_client<SetDumpAngle>(this, "set_dump_angle");
}

rclcpp_action::GoalResponse CrawlerDumpReleaseSoil::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const tms_msg_rp::action::TmsRpCrawlerDumpDumpAngle::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CrawlerDumpReleaseSoil::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel tms_if_crawlerdump_release_soil_node node");
    if (client_future_goal_handle_.valid() &&
        client_future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        auto goal_handle = client_future_goal_handle_.get();
        action_client_->async_cancel_goal(goal_handle);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CrawlerDumpReleaseSoil::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    using namespace std::placeholders;
    std::thread{ std::bind(&CrawlerDumpReleaseSoil::execute, this, _1), goal_handle }.detach();
}

void CrawlerDumpReleaseSoil::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "tms_if_for_opera(tms_if_crawlerdump_release_soil) is executing...");
    current_goal_handle_ = goal_handle;
    auto result = std::make_shared<tms_msg_rp::action::TmsRpCrawlerDumpDumpAngle::Result>();
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

    auto goal_msg = SetDumpAngle::Goal();
    auto received_goal = goal_handle->get_goal();
    goal_msg.target_angle = received_goal->target_angle;

    //進捗状況を表示するFeedbackコールバックを設�?
    auto send_goal_options = rclcpp_action::Client<SetDumpAngle>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const auto& goal_handle) { goal_response_callback(goal_handle); };
    send_goal_options.feedback_callback = [this](const auto tmp, const auto feedback) {
        feedback_callback(tmp, feedback);
    };
    send_goal_options.result_callback = [this, goal_handle](const auto& result) { result_callback(goal_handle, result); };

    //Goal をサーバ�?�に送信
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    client_future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
}

void CrawlerDumpReleaseSoil::goal_response_callback(const GoalHandleCrawlerDumpReleaseSoil::SharedPtr& goal_handle)
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

  
void CrawlerDumpReleaseSoil::feedback_callback(
    const GoalHandleCrawlerDumpReleaseSoil::SharedPtr,
    const std::shared_ptr<const GoalHandleCrawlerDumpReleaseSoil::Feedback> feedback)
{

}


//result
void CrawlerDumpReleaseSoil::result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                                             const GoalHandleCrawlerDumpReleaseSoil::WrappedResult& result)
{
  if (!goal_handle->is_active())
  {
    RCLCPP_WARN(this->get_logger(), "Attempted to succeed an already succeeded goal");
    return;
  }

  auto result_to_st_node = std::make_shared<tms_msg_rp::action::TmsRpCrawlerDumpDumpAngle::Result>();
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      goal_handle->succeed(result_to_st_node);
      RCLCPP_INFO(this->get_logger(), "tms if release_soil is succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      goal_handle->abort(result_to_st_node);
      RCLCPP_INFO(this->get_logger(), "tms if release_soil is aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      goal_handle->canceled(result_to_st_node);
      RCLCPP_INFO(this->get_logger(), "tms if release_soil is canceled");
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
    rclcpp::spin(std::make_shared<CrawlerDumpReleaseSoil>());
    rclcpp::shutdown();
    return 0;
}
