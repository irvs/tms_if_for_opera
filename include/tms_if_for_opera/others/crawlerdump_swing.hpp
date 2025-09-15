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

#ifndef CRAWLERDUMP_SWING_HPP
#define CRAWLERDUMP_SWING_HPP

#include <memory>
#include <map>

#include <chrono>
#include <functional>
#include <future>
#include <string>
#include <sstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tms_msg_rp/action/tms_rp_crawler_dump_swing_angle.hpp"
#include "com3_msgs/action/set_swing_angle.hpp"


class CrawlerDumpSwing : public rclcpp::Node
{
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<tms_msg_rp::action::TmsRpCrawlerDumpSwingAngle>;
    using SetSwingAngle = com3_msgs::action::SetSwingAngle;
    using GoalHandleCrawlerDumpSwing = rclcpp_action::ClientGoalHandle<SetSwingAngle>;
    CrawlerDumpSwing();


private:
    rclcpp_action::Server<tms_msg_rp::action::TmsRpCrawlerDumpSwingAngle>::SharedPtr action_server_;
    std::map<std::pair<std::string, std::string>, double> param_from_db_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const tms_msg_rp::action::TmsRpCrawlerDumpSwingAngle::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    std::shared_ptr<GoalHandle> current_goal_handle_;

    // Member as an action client
    rclcpp_action::Client<SetSwingAngle>::SharedPtr action_client_;
    std::shared_future<GoalHandleCrawlerDumpSwing::SharedPtr> client_future_goal_handle_;
    std::map<std::string, double> parameters;
    void goal_response_callback(const GoalHandleCrawlerDumpSwing::SharedPtr& goal_handle);
    void feedback_callback(GoalHandleCrawlerDumpSwing::SharedPtr,
                            const std::shared_ptr<const SetSwingAngle::Feedback> feedback);
    void result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                        const GoalHandleCrawlerDumpSwing::WrappedResult& result);
};

#endif