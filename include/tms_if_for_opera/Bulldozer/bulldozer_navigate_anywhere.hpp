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

#ifndef NAVIGATION2_NAVIGATE_ANYWHERE_HPP
#define NAVIGATION2_NAVIGATE_ANYWHERE_HPP

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
#include "nav2_msgs/action/navigate_to_pose.hpp"


class Navigation2NavigateAnywhere : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    Navigation2NavigateAnywhere();


private:
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const NavigateToPose::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    std::shared_ptr<GoalHandle> current_goal_handle_;

    // Member as an action client
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    std::shared_future<GoalHandleNavigateToPose::SharedPtr> client_future_goal_handle_;
    std::map<std::pair<std::string, std::string>, double> parameters;
    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr& goal_handle);
    void feedback_callback(GoalHandleNavigateToPose::SharedPtr,
                            const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                        const GoalHandleNavigateToPose::WrappedResult& result);
};

#endif