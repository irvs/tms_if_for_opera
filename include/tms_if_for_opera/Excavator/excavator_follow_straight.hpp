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

#ifndef EXCAVATOR_FOLLOW_STRAIGHT_HPP
#define EXCAVATOR_FOLLOW_STRAIGHT_HPP

#include <memory>
#include <thread>
#include <future>
// #include <chrono>
// #include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "std_msgs/msg/float64.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav_msgs/msg/path.hpp"

// #include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tms_msg_rp/action/tms_rp_follow_straight.hpp"

class ExcavatorFollowStraight : public rclcpp::Node
{
public:
  using FollowStraight = tms_msg_rp::action::TmsRpFollowStraight;
  using GoalHandleFollowStraight = rclcpp_action::ServerGoalHandle<FollowStraight>;
  // using FollowPath = nav2_msgs::action::FollowPath;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  // using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;


  ExcavatorFollowStraight();

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowStraight::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowStraight> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleFollowStraight> goal_handle);

  void execute(
    const std::shared_ptr<GoalHandleFollowStraight> goal_handle);

  void goal_response_callback(
    const GoalHandleNavigateToPose::SharedPtr & goal_handle);

  void feedback_callback(
    const std::shared_ptr<GoalHandleFollowStraight> goal_handle,
    const std::shared_ptr<const FollowStraight::Feedback> feedback);

  void result_callback(
    const std::shared_ptr<GoalHandleFollowStraight> goal_handle,
    const GoalHandleNavigateToPose::WrappedResult & result);

  // nav_msgs::msg::Path createStraightPath(
  //   const geometry_msgs::msg::PoseStamped & start,
  //   const geometry_msgs::msg::PoseStamped & goal);

  rclcpp_action::Server<FollowStraight>::SharedPtr action_server_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  std::shared_future<GoalHandleNavigateToPose::SharedPtr>
    client_future_goal_handle_;

  std::shared_ptr<GoalHandleFollowStraight>
    current_goal_handle_;
};

#endif