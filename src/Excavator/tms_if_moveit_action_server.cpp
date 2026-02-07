#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "moveit_msgs/msg/move_it_error_codes.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

#include "tms_msg_rp/action/tms_rp_excavator.hpp"

using TmsRpExcavator = tms_msg_rp::action::TmsRpExcavator;
using GoalHandleTms = rclcpp_action::ServerGoalHandle<TmsRpExcavator>;

class TmsIfMoveItActionServer : public rclcpp::Node
{
public:
  explicit TmsIfMoveItActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("tms_if_moveit_action_server", options)
  {
    action_server_ = rclcpp_action::create_server<TmsRpExcavator>(
      this,
      "tms_rp_excavator",  // <-- Action名（トピック名）
      std::bind(&TmsIfMoveItActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TmsIfMoveItActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&TmsIfMoveItActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Action server started: /tms_rp_excavator");
  }

private:
  rclcpp_action::Server<TmsRpExcavator>::SharedPtr action_server_;

  // last plan cache
  std::mutex plan_mtx_;
  moveit::planning_interface::MoveGroupInterface::Plan last_plan_;
  bool has_last_plan_{false};

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const TmsRpExcavator::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "=== Received Goal ===");
    RCLCPP_INFO(get_logger(), "  Command: %d", goal->command);
    RCLCPP_INFO(get_logger(), "  Planning group: %s", goal->planning_group.c_str());
    RCLCPP_INFO(get_logger(), "  Pose sequence size: %zu", goal->pose_sequence.size());
    RCLCPP_INFO(get_logger(), "  Joint values sequence size: %zu", goal->joint_values_sequence.size());
    
    // コマンドに応じて planning_group が必要かチェック
    const bool needs_group =
      goal->command == TmsRpExcavator::Goal::CMD_PLAN_TO_POSE ||
      goal->command == TmsRpExcavator::Goal::CMD_PLAN_TO_JOINTS ||
      goal->command == TmsRpExcavator::Goal::CMD_PLAN_AND_EXECUTE_POSE ||
      goal->command == TmsRpExcavator::Goal::CMD_PLAN_AND_EXECUTE_JOINTS ||
      goal->command == TmsRpExcavator::Goal::CMD_EXECUTE_LAST_PLAN ||
      goal->command == TmsRpExcavator::Goal::CMD_EXECUTE_PLAN;

    if (needs_group && goal->planning_group.empty()) {
      RCLCPP_WARN(get_logger(), "Rejected: planning_group is empty for this command.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    RCLCPP_INFO(get_logger(), "Goal ACCEPTED");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTms> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleTms> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Goal accepted, starting execution thread");
    std::thread{std::bind(&TmsIfMoveItActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // -------- main execute --------

  void execute(const std::shared_ptr<GoalHandleTms> goal_handle)
  {
    RCLCPP_INFO(get_logger(), ">>> Execute thread started");
    
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<TmsRpExcavator::Result>();
    auto feedback = std::make_shared<TmsRpExcavator::Feedback>();

    auto publish_fb = [&](const std::string& st, float prog){
      RCLCPP_INFO(get_logger(), "Feedback: %s (%.1f%%)", st.c_str(), prog * 100.0f);
      feedback->state = st;
      feedback->progress = prog;
      goal_handle->publish_feedback(feedback);
    };

    auto finish = [&](bool ok, int32_t err_code, const std::string& msg){
      RCLCPP_INFO(get_logger(), "Finish: success=%d, error_code=%d, msg=%s", ok, err_code, msg.c_str());
      result->success = ok;
      result->moveit_error_code = err_code;
      result->message = msg;
      if (ok) goal_handle->succeed(result);
      else goal_handle->abort(result);
    };

    auto cancel_if_needed = [&](moveit::planning_interface::MoveGroupInterface* mg)->bool{
      if (goal_handle->is_canceling()) {
        RCLCPP_WARN(get_logger(), "Goal is being canceled");
        if (mg) mg->stop();
        result->success = false;
        result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
        result->message = "Canceled.";
        goal_handle->canceled(result);
        return true;
      }
      return false;
    };

    publish_fb("received", 0.05f);

    // ---- commands requiring MoveGroup ----
    publish_fb("initializing_move_group", 0.15f);
    
    RCLCPP_INFO(get_logger(), "Creating MoveGroupInterface with planning_group: %s", goal->planning_group.c_str());

    moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), goal->planning_group);
    
    RCLCPP_INFO(get_logger(), "MoveGroupInterface created successfully");

    // Apply path constraints if provided
    if (!goal->constraints.name.empty() || 
        !goal->constraints.joint_constraints.empty() ||
        !goal->constraints.position_constraints.empty() ||
        !goal->constraints.orientation_constraints.empty() ||
        !goal->constraints.visibility_constraints.empty()) {
      RCLCPP_INFO(get_logger(), "Applying path constraints");
      move_group.setPathConstraints(goal->constraints);
    }

    // Apply planning scene diff if provided
    if (!goal->planning_scene.name.empty() ||
        !goal->planning_scene.world.collision_objects.empty() ||
        goal->planning_scene.is_diff) {
      RCLCPP_INFO(get_logger(), "Applying planning scene");
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      
      // Apply the entire planning scene diff
      planning_scene_interface.applyPlanningScene(goal->planning_scene);
    }

    if (cancel_if_needed(&move_group)) return;

    if (goal->command == TmsRpExcavator::Goal::CMD_EXECUTE_LAST_PLAN) {
      publish_fb("executing_last_plan", 0.3f);

      moveit::planning_interface::MoveGroupInterface::Plan plan_copy;
      {
        std::lock_guard<std::mutex> lk(plan_mtx_);
        if (!has_last_plan_) {
          finish(false, moveit_msgs::msg::MoveItErrorCodes::FAILURE, "No last plan.");
          return;
        }
        plan_copy = last_plan_;
      }

      if (cancel_if_needed(&move_group)) return;

      auto exec_res = move_group.execute(plan_copy);
      if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
        finish(true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, "Executed last plan.");
      } else {
        finish(false, exec_res.val, "Execute last plan failed.");
      }
      return;
    }

    if (goal->command == TmsRpExcavator::Goal::CMD_EXECUTE_PLAN) {
      publish_fb("executing_provided_plan", 0.3f);

      if (goal->plan.empty()) {
        finish(false, moveit_msgs::msg::MoveItErrorCodes::FAILURE, "No plan provided.");
        return;
      }

      RCLCPP_INFO(get_logger(), "Executing %zu trajectory(ies) from provided plan", goal->plan.size());

      // 複数のtrajectoryを順次実行
      for (size_t i = 0; i < goal->plan.size(); ++i) {
        if (cancel_if_needed(&move_group)) return;

        moveit::planning_interface::MoveGroupInterface::Plan plan_to_execute;
        plan_to_execute.trajectory_ = goal->plan[i];

        publish_fb("executing_trajectory_" + std::to_string(i+1), 0.3f + 0.4f * (float)i / goal->plan.size());

        auto exec_res = move_group.execute(plan_to_execute);

        if (exec_res != moveit::core::MoveItErrorCode::SUCCESS) {
          finish(false, exec_res.val, "Execute plan failed at trajectory " + std::to_string(i+1));
          return;
        }

        RCLCPP_INFO(get_logger(), "Executed trajectory %zu/%zu", i+1, goal->plan.size());
      }

      finish(true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, 
             "Executed all " + std::to_string(goal->plan.size()) + " trajectory(ies).");
      return;
    }

    // ---- planning/execution ----
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (goal->command == TmsRpExcavator::Goal::CMD_PLAN_TO_POSE ||
        goal->command == TmsRpExcavator::Goal::CMD_PLAN_AND_EXECUTE_POSE)
    {
      publish_fb("setting_pose_targets", 0.25f);

      if (goal->pose_sequence.empty()) {
        finish(false, moveit_msgs::msg::MoveItErrorCodes::FAILURE, "pose_sequence is empty.");
        return;
      }

      // pose_sequenceの最初のポーズをターゲットとして設定
      move_group.setPoseTarget(goal->pose_sequence[0]);

      if (cancel_if_needed(&move_group)) return;

      publish_fb("planning", 0.45f);

      auto plan_res = move_group.plan(plan);
      if (plan_res != moveit::core::MoveItErrorCode::SUCCESS) {
        finish(false, plan_res.val, "Planning to pose failed.");
        return;
      }
    }
    else if (goal->command == TmsRpExcavator::Goal::CMD_PLAN_TO_JOINTS ||
             goal->command == TmsRpExcavator::Goal::CMD_PLAN_AND_EXECUTE_JOINTS)
    {
      publish_fb("setting_joint_targets", 0.25f);

      if (goal->joint_values_sequence.empty()) {
        finish(false, moveit_msgs::msg::MoveItErrorCodes::FAILURE, "joint_values_sequence is empty.");
        return;
      }

      // joint_values_sequenceの最初の値をターゲットとして設定
      const auto& jv = goal->joint_values_sequence[0];
      if (jv.joint_names.size() != jv.joint_values.size() || jv.joint_names.empty()) {
        finish(false, moveit_msgs::msg::MoveItErrorCodes::FAILURE,
               "joint_names and joint_values mismatch/empty.");
        return;
      }

      std::map<std::string, double> joint_map;
      for (size_t i = 0; i < jv.joint_names.size(); ++i) {
        joint_map[jv.joint_names[i]] = jv.joint_values[i];
      }
      move_group.setJointValueTarget(joint_map);

      if (cancel_if_needed(&move_group)) return;

      publish_fb("planning", 0.45f);

      auto plan_res = move_group.plan(plan);
      if (plan_res != moveit::core::MoveItErrorCode::SUCCESS) {
        finish(false, plan_res.val, "Planning to joints failed.");
        return;
      }
    }
    else {
      finish(false, moveit_msgs::msg::MoveItErrorCodes::FAILURE, "Unknown command.");
      return;
    }

    // cache last plan
    {
      std::lock_guard<std::mutex> lk(plan_mtx_);
      last_plan_ = plan;
      has_last_plan_ = true;
    }

    // Resultにplanを格納
    result->plan.push_back(plan.trajectory_);

    if (cancel_if_needed(&move_group)) return;

    // PLAN only
    if (goal->command == TmsRpExcavator::Goal::CMD_PLAN_TO_POSE ||
        goal->command == TmsRpExcavator::Goal::CMD_PLAN_TO_JOINTS)
    {
      finish(true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, "Planned (cached as last plan).");
      return;
    }

    // PLAN + EXECUTE
    publish_fb("executing", 0.75f);

    if (cancel_if_needed(&move_group)) return;

    auto exec_res = move_group.execute(plan);
    if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
      finish(true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, "Executed.");
    } else {
      finish(false, exec_res.val, "Execute failed.");
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TmsIfMoveItActionServer>());
  rclcpp::shutdown();
  return 0;
}
