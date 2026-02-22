#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include <algorithm>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/action/move_group_sequence.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"

#include "tms_msg_rp/action/tms_rp_excavator.hpp"
#include "tms_msg_rp/srv/tms_rp_excavator_param_get.hpp"
#include "tms_msg_rp/srv/tms_rp_excavator_param_set.hpp"
#include <unordered_map>

using TmsRpExcavator = tms_msg_rp::action::TmsRpExcavator;
using GoalHandleTms = rclcpp_action::ServerGoalHandle<TmsRpExcavator>;
using TmsRpExcavatorParamGet = tms_msg_rp::srv::TmsRpExcavatorParamGet;
using TmsRpExcavatorParamSet = tms_msg_rp::srv::TmsRpExcavatorParamSet;
using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;

class TmsIfMoveItActionServer : public rclcpp::Node
{
public:
  explicit TmsIfMoveItActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("tms_if_moveit_action_server", options)
  {
    // パラメータの取得
    this->declare_parameter<std::string>("planning_group", "manipulator");
    this->get_parameter("planning_group", planning_group_);
    RCLCPP_INFO(this->get_logger(), "Planning group: %s", planning_group_.c_str());

    // MoveGroupInterface用の専用ノードを作成
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    // 現在のノードのすべてのパラメータを取得して専用ノードに引き継ぐ
    auto param_names = this->list_parameters({}, 0).names;
    for (const auto& param_name : param_names)
    {
      rclcpp::Parameter param = this->get_parameter(param_name);
      node_options.append_parameter_override(param_name, param.get_parameter_value());
    }
    
    // 専用ノードを作成（ネームスペースを保持）
    std::string node_namespace = this->get_namespace();
    move_group_node_ = rclcpp::Node::make_shared(
      std::string(this->get_name()) + "_move_group", 
      node_namespace,
      node_options
    );
    
    // Robot状態監視のため、専用のExecutorでスピン（detachで常駐）
    executor_.add_node(move_group_node_);
    std::thread([this]() { 
      RCLCPP_INFO(this->get_logger(), "MoveGroup executor thread started");
      executor_.spin(); 
    }).detach();
    
    RCLCPP_INFO(get_logger(), "MoveGroup node created: %s", move_group_node_->get_name());
    
    // MoveGroupInterfaceを一度だけ初期化
    RCLCPP_INFO(get_logger(), "Creating MoveGroupInterface with planning_group: %s", planning_group_.c_str());
    move_group_options_ = std::make_shared<moveit::planning_interface::MoveGroupInterface::Options>(
        planning_group_, "robot_description", node_namespace);
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        move_group_node_, *move_group_options_);
    
    RCLCPP_INFO(get_logger(), "MoveGroupInterface created successfully");
    
    action_server_ = rclcpp_action::create_server<TmsRpExcavator>(
      this,
      "tms_rp_excavator",
      std::bind(&TmsIfMoveItActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TmsIfMoveItActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&TmsIfMoveItActionServer::handle_accepted, this, std::placeholders::_1)
    );

    // サービスサーバーの作成
    param_get_service_ = this->create_service<TmsRpExcavatorParamGet>(
      "tms_rp_excavator_param_get",
      std::bind(&TmsIfMoveItActionServer::handle_param_get, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    param_set_service_ = this->create_service<TmsRpExcavatorParamSet>(
      "tms_rp_excavator_param_set",
      std::bind(&TmsIfMoveItActionServer::handle_param_set, this, std::placeholders::_1, std::placeholders::_2)
    );

    // MoveGroupSequenceアクションクライアントの作成
    move_group_sequence_client_ = rclcpp_action::create_client<MoveGroupSequence>(
      this, 
      "sequence_move_group"
    );

    RCLCPP_INFO(get_logger(), "Action server ready.");
    RCLCPP_INFO(get_logger(), "Services ready: tms_rp_excavator_param_get, tms_rp_excavator_param_set");
  }
  
  ~TmsIfMoveItActionServer()
  {
    executor_.cancel();
  }

private:
  rclcpp_action::Server<TmsRpExcavator>::SharedPtr action_server_;
  
  // MoveGroupInterface用の専用ノードとExecutor
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  
  // MoveGroupInterfaceをメンバー変数として保持
  std::string planning_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Options> move_group_options_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // MoveGroupの設定値を保存（Getterがないため）
  double current_max_velocity_scaling_factor_{1.0};
  double current_max_acceleration_scaling_factor_{1.0};
  int current_num_planning_attempts_{10};
  std::string current_planner_id_;
  std::string current_planning_pipeline_id_;

  // last plan cache
  std::mutex plan_mtx_;
  moveit::planning_interface::MoveGroupInterface::Plan last_plan_;
  bool has_last_plan_{false};

  // サービスサーバー
  rclcpp::Service<TmsRpExcavatorParamGet>::SharedPtr param_get_service_;
  rclcpp::Service<TmsRpExcavatorParamSet>::SharedPtr param_set_service_;

  // MoveGroupSequenceアクションクライアント
  rclcpp_action::Client<MoveGroupSequence>::SharedPtr move_group_sequence_client_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const TmsRpExcavator::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "=== Received Goal ===");
    RCLCPP_INFO(get_logger(), "  Command: %d", goal->command);
    RCLCPP_INFO(get_logger(), "  Planning group: %s (ignored, using fixed planning_group from parameter)", goal->planning_group.c_str());
    RCLCPP_INFO(get_logger(), "  Pose sequence size: %zu", goal->pose_sequence.size());
    RCLCPP_INFO(get_logger(), "  Joint values sequence size: %zu", goal->joint_values_sequence.size());
    
    // 注意: 現在の実装では、ゴールで指定されたplanning_groupは無視され、
    // コンストラクタで初期化した固定のplanning_groupが使用されます。
    // 動的にplanning_groupを切り替えたい場合は、execute()内で
    // MoveGroupInterfaceを毎回初期化する必要があります。
    
    // コマンドに応じて planning_group が必要かチェック
    const bool needs_group =
      goal->command == TmsRpExcavator::Goal::CMD_PLAN_TO_POSE ||
      goal->command == TmsRpExcavator::Goal::CMD_PLAN_TO_JOINTS ||
      goal->command == TmsRpExcavator::Goal::CMD_PLAN_AND_EXECUTE_POSE ||
      goal->command == TmsRpExcavator::Goal::CMD_PLAN_AND_EXECUTE_JOINTS ||
      goal->command == TmsRpExcavator::Goal::CMD_EXECUTE_LAST_PLAN ||
      goal->command == TmsRpExcavator::Goal::CMD_EXECUTE_PLAN ||
      goal->command == TmsRpExcavator::Goal::CMD_PLAN_MOTION_SEQUENCE ||
      goal->command == TmsRpExcavator::Goal::CMD_PLAN_AND_EXECUTE_MOTION_SEQUENCE;

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
    
    RCLCPP_INFO(get_logger(), "Using MoveGroupInterface with planning_group: %s", planning_group_.c_str());

    // 前回のゴールで設定された制約をクリア
    move_group_->clearPathConstraints();
    RCLCPP_INFO(get_logger(), "Cleared previous path constraints");

    // Planning sceneをクリーンな状態に初期化
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    // すべての既存の障害物（collision objects）を削除
    std::vector<std::string> object_ids = planning_scene_interface.getKnownObjectNames();
    if (!object_ids.empty()) {
      RCLCPP_INFO(get_logger(), "Removing %zu existing collision objects", object_ids.size());
      planning_scene_interface.removeCollisionObjects(object_ids);
    }
    
    // ロボットに取り付けられたオブジェクトも削除
    std::map<std::string, moveit_msgs::msg::AttachedCollisionObject> attached_objects = 
        planning_scene_interface.getAttachedObjects();
    if (!attached_objects.empty()) {
      RCLCPP_INFO(get_logger(), "Removing %zu attached objects", attached_objects.size());
      std::vector<std::string> attached_object_ids;
      for (const auto& obj : attached_objects) {
        attached_object_ids.push_back(obj.first);
      }
      planning_scene_interface.removeCollisionObjects(attached_object_ids);
    }
    
    RCLCPP_INFO(get_logger(), "Planning scene cleared");

    // Apply path constraints if provided
    if (!goal->constraints.name.empty() || 
        !goal->constraints.joint_constraints.empty() ||
        !goal->constraints.position_constraints.empty() ||
        !goal->constraints.orientation_constraints.empty() ||
        !goal->constraints.visibility_constraints.empty()) {
      RCLCPP_INFO(get_logger(), "Applying path constraints");
      move_group_->setPathConstraints(goal->constraints);
    }

    // Apply planning scene diff if provided
    if (!goal->planning_scene.name.empty() ||
        !goal->planning_scene.world.collision_objects.empty() ||
        goal->planning_scene.is_diff) {
      RCLCPP_INFO(get_logger(), "Applying planning scene");
      
      // Apply the entire planning scene diff
      planning_scene_interface.applyPlanningScene(goal->planning_scene);
    }

    if (cancel_if_needed(move_group_.get())) return;

    // ---- Motion Sequence commands ----
    if (goal->command == TmsRpExcavator::Goal::CMD_PLAN_MOTION_SEQUENCE ||
        goal->command == TmsRpExcavator::Goal::CMD_PLAN_AND_EXECUTE_MOTION_SEQUENCE)
    {
      publish_fb("preparing_motion_sequence", 0.25f);

      if (goal->motion_sequence_items.empty()) {
        finish(false, moveit_msgs::msg::MoveItErrorCodes::FAILURE, "motion_sequence_items is empty.");
        return;
      }

      RCLCPP_INFO(get_logger(), "Processing motion sequence with %zu items", goal->motion_sequence_items.size());

      // MoveGroupSequenceアクションクライアントが利用可能か確認
      if (!move_group_sequence_client_->wait_for_action_server(std::chrono::seconds(5))) {
        finish(false, moveit_msgs::msg::MoveItErrorCodes::FAILURE, 
               "MoveGroupSequence action server not available.");
        return;
      }

      // MoveGroupSequenceゴールを作成
      auto sequence_goal = MoveGroupSequence::Goal();
      sequence_goal.request.items = goal->motion_sequence_items;

      if (!sequence_goal.request.items.empty()) {
        if (const auto* prev = pickPrevTrajectory(goal->previous_pose)) {
          moveit_msgs::msg::RobotState rs;
          if (fillRobotStateFromPrevTrajectoryLastPoint(*prev, rs, get_logger())) {
            sequence_goal.request.items[0].req.start_state = rs;
            RCLCPP_INFO(get_logger(), "Injected start_state into motion_sequence item[0].");
          }
        }
      }
      
      // Planning optionsを設定
      sequence_goal.planning_options.planning_scene_diff = goal->planning_scene;
      
      // CMD_PLAN_MOTION_SEQUENCEの場合はプランのみ
      if (goal->command == TmsRpExcavator::Goal::CMD_PLAN_MOTION_SEQUENCE) {
        sequence_goal.planning_options.plan_only = true;
        RCLCPP_INFO(get_logger(), "Plan only mode for motion sequence");
      } else {
        sequence_goal.planning_options.plan_only = false;
        RCLCPP_INFO(get_logger(), "Plan and execute mode for motion sequence");
      }

      if (cancel_if_needed(move_group_.get())) return;

      publish_fb("sending_sequence_goal", 0.35f);

      // ゴールを送信
      auto send_goal_options = rclcpp_action::Client<MoveGroupSequence>::SendGoalOptions();
      
      // フィードバックコールバック
      send_goal_options.feedback_callback = 
        [&](rclcpp_action::ClientGoalHandle<MoveGroupSequence>::SharedPtr,
            const std::shared_ptr<const MoveGroupSequence::Feedback> sequence_feedback)
        {
          RCLCPP_INFO(get_logger(), "Sequence feedback: %s", sequence_feedback->state.c_str());
          publish_fb("sequence_" + sequence_feedback->state, 0.5f);
        };

      // 結果を受け取るための変数
      std::promise<MoveGroupSequence::Result::SharedPtr> result_promise;
      auto result_future = result_promise.get_future();

      // 結果コールバック
      send_goal_options.result_callback = 
        [&](const rclcpp_action::ClientGoalHandle<MoveGroupSequence>::WrappedResult& wrapped_result)
        {
          result_promise.set_value(wrapped_result.result);
        };

      auto goal_handle_future = move_group_sequence_client_->async_send_goal(sequence_goal, send_goal_options);

      // ゴールが受け入れられるまで待機
      if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        finish(false, moveit_msgs::msg::MoveItErrorCodes::FAILURE, 
               "Failed to send motion sequence goal.");
        return;
      }

      auto goal_handle = goal_handle_future.get();
      if (!goal_handle) {
        finish(false, moveit_msgs::msg::MoveItErrorCodes::FAILURE, 
               "Motion sequence goal was rejected.");
        return;
      }

      RCLCPP_INFO(get_logger(), "Motion sequence goal accepted, waiting for result...");

      if (cancel_if_needed(move_group_.get())) return;

      publish_fb("executing_sequence", 0.50f);

      // 結果を待機
      auto wait_result = result_future.wait_for(std::chrono::seconds(300)); // 5分のタイムアウト
      
      if (wait_result != std::future_status::ready) {
        // タイムアウトまたはキャンセル
        move_group_sequence_client_->async_cancel_goal(goal_handle);
        finish(false, moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT, 
               "Motion sequence execution timed out.");
        return;
      }

      auto sequence_result = result_future.get();

      if (cancel_if_needed(move_group_.get())) return;

      // 結果を処理
      if (sequence_result->response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Motion sequence completed successfully");
        
        // 計画された軌道をresultに格納
        if (!sequence_result->response.planned_trajectories.empty()) {
          result->plan = sequence_result->response.planned_trajectories;
          RCLCPP_INFO(get_logger(), "Stored %zu planned trajectories", result->plan.size());
        }

        if (goal->command == TmsRpExcavator::Goal::CMD_PLAN_MOTION_SEQUENCE) {
          finish(true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, 
                 "Motion sequence planned successfully.");
        } else {
          finish(true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, 
                 "Motion sequence executed successfully.");
        }
      } else {
        std::string error_msg = "Motion sequence failed with error code: " + 
                               std::to_string(sequence_result->response.error_code.val);
        RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
        finish(false, sequence_result->response.error_code.val, error_msg);
      }
      
      return;
    }

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

      if (cancel_if_needed(move_group_.get())) return;

      auto exec_res = move_group_->execute(plan_copy);
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
        if (cancel_if_needed(move_group_.get())) return;

        moveit::planning_interface::MoveGroupInterface::Plan plan_to_execute;
        plan_to_execute.trajectory_ = goal->plan[i];

        publish_fb("executing_trajectory_" + std::to_string(i+1), 0.3f + 0.4f * (float)i / goal->plan.size());

        auto exec_res = move_group_->execute(plan_to_execute);

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
      if (const auto* prev = pickPrevTrajectory(goal->previous_pose)) {
        if (!setStartStateFromPrevRobotTrajectory(*prev, *move_group_, planning_group_, get_logger())) {
          return;
        }
      } else {
        move_group_->setStartStateToCurrentState();
      } 

      // pose_sequenceの最初のポーズをターゲットとして設定
      move_group_->setPoseTarget(goal->pose_sequence[0]);

      if (cancel_if_needed(move_group_.get())) return;

      publish_fb("planning", 0.45f);

      auto plan_res = move_group_->plan(plan);
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

      if (const auto* prev = pickPrevTrajectory(goal->previous_pose)) {
        if (!setStartStateFromPrevRobotTrajectory(*prev, *move_group_, planning_group_, get_logger())) {
          return;
        }
      } else {
        move_group_->setStartStateToCurrentState();
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
      auto current_state = move_group_->getCurrentState();
      const auto* jmg = current_state->getJointModelGroup(planning_group_);
      if (!jmg) {
        finish(false, moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                "JointModelGroup not found for planning_group: " + planning_group_);
        return;
      }
    
      const auto& active_names = jmg->getActiveJointModelNames();
    
      std::vector<double> curr_positions;
      current_state->copyJointGroupPositions(jmg, curr_positions);
    
      if (active_names.size() != curr_positions.size()) {
        finish(false, moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                "Active joint names and positions size mismatch.");
        return;
      }
    
      for (size_t i = 0; i < active_names.size(); ++i) {
        if (joint_map.find(active_names[i]) == joint_map.end()) {
          joint_map[active_names[i]] = curr_positions[i];
        }
      }
    move_group_->setJointValueTarget(joint_map);
      

      if (cancel_if_needed(move_group_.get())) return;

      publish_fb("planning", 0.45f);

      auto plan_res = move_group_->plan(plan);
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

    if (cancel_if_needed(move_group_.get())) return;

    // PLAN only
    if (goal->command == TmsRpExcavator::Goal::CMD_PLAN_TO_POSE ||
        goal->command == TmsRpExcavator::Goal::CMD_PLAN_TO_JOINTS)
    {
      finish(true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, "Planned (cached as last plan).");
      return;
    }

    // PLAN + EXECUTE
    publish_fb("executing", 0.75f);

    if (cancel_if_needed(move_group_.get())) return;

    auto exec_res = move_group_->execute(plan);
    if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
      finish(true, moveit_msgs::msg::MoveItErrorCodes::SUCCESS, "Executed.");
    } else {
      finish(false, exec_res.val, "Execute failed.");
    }
  }

  // -------- Service Handlers --------

  void handle_param_get(
    const std::shared_ptr<TmsRpExcavatorParamGet::Request> request,
    std::shared_ptr<TmsRpExcavatorParamGet::Response> response)
  {
    RCLCPP_INFO(get_logger(), "TmsRpExcavatorParamGet service called");

    try {
      if (request->get_joint_limits) {
        // ロボットモデルを取得
        const moveit::core::RobotModelConstPtr& robot_model = move_group_->getRobotModel();
        const moveit::core::JointModelGroup* joint_model_group = 
            robot_model->getJointModelGroup(planning_group_);

        if (!joint_model_group) {
          response->success = false;
          response->message = "Failed to get joint model group: " + planning_group_;
          RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
          return;
        }

        // 関節名のリストを取得
        const std::vector<std::string>& joint_names = joint_model_group->getActiveJointModelNames();
        response->joint_names = joint_names;

        // 各関節の制限を取得
        for (const auto& joint_name : joint_names) {
          const moveit::core::JointModel* joint_model = robot_model->getJointModel(joint_name);
          if (!joint_model) continue;

          const moveit::core::JointModel::Bounds& bounds = joint_model->getVariableBounds();
          
          // 最初の変数の制限を取得（ほとんどの関節は1自由度）
          if (!bounds.empty()) {
            response->min_positions.push_back(bounds[0].min_position_);
            response->max_positions.push_back(bounds[0].max_position_);
            response->max_velocities.push_back(bounds[0].max_velocity_);
            response->max_accelerations.push_back(bounds[0].max_acceleration_);
          }
        }

        RCLCPP_INFO(get_logger(), "Retrieved joint limits for %zu joints", joint_names.size());
      }
      
      if (request->get_current_state) {
        auto state = move_group_->getCurrentState();
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.name = move_group_->getJointNames();
        state->copyJointGroupPositions(
            state->getJointModelGroup(planning_group_),
            js.position);
          response->joint_states = js;

        // 現在のエンドエフェクタ位置を取得
        geometry_msgs::msg::PoseStamped current_pose_stamped = move_group_->getCurrentPose();
        response->current_ee_pose = current_pose_stamped.pose;

        RCLCPP_INFO(get_logger(), "Retrieved current state: pose=[%.3f, %.3f, %.3f], %zu joints",
                    response->current_ee_pose.position.x,
                    response->current_ee_pose.position.y,
                    response->current_ee_pose.position.z,
                    response->joint_states.name.size());
      }
      
      if (request->get_configuration) {
        // 現在の設定値を取得（Getterがある値と保存した値）
        response->goal_position_tolerance = move_group_->getGoalPositionTolerance();
        response->goal_orientation_tolerance = move_group_->getGoalOrientationTolerance();
        response->goal_joint_tolerance = move_group_->getGoalJointTolerance();
        response->max_velocity_scaling_factor = current_max_velocity_scaling_factor_;
        response->max_acceleration_scaling_factor = current_max_acceleration_scaling_factor_;
        response->planning_time = move_group_->getPlanningTime();
        response->num_planning_attempts = current_num_planning_attempts_;
        response->planner_id = current_planner_id_;
        response->planning_pipeline_id = current_planning_pipeline_id_;

        RCLCPP_INFO(get_logger(), "Retrieved configuration: vel_scale=%.2f, acc_scale=%.2f, planner=%s, pipeline=%s",
                    response->max_velocity_scaling_factor,
                    response->max_acceleration_scaling_factor,
                    response->planner_id.c_str(),
                    response->planning_pipeline_id.c_str());
      }

      response->success = true;
      response->message = "Successfully retrieved MoveGroup info";
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());

    } catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Exception: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    }
  }

  void handle_param_set(
    const std::shared_ptr<TmsRpExcavatorParamSet::Request> request,
    std::shared_ptr<TmsRpExcavatorParamSet::Response> response)
  {
    RCLCPP_INFO(get_logger(), "TmsRpExcavatorParamSet service called");

    try {
      // 各パラメータを設定（負の値やNaNは無視）
      if (request->goal_position_tolerance >= 0.0 && !std::isnan(request->goal_position_tolerance)) {
        move_group_->setGoalPositionTolerance(request->goal_position_tolerance);
        RCLCPP_INFO(get_logger(), "Set goal position tolerance: %.4f", request->goal_position_tolerance);
      }

      if (request->goal_orientation_tolerance >= 0.0 && !std::isnan(request->goal_orientation_tolerance)) {
        move_group_->setGoalOrientationTolerance(request->goal_orientation_tolerance);
        RCLCPP_INFO(get_logger(), "Set goal orientation tolerance: %.4f", request->goal_orientation_tolerance);
      }

      if (request->goal_joint_tolerance >= 0.0 && !std::isnan(request->goal_joint_tolerance)) {
        move_group_->setGoalJointTolerance(request->goal_joint_tolerance);
        RCLCPP_INFO(get_logger(), "Set goal joint tolerance: %.4f", request->goal_joint_tolerance);
      }

      if (request->max_velocity_scaling_factor >= 0.0 && request->max_velocity_scaling_factor <= 1.0 &&
          !std::isnan(request->max_velocity_scaling_factor)) {
        move_group_->setMaxVelocityScalingFactor(request->max_velocity_scaling_factor);
        current_max_velocity_scaling_factor_ = request->max_velocity_scaling_factor;
        RCLCPP_INFO(get_logger(), "Set max velocity scaling factor: %.4f", request->max_velocity_scaling_factor);
      }

      if (request->max_acceleration_scaling_factor >= 0.0 && request->max_acceleration_scaling_factor <= 1.0 &&
          !std::isnan(request->max_acceleration_scaling_factor)) {
        move_group_->setMaxAccelerationScalingFactor(request->max_acceleration_scaling_factor);
        current_max_acceleration_scaling_factor_ = request->max_acceleration_scaling_factor;
        RCLCPP_INFO(get_logger(), "Set max acceleration scaling factor: %.4f", request->max_acceleration_scaling_factor);
      }

      if (request->planning_time > 0.0 && !std::isnan(request->planning_time)) {
        move_group_->setPlanningTime(request->planning_time);
        RCLCPP_INFO(get_logger(), "Set planning time: %.2f sec", request->planning_time);
      }

      if (request->num_planning_attempts > 0) {
        move_group_->setNumPlanningAttempts(request->num_planning_attempts);
        current_num_planning_attempts_ = request->num_planning_attempts;
        RCLCPP_INFO(get_logger(), "Set num planning attempts: %d", request->num_planning_attempts);
      }

      move_group_->allowReplanning(request->allow_replanning);
      RCLCPP_INFO(get_logger(), "Set allow replanning: %s", request->allow_replanning ? "true" : "false");

      // planning_pipeline_idが指定されている場合は設定
      if (!request->planning_pipeline_id.empty()) {
        move_group_->setPlanningPipelineId(request->planning_pipeline_id);
        current_planning_pipeline_id_ = request->planning_pipeline_id;
        RCLCPP_INFO(get_logger(), "Set planning pipeline ID: %s", request->planning_pipeline_id.c_str());
      }

      // planner_idが指定されている場合は設定
      if (!request->planner_id.empty()) {
        move_group_->setPlannerId(request->planner_id);
        current_planner_id_ = request->planner_id;
        RCLCPP_INFO(get_logger(), "Set planner ID: %s", request->planner_id.c_str());
      }

      // 現在の設定値をレスポンスに返す
      response->goal_position_tolerance = move_group_->getGoalPositionTolerance();
      response->goal_orientation_tolerance = move_group_->getGoalOrientationTolerance();
      response->goal_joint_tolerance = move_group_->getGoalJointTolerance();
      response->max_velocity_scaling_factor = current_max_velocity_scaling_factor_;
      response->max_acceleration_scaling_factor = current_max_acceleration_scaling_factor_;
      response->planning_time = move_group_->getPlanningTime();
      response->num_planning_attempts = current_num_planning_attempts_;
      response->allow_replanning = request->allow_replanning;
      response->planner_id = current_planner_id_;
      response->planning_pipeline_id = current_planning_pipeline_id_;

      response->success = true;
      response->message = "Successfully configured MoveGroup";
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());

    } catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Exception: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    }
  }

  static bool setStartStateFromPrevRobotTrajectory(
    const moveit_msgs::msg::RobotTrajectory& prev_traj,
    moveit::planning_interface::MoveGroupInterface& mg,
    const std::string& planning_group,
    const rclcpp::Logger& logger)
  {
    const auto& jt = prev_traj.joint_trajectory;
  
    if (jt.joint_names.empty() || jt.points.empty()) {
      RCLCPP_WARN(logger, "prev RobotTrajectory is empty; fallback to current state.");
      mg.setStartStateToCurrentState();
      return false;
    }
  
    const auto& last_pt = jt.points.back();
    if (last_pt.positions.size() != jt.joint_names.size()) {
      RCLCPP_ERROR(logger,
        "prev_traj last point mismatch: positions=%zu joint_names=%zu",
        last_pt.positions.size(), jt.joint_names.size());
      return false;
    }
  
    const auto robot_model = mg.getRobotModel();
    const auto* jmg = robot_model->getJointModelGroup(planning_group);
    if (!jmg) {
      RCLCPP_ERROR(logger, "JointModelGroup not found: %s", planning_group.c_str());
      return false;
    }
  
    // joint_name -> position の辞書を作る
    std::unordered_map<std::string, double> name_to_pos;
    name_to_pos.reserve(jt.joint_names.size());
    for (size_t i = 0; i < jt.joint_names.size(); ++i) {
      name_to_pos[jt.joint_names[i]] = last_pt.positions[i];
    }
  
    // planning_group の active joints だけ取り出してセット
    const auto& active = jmg->getActiveJointModelNames();
    std::vector<double> group_positions;
    group_positions.reserve(active.size());
  
    for (const auto& jn : active) {
      auto it = name_to_pos.find(jn);
      if (it == name_to_pos.end()) {
        RCLCPP_ERROR(logger,
          "prev_traj does not contain required joint '%s' for group '%s'",
          jn.c_str(), planning_group.c_str());
        return false;
      }
      group_positions.push_back(it->second);
    }
  
    moveit::core::RobotState start_state(*mg.getCurrentState());
    start_state.setJointGroupPositions(jmg, group_positions);
    start_state.update();
  
    mg.setStartState(start_state);
    RCLCPP_INFO(logger, "Start state set from prev RobotTrajectory last point.");
    return true;
  }

  static const moveit_msgs::msg::RobotTrajectory* pickPrevTrajectory(
    const std::vector<moveit_msgs::msg::RobotTrajectory>& prev_vec)
  {
    if (prev_vec.empty()) return nullptr;
    const auto& t = prev_vec.back();  // 「最後のtrajectory」を採用
    if (t.joint_trajectory.joint_names.empty()) return nullptr;
    if (t.joint_trajectory.points.empty()) return nullptr;
    return &t;
  }  

  static bool fillRobotStateFromPrevTrajectoryLastPoint(
    const moveit_msgs::msg::RobotTrajectory& prev_traj,
    moveit_msgs::msg::RobotState& out_state,
    const rclcpp::Logger& logger)
  {
    const auto& jt = prev_traj.joint_trajectory;
    if (jt.joint_names.empty() || jt.points.empty()) {
      RCLCPP_WARN(logger, "prev_traj empty; cannot make RobotState.");
      return false;
    }
    const auto& last = jt.points.back();
    if (last.positions.size() != jt.joint_names.size()) {
      RCLCPP_ERROR(logger, "size mismatch: names=%zu pos=%zu",
                   jt.joint_names.size(), last.positions.size());
      return false;
    }
  
    out_state.joint_state.name = jt.joint_names;
    out_state.joint_state.position = last.positions;
    out_state.is_diff = true;
    return true;
  }  
  
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TmsIfMoveItActionServer>());
  rclcpp::shutdown();
  return 0;
}
