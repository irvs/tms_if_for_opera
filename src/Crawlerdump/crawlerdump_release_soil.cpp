// ============================ crawlerdump_release_soil.cpp ============================
#include <thread>
#include <utility>

#include "tms_if_for_opera/Crawlerdump/crawlerdump_release_soil.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

CrawlerdumpReleaseSoil::CrawlerdumpReleaseSoil()
: rclcpp::Node("tms_if_crawlerdump_release_soil_node")
{
  // server: 受け口（外からこのノードへ）
  action_server_ = rclcpp_action::create_server<Action>(
    this,
    "tms_rp_set_dump_angle",
    std::bind(&CrawlerdumpReleaseSoil::handle_goal, this, _1, _2),
    std::bind(&CrawlerdumpReleaseSoil::handle_cancel, this, _1),
    std::bind(&CrawlerdumpReleaseSoil::handle_accepted, this, _1));

  // client: 中継先（このノードから別アクションへ）
  action_client_ = rclcpp_action::create_client<Action>(this, "set_dump_angle");
}

rclcpp_action::GoalResponse CrawlerdumpReleaseSoil::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Action::Goal> /*goal*/)
{
  RCLCPP_INFO(get_logger(), "Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CrawlerdumpReleaseSoil::handle_cancel(
  const std::shared_ptr<ServerGoalHandle> /*server_goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel (server side)");

  // client 側に投げたゴールが既に作られているならキャンセルを中継
  try {
    if (client_future_goal_handle_.valid() &&
        client_future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto client_goal_handle = client_future_goal_handle_.get();
      if (client_goal_handle) {
        action_client_->async_cancel_goal(client_goal_handle);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception in cancel relay: %s", e.what());
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void CrawlerdumpReleaseSoil::handle_accepted(const std::shared_ptr<ServerGoalHandle> goal_handle)
{
  std::thread{std::bind(&CrawlerdumpReleaseSoil::execute, this, _1), goal_handle}.detach();
}

void CrawlerdumpReleaseSoil::execute(const std::shared_ptr<ServerGoalHandle> server_goal_handle)
{
  RCLCPP_INFO(get_logger(), "tms_if_for_opera(crawlerdump_release_soil) executing...");
  current_goal_handle_ = server_goal_handle;

  auto result_to_server = std::make_shared<Action::Result>();

  // ---- 中継先の Action server がいないときは abort ----
  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_logger(), "Downstream action server 'set_dump_angle' not available");
    if (server_goal_handle->is_active()) {
      server_goal_handle->abort(result_to_server);
    }
    return;
  }

  // ---- server 側で受けた Goal を client 側へ中継 ----
  Action::Goal goal_msg{};
  auto received_goal = server_goal_handle->get_goal();
  goal_msg = *received_goal;  // ★ここが「丸ごとコピー」の正解

  typename Client::SendGoalOptions send_goal_options;
  send_goal_options.goal_response_callback =
    [this](const typename ClientGoalHandle::SharedPtr & gh) {
      goal_response_callback(gh);
    };

  send_goal_options.feedback_callback =
    [this](typename ClientGoalHandle::SharedPtr gh,
           const std::shared_ptr<const Action::Feedback> feedback) {
      feedback_callback(gh, feedback);
    };

  send_goal_options.result_callback =
    [this, server_goal_handle](const WrappedResult & wrapped) {
      result_callback(server_goal_handle, wrapped);
    };

  RCLCPP_INFO(get_logger(), "Relaying goal to downstream action server...");
  client_future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
}

void CrawlerdumpReleaseSoil::goal_response_callback(const typename ClientGoalHandle::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Downstream goal was rejected");
  } else {
    RCLCPP_INFO(get_logger(), "Downstream goal accepted, waiting for result");
  }
}

void CrawlerdumpReleaseSoil::feedback_callback(
  typename ClientGoalHandle::SharedPtr /*gh*/,
  const std::shared_ptr<const Action::Feedback> /*feedback*/)
{
  // 必要なら server_goal_handle->publish_feedback(...) をここでやる
}

void CrawlerdumpReleaseSoil::result_callback(
  const std::shared_ptr<ServerGoalHandle> server_goal_handle,
  const WrappedResult & result)
{
  if (!server_goal_handle->is_active()) {
    RCLCPP_WARN(get_logger(), "Server goal is not active anymore");
    return;
  }

  auto result_to_server = std::make_shared<Action::Result>();

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      server_goal_handle->succeed(result_to_server);
      RCLCPP_INFO(get_logger(), "release_soil succeeded");
      break;

    case rclcpp_action::ResultCode::ABORTED:
      server_goal_handle->abort(result_to_server);
      RCLCPP_INFO(get_logger(), "release_soil aborted");
      break;

    case rclcpp_action::ResultCode::CANCELED:
      server_goal_handle->canceled(result_to_server);
      RCLCPP_INFO(get_logger(), "release_soil canceled");
      break;

    default:
      server_goal_handle->abort(result_to_server);
      RCLCPP_INFO(get_logger(), "Unknown result code");
      break;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrawlerdumpReleaseSoil>());
  rclcpp::shutdown();
  return 0;
}
