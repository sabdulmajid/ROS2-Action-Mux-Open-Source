#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
#include <custom_interfaces/action/wait.hpp>

using Wait = custom_interfaces::action::Wait;
using GoalHandleWait = rclcpp_action::ServerGoalHandle<Wait>;

class ActionServer : public rclcpp::Node
{
public:
  ActionServer() : Node("action_server")
  {
    action_server_ = rclcpp_action::create_server<Wait>(
        this, "wait_action",
        std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));

    cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
        "cancel_current_goal", 10,
        std::bind(&ActionServer::cancel_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Action server started.");
  }

private:
  rclcpp_action::Server<Wait>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  std::shared_ptr<GoalHandleWait> current_goal_handle_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const Wait::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(mutex_); // ensure thread-safe access to current_goal_handle_
    if (current_goal_handle_ && current_goal_handle_->is_active())
    {
      RCLCPP_INFO(this->get_logger(), "Goal rejected: another goal is active.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Goal accepted with ID: %d", goal->goal_id);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleWait> goal_handle)
  {
    // handle cancellation request for a specific goal
    RCLCPP_INFO(this->get_logger(), "Goal cancellation requested for ID: %d",
                goal_handle->get_goal()->goal_id);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWait> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_); // ensure thread-safe assignment of current_goal_handle_
      current_goal_handle_ = goal_handle;
    }
    // detach a thread to execute the goal asynchronously
    std::thread{std::bind(&ActionServer::execute_goal, this, std::placeholders::_1), goal_handle}
        .detach();
  }

  void execute_goal(const std::shared_ptr<GoalHandleWait> goal_handle)
  {
    auto feedback = std::make_shared<Wait::Feedback>();
    auto result = std::make_shared<Wait::Result>();
    feedback->status = "waiting";
    int goal_id = goal_handle->get_goal()->goal_id;

    rclcpp::Rate rate(1); // 1 Hz feedback
    for (int i = 0; i < 5; ++i)
    {
      if (!rclcpp::ok() || goal_handle->is_canceling())
      {
        result->goal_id = goal_id;
        goal_handle->canceled(result); // notify that the goal was canceled
        RCLCPP_INFO(this->get_logger(), "Goal ID %d canceled.", goal_id);
        {
          std::lock_guard<std::mutex> lock(mutex_); // reset current_goal_handle_ safely
          current_goal_handle_ = nullptr;
        }
        return;
      }
      goal_handle->publish_feedback(feedback); // send periodic feedback
      rate.sleep();
    }

    result->goal_id = goal_id;
    goal_handle->succeed(result); // notify that the goal was successfully completed
    RCLCPP_INFO(this->get_logger(), "Goal ID %d succeeded.", goal_id);
    {
      std::lock_guard<std::mutex> lock(mutex_); // reset current_goal_handle_ safely
      current_goal_handle_ = nullptr;
    }
  }

  void cancel_callback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_); // ensure thread-safe access to current_goal_handle_
    if (current_goal_handle_ && current_goal_handle_->is_active())
    {
      current_goal_handle_->abort(std::make_shared<Wait::Result>()); // abort the current goal
      RCLCPP_INFO(this->get_logger(), "Current goal aborted due to cancel request.");
    }
  }

  std::mutex mutex_; // protects access to current_goal_handle_
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActionServer>());
  rclcpp::shutdown();
  return 0;
}