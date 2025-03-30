#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/empty.hpp"
#include <custom_interfaces/action/wait.hpp>

class HighPriorityClient : public rclcpp::Node
{
public:
  using Wait = custom_interfaces::action::Wait;
  HighPriorityClient() : Node("high_priority_client")
  {
    client_ = rclcpp_action::create_client<Wait>(this, "wait_action");
    cancel_pub_ = create_publisher<std_msgs::msg::Empty>("cancel_current_goal", 10);
    goal_sub_ = create_subscription<std_msgs::msg::Int32>(
        "high_priority_goal", 10,
        std::bind(&HighPriorityClient::goal_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "High-priority client started.");
  }

private:
  rclcpp_action::Client<Wait>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goal_sub_;

  void goal_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    cancel_pub_->publish(std_msgs::msg::Empty()); // publish a message to cancel the current goal
    rclcpp::sleep_for(std::chrono::milliseconds(100)); // ensure the cancellation is processed before sending a new goal
    send_goal(msg->data); // send the new high-priority goal
  }

  void send_goal(int goal_id)
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(10)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available.");
      return;
    }

    auto goal_msg = Wait::Goal();
    goal_msg.goal_id = goal_id; // set the goal ID for the action

    auto send_goal_options = rclcpp_action::Client<Wait>::SendGoalOptions();
    send_goal_options.result_callback =
        [this, goal_id](const rclcpp_action::ClientGoalHandle<Wait>::WrappedResult &result)
    {
      // handle the result of the goal execution
      switch (result.code)
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "High-priority goal %d succeeded.", goal_id);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(this->get_logger(), "High-priority goal %d aborted.", goal_id);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "High-priority goal %d canceled.", goal_id);
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "High-priority goal %d unknown result.", goal_id);
      }
    };

    RCLCPP_INFO(this->get_logger(), "Sending high-priority goal with ID: %d", goal_id);
    client_->async_send_goal(goal_msg, send_goal_options); // send the goal asynchronously
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HighPriorityClient>()); // keep the node running to process callbacks
  rclcpp::shutdown();
  return 0;
}