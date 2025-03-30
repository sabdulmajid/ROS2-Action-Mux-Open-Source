#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("int_publisher");
    auto publisher = node->create_publisher<std_msgs::msg::Int32>("/int_topic", 10);
    rclcpp::Rate rate(1);
    int count = 0;
    while (rclcpp::ok())
    {
        auto msg = std_msgs::msg::Int32();
        msg.data = count++;
        RCLCPP_INFO(node->get_logger(), "Publishing: %d", msg.data);
        publisher->publish(msg);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}