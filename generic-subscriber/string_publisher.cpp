#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("string_publisher");
    auto publisher = node->create_publisher<std_msgs::msg::String>("/string_topic", 10);
    rclcpp::Rate rate(1);
    int count = 0;
    while (rclcpp::ok())
    {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello " + std::to_string(count++);
        RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", msg.data.c_str());
        publisher->publish(msg);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}