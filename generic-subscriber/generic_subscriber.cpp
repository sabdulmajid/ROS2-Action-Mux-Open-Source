#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

class GenericSubscriber : public rclcpp::Node
{
public:
    GenericSubscriber() : Node("generic_subscriber")
    {
        declare_parameter("topic_name", "/test_topic"); // declare a parameter for the topic name
        std::string topic_name = get_parameter("topic_name").as_string();

        // Get topic type
        bool type_found = false;
        while (!type_found && rclcpp::ok())
        {
            auto topic_names_and_types = get_topic_names_and_types(); // retrieve all topic names and their types
            for (const auto &topic : topic_names_and_types)
            {
                if (topic.first == topic_name && !topic.second.empty())
                {
                    topic_type_ = topic.second[0]; // assign the first type if the topic is found
                    type_found = true;
                    break;
                }
            }
            if (!type_found)
            {
                RCLCPP_WARN(this->get_logger(), "Topic %s not found, waiting...", topic_name.c_str());
                rclcpp::sleep_for(std::chrono::seconds(1)); // wait before retrying if the topic is not found
            }
        }

        subscriber_ = create_generic_subscription(
            topic_name, "serialized", 10,
            std::bind(&GenericSubscriber::callback, this, std::placeholders::_1)); // create a generic subscription with a serialized message type

        RCLCPP_INFO(this->get_logger(), "Subscribed to %s with type %s",
                    topic_name.c_str(), topic_type_.c_str());
    }

private:
    rclcpp::GenericSubscription::SharedPtr subscriber_;
    std::string topic_type_;

    void callback(const std::shared_ptr<rclcpp::SerializedMessage> msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received message on topic %s with type %s",
                    subscriber_->get_topic_name(), topic_type_.c_str()); // log the received message details
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GenericSubscriber>()); // keep the node running and processing callbacks
    rclcpp::shutdown();
    return 0;
}