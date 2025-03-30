# ROS 2 Generic Subscriber

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)  

The **ROS 2 Generic Subscriber** is a flexible component within the ROS 2 Action Mux project. It dynamically subscribes to any ROS 2 topic, detects the message type, and prints it upon receiving a message—perfect for debugging or monitoring.

---

## Features

- **Dynamic Subscription**: Connects to any specified ROS 2 topic.
- **Type Detection**: Identifies and displays the message type automatically.
- **Flexible Configuration**: Adjustable via parameters for ease of use.

---

## Installation

To install the Generic Subscriber, follow these steps:

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/ros2_action_mux.git
   cd ros2_action_mux
   ```
2. **Build the Package:**
   ```bash
   colcon build --packages-select ros2_action_mux
   ```
3. **Source the Workspace:**
   ```bash
   source install/setup.bash
   ```
4. **Run the Generic Subscriber**
   ```bash
   ros2 run ros2_action_mux generic_subscriber --ros-args -p topic_name:=/your_topic
   ```

---

## Usage
**Configuring the Subscriber**
- **Topic Name:** Use the topic_name parameter to set the subscription target.
- **Message Type Detection:** The subscriber logs the message type when a message arrives.

**Example**: start publisher & subscriber
   ```bash
   ros2 run ros2_action_mux string_publisher
   ros2 run ros2_action_mux generic_subscriber --ros-args -p topic_name:=/string_topic
   ```
   **Expected output:** `[INFO] [generic_subscriber]: Received message on topic /string_topic with type std_msgs/msg/String`