# ROS 2 Action Server

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)

The **ROS 2 Action Server** is a key component of the ROS 2 Action Mux project. It provides a robust action server capable of handling goals with a specified wait time, supporting preemption for higher-priority goals, and managing goal abortions seamlessly.

---

## Features

- **Goal Handling**: Processes goals with unique IDs efficiently.
- **Preemption**: Interrupts lower-priority goals when a higher-priority goal is received.
- **Feedback Mechanism**: Delivers real-time feedback during goal execution.
- **Abortion Handling**: Manages goal cancellations gracefully within the defined wait time.

---

## Installation

To set up the Action Server, follow these steps:

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
4. **Launch the Action Server**
   ```bash
   ros2 run ros2_action_mux action_server
   ```

---

## Usage
**Interacting with the Action Server**
- Send a Goal: Use an action client to submit a goal with a specific ID.
- Preempt Goals: Publish to the cancel_current_goal topic to stop the current goal.
- Monitor Feedback: Check real-time updates during goal processing.

**Example**: send a goal with ID 1
   ```
   ros2 action send_goal /wait_action custom_interfaces/action/Wait "{goal_id: 1}"
   ```