# ROS 2 Action Mux

This project implements a basic ROS 2 Action Mux system and a generic subscriber.

## Prerequisites
- ROS 2 (Humble or later)
- C++17 compatible compiler
- Colcon build tool

## Build Instructions
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repository_url>
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash