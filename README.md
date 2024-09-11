
We introduced Large Language Model（LLM） to realize the human-robot interaction system with autonomous underwater vehicles (AUV)

# Contribution
Our work is to establish a ROS2-based AUV interactive system. By leveraging LLM's understanding of human instructions, we provide high-level decisions to the low-level controllers, thereby reducing the complexity of sea trials.


# Prerequisites
* Install [ROS2 Humble](https://docs.ros.org/en/humble/)
* Ubuntu 22.04
* Install [Ignition Gazebo](https://gazebosim.org/docs/garden/ros_installation/)

# LLM in ROS2
The combination of LLM and ROS2 aims to utilize the LLM's ability to understand natural language commands in conjunction with ROS2's robotics framework to provide an efficient and flexible control system.
## How to use
* colcon packages
```python
colcon build --packages-select bluerov_llm
```
* Loading Environment Variables for a Specific Workspace or ROS 2 Installation
```python
source install/setup.bash
source /opt/ros/humble/setup.bash
```
* case1: Navigation instruction
```python
ros2 run bluerov_llm chat "Change the MPC weight matrix to [20,20,20,0.1,0.1,0.1]."
ros2 run bluerov_llm chat "Based on the previous instruction, Increase the NMPC weight matrix, first state to 30."
```
