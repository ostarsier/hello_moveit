
- 1.urdf
ros2 param get /robot_state_publisher robot_description
- 2.srdf
ros2 param get /move_group robot_description_semantic
- 3.运动学解析器
ros2 param get /move_group robot_description_kinematics.manipulator.kinematics_solver


