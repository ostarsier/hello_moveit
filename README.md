https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html



# source环境
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash
conda activate ros2

# 编译代码
colcon build
source ~/ws_moveit_my/install/setup.bash

# 运行moveit
ros2 launch moveit2_tutorials demo.launch.py
ros2 run hello_moveit hello_moveit
vla_moveit_position -> moveit逆运动学 -> vla_moveit_joint




