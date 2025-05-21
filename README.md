

# create ros  package
ros2 pkg create --build-type ament_cmake x1_moveit_urdf  
CMakeLists.txt 添加
# 安装URDF和meshes文件到share目录
install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)


# setup
ros2 launch moveit_setup_assistant setup_assistant.launch.py

# source环境
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash
source ~/ws_moveit_my/install/setup.bash
conda activate ros2

# 编译代码
colcon build --packages-select x1_moveit_urdf
colcon build --packages-select x1_moveit_config
colcon build --packages-select x1_moveit_service

source ~/ws_moveit_my/install/setup.bash

# 运行moveit
ros2 launch moveit2_tutorials demo.launch.py

ros2 launch fanuc_moveit_config demo.launch.py
ros2 launch x1_moveit_config demo.launch.py

ros2 launch x1_moveit_config demo.launch.py

ros2 run x1_moveit_service x1_moveit_service


# ros2命令
ros2 topic echo /joint_states
ros2 topic echo /tf


ros2 pkg create --build-type ament_cmake x1_moveit_service 


ros2 pkg create \
 --build-type ament_cmake \
 --dependencies moveit_ros_planning_interface rclcpp \
 --node-name x1_moveit_service x1_moveit_service




