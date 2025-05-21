

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

# 编译
colcon build 

# 运行moveit
ros2 launch x1_moveit_config demo.launch.py

ros2 run x1_moveit_service x1_moveit_service


# ros2命令
ros2 topic echo /joint_states

# 手默认的状态，关节都是0时的position
"right_shoulder_pitch_joint",
"right_shoulder_yaw_joint",
"right_shoulder_roll_joint",
"right_elbow_pitch_joint",
"right_elbow_yaw_joint",
"right_wrist_pitch_joint"

目标位置: x=0.002, y=-0.199, z=-0.006
目标姿态: x=0.500, y=0.500, z=0.500, w=0.500


