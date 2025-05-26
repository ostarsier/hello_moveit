

# 项目目录
- x1_moveit_config: 右手配置
- x1_moveit_config_double: 双手配置
- x1_moveit_urdf: urdf文件
- x1_moveit_service: 逆运动学服务

# 创建urdf包
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
ros2 launch moveit2_tutorials demo.launch.py

ros2 launch x1_moveit_config demo.launch.py

ros2 run x1_moveit_service x1_moveit_service


# ros2命令
ros2 topic echo /joint_states


# 手默认的状态，关节都是0时的position
xyz: 蓝红绿
"right_shoulder_pitch_joint",
"right_shoulder_yaw_joint",
"right_shoulder_roll_joint",
"right_elbow_pitch_joint",
"right_elbow_yaw_joint",
"right_wrist_pitch_joint"

const double initial_x = 0.002;
const double initial_y = -0.199;
const double initial_z = -0.006;
const double initial_roll = 1.883;
const double initial_pitch = 1.571;
const double initial_yaw = -0.312;

# ros2 用的pip
/usr/bin/python3.10 -m pip install redis

# joycon
- x1_moveit_config demo.launch.py
- 运行rviz
ros2 launch x1_moveit_config demo.launch.py
- 发布手柄pose
ros2 run joycon pose_pub
- 运行逆运动学服务
ros2 run x1_moveit_service x1_moveit_ik

ros2 service call /ik_solve x1_moveit_proto/srv/IkSolve "{x: 0.002, y: -0.199, z: -0.006}"

# 其他
1.逆运动学多个解
多次尝试求解IK
设置了10次尝试，每次使用略微不同的随机种子状态
收集所有成功的IK解到解集合中
选择最佳解
计算每个解与初始状态的欧几里德距离
选择距离最小的解作为最佳解（即关节变化最小的解）