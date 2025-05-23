#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>

// 规划并执行到: 右手关节状态全部设置为0时的position

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "x1_moveit_service",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("x1_moveit_service");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  // 加载机器人模型
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();

  // 创建机器人状态
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));

  // 设置关节状态
  std::vector<std::string> joint_names = {
    "right_shoulder_pitch_joint",
    "right_shoulder_yaw_joint",
    "right_shoulder_roll_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
    "right_wrist_pitch_joint"
  };

  std::vector<double> joint_positions = {
     1.407659379984697,
     -2.20502064988792e-05,
     7.674062572431711e-05,
     2.0907559106733458,
     -2.4927829739344802e-05,
     0.6830936329077301
  };

  robot_state->setJointGroupPositions("manipulator", joint_positions);

  // 通过正运动学计算末端执行器位置
  const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform("right_wrist_pitch");

  // 转换为geometry_msgs::msg::Pose
  geometry_msgs::msg::Pose target_pose;
  Eigen::Quaterniond quat(end_effector_state.rotation());
  target_pose.position.x = end_effector_state.translation().x(); 
  target_pose.position.y = end_effector_state.translation().y(); 
  target_pose.position.z = end_effector_state.translation().z(); 
  target_pose.orientation.x = quat.x();
  target_pose.orientation.y = quat.y();
  target_pose.orientation.z = quat.z();
  target_pose.orientation.w = quat.w();

  RCLCPP_INFO(logger, "目标位置: x=%.3f, y=%.3f, z=%.3f", 
             target_pose.position.x, target_pose.position.y, target_pose.position.z);
  Eigen::Vector3d rpy = end_effector_state.rotation().eulerAngles(0, 1, 2); // 0: roll, 1: pitch, 2: yaw
  RCLCPP_INFO(logger, "目标姿态(欧拉角): roll=%.3f, pitch=%.3f, yaw=%.3f", rpy[0], rpy[1], rpy[2]);

  RCLCPP_INFO(logger, "目标姿态(四元数): x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
    target_pose.orientation.x, target_pose.orientation.y, 
    target_pose.orientation.z, target_pose.orientation.w);

  // 设置速度优化参数
  move_group_interface.setMaxVelocityScalingFactor(1.0);  // 最大速度
  move_group_interface.setMaxAccelerationScalingFactor(1.0);  // 最大加速度
  move_group_interface.setPlanningTime(0.5);  // 减少规划时间
  move_group_interface.setPlannerId("RRTConnect");  // 使用更快的规划器
  move_group_interface.setNumPlanningAttempts(1);  // 减少规划尝试次数

  move_group_interface.setPoseTarget(target_pose);

// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  // plan 不是单一关节的一个值，而是包含了多个关节在一段时间内的完整轨迹。
  // 内部包含了trajectory, 里有一个 points 数组，每个 point 记录了所有相关关节在某一时刻的角度（位置）、速度、加速度等信息
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}