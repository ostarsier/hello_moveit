#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>

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
    0,
    0,
    0,
    0,
    0,
    0
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
  RCLCPP_INFO(logger, "目标姿态: x=%.3f, y=%.3f, z=%.3f, w=%.3f", 
             target_pose.orientation.x, target_pose.orientation.y, 
             target_pose.orientation.z, target_pose.orientation.w);

  move_group_interface.setPoseTarget(target_pose);

// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}