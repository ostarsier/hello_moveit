#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2_ik_demo");

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit2_ik_demo_node");

  RCLCPP_INFO(LOGGER, "Loading robot model...");
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  robot_model_loader.loadKinematicsSolvers(); // 显式加载运动学求解器
  
  moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();
  if (!robot_model)
  {
    RCLCPP_ERROR(LOGGER, "Failed to load robot model. Make sure 'robot_description' is on the parameter server.");
    return 1;
  }
  RCLCPP_INFO(LOGGER, "Robot model loaded: %s", robot_model->getModelFrame().c_str());
  
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(robot_model));
  kinematic_state->setToDefaultValues(); // 设置初始关节状态

  // 获取规划组
  const std::string PLANNING_GROUP = "panda_arm";
  const moveit::core::JointModelGroup* joint_model_group =
      robot_model->getJointModelGroup(PLANNING_GROUP);

  if (!joint_model_group)
  {
    RCLCPP_ERROR(LOGGER, "Failed to get joint model group '%s'. Please check your SRDF configuration.", PLANNING_GROUP.c_str());
    return 1;
  }

  RCLCPP_INFO(LOGGER, "Planning group '%s' loaded.", PLANNING_GROUP.c_str());

  // 设置末端执行器的目标位姿
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.5;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.8;
  target_pose.orientation.x = 0.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 1.0;

  RCLCPP_INFO(LOGGER, "Attempting to solve IK for target pose:");
  RCLCPP_INFO(LOGGER, "  Position: x=%f, y=%f, z=%f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
  RCLCPP_INFO(LOGGER, "  Orientation: x=%f, y=%f, z=%f, w=%f", target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

  // 获取末端执行器的链接名称 (例如: "tool0" 或 "end_effector_link")
  // 确保这个链接是你的规划组中的末端执行器
  const std::string& end_effector_link = joint_model_group->getLinkModelNames().back(); // 假设最后一个链接是末端执行器
  RCLCPP_INFO(LOGGER, "End effector link: %s", end_effector_link.c_str());

  // IK 参数设置
  double timeout = 0.1; // 超时时间（秒）
  int attempts = 10;    // 尝试次数

  std::vector<double> joint_values;
  
  // 将geometry_msgs::msg::Pose转换为Eigen::Isometry3d
  Eigen::Isometry3d target_pose_eigen;
  tf2::fromMsg(target_pose, target_pose_eigen);
  
  // 调用 setFromIK 进行逆运动学计算
  bool found_ik = kinematic_state->setFromIK(joint_model_group, target_pose_eigen, end_effector_link, timeout);

  if (found_ik) {
    RCLCPP_INFO(LOGGER, "IK solution found!");
    // 获取计算出的关节值
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    
    // 打印关节角度
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    RCLCPP_INFO(LOGGER, "Joint angles for target pose:");
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      RCLCPP_INFO(LOGGER, "  %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    
    // 验证IK解
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector_link);
    geometry_msgs::msg::Pose current_pose;
    current_pose = tf2::toMsg(end_effector_state);
    
    RCLCPP_INFO(LOGGER, "Verification - End-effector pose from IK solution:");
    RCLCPP_INFO(LOGGER, "  Position: x=%f, y=%f, z=%f", 
              current_pose.position.x, current_pose.position.y, current_pose.position.z);
    RCLCPP_INFO(LOGGER, "  Orientation: x=%f, y=%f, z=%f, w=%f", 
              current_pose.orientation.x, current_pose.orientation.y, 
              current_pose.orientation.z, current_pose.orientation.w);
  } else {
    RCLCPP_WARN(LOGGER, "Could not find an IK solution for the target pose.");
  }

rclcpp::shutdown();
return 0;
}