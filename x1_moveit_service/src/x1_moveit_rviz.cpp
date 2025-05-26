#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <x1_moveit_proto/srv/joycon_command.hpp>
// 规划到指定

// 辅助函数 - 四元数转欧拉角
Eigen::Vector3d quaternionToRPY(const Eigen::Quaterniond & quat)
{
  tf2::Quaternion tf_quat(quat.x(), quat.y(), quat.z(), quat.w());
  tf2::Matrix3x3 m(tf_quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return Eigen::Vector3d(roll, pitch, yaw);
}

// 辅助函数 - 欧拉角转四元数
Eigen::Quaterniond rpyToQuaternion(const Eigen::Vector3d & rpy)
{
  tf2::Quaternion quat;
  quat.setRPY(rpy[0], rpy[1], rpy[2]);
  return Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z());
}

// 辅助函数 - 将Joycon的pose数据转换为geometry_msgs::msg::Pose，只使用xyz位置信息
geometry_msgs::msg::Pose joyconPoseToPose(const std::array<double, 6> & pose_data)
{
  geometry_msgs::msg::Pose pose;
  
  // 设置位置 (x, y, z)
  pose.position.x = pose_data[0];
  pose.position.y = pose_data[1];
  pose.position.z = pose_data[2];
  
  // 不设置姿态，将在handleJoyconCommand函数中使用当前姿态
  // 这里设置一个默认值，后续会被覆盖
  pose.orientation.w = 1.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  
  return pose;
}

// Joycon服务回调函数
void handleJoyconCommand(
  const std::shared_ptr<x1_moveit_proto::srv::JoyconCommand::Request> request,
  std::shared_ptr<x1_moveit_proto::srv::JoyconCommand::Response> response,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
  moveit::core::RobotModelPtr robot_model,
  moveit::core::RobotStatePtr robot_state,
  const std::string & planning_group_name,
  const std::vector<std::string> & joint_names)
{
  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "接收到Joycon服务请求");
  
  // 将Joycon的pose数据转换为std::array
  std::array<double, 6> pose_data;
  for (size_t i = 0; i < 6; ++i) {
    pose_data[i] = request->pose[i];
  }
  
  // 获取当前末端执行器的姿态
  geometry_msgs::msg::PoseStamped current_pose = move_group_interface->getCurrentPose();
  
  // 转换为geometry_msgs::msg::Pose，只使用xyz位置信息
  geometry_msgs::msg::Pose target_pose = joyconPoseToPose(pose_data);
  
  // 使用当前姿态
  target_pose.orientation = current_pose.pose.orientation;
  
  RCLCPP_INFO(logger, "当前姿态: x=%.3f, y=%.3f, z=%.3f, qw=%.3f, qx=%.3f, qy=%.3f, qz=%.3f", 
              current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
              current_pose.pose.orientation.w, current_pose.pose.orientation.x, 
              current_pose.pose.orientation.y, current_pose.pose.orientation.z);
  
  RCLCPP_INFO(logger, "目标位置: x=%.3f, y=%.3f, z=%.3f", 
              target_pose.position.x, target_pose.position.y, target_pose.position.z);
  
  Eigen::Quaterniond quat(
    target_pose.orientation.w,
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z
  );
  
  Eigen::Vector3d rpy = quaternionToRPY(quat);
  RCLCPP_INFO(logger, "目标姿态(欧拉角): roll=%.3f, pitch=%.3f, yaw=%.3f", 
              rpy[0], rpy[1], rpy[2]);
  
  // 设置速度优化参数
  move_group_interface->setMaxVelocityScalingFactor(1.0);  // 最大速度
  move_group_interface->setMaxAccelerationScalingFactor(1.0);  // 最大加速度
  move_group_interface->setPlanningTime(0.5);  // 减少规划时间
  move_group_interface->setPlannerId("RRTConnect");  // 使用更快的规划器
  move_group_interface->setNumPlanningAttempts(1);  // 减少规划尝试次数
  
  // 设置目标位姿
  move_group_interface->setPoseTarget(target_pose);
  
  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface->plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan and set response
  if(success) {
    // plan 不是单一关节的一个值，而是包含了多个关节在一段时间内的完整轨迹。
    // 内部包含了trajectory, 里有一个 points 数组，每个 point 记录了所有相关关节在某一时刻的角度（位置）、速度、加速度等信息
    // 打印每个point的详细信息
    // const auto& points = plan.trajectory_.joint_trajectory.points;
    // for (size_t i = 0; i < points.size(); ++i) {
    //   const auto& point = points[i];
    //   std::cout << "Point " << i << ":" << std::endl;
    //   std::cout << "  positions: ";
    //   for (auto v : point.positions)
    //     std::cout << v << " ";
    // }
    move_group_interface->execute(plan);
    response->success = true;
    response->message = "规划和执行成功";
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
    response->success = false;
    response->message = "规划失败";
  }
}

// 主函数
int main(int argc, char * argv[])
{
  // 初始化ROS
  rclcpp::init(argc, argv);
  
  // 创建节点
  auto node = std::make_shared<rclcpp::Node>(
    "x1_moveit_rviz",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  
  // 创建ROS日志记录器
  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "初始化 x1_moveit_rviz 节点");
  
  // 从SRDF中获取规划组名称
  std::string planning_group_name = "manipulator"; 

  // 等待MoveIt加载
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(2s);
  
  // 创建MoveGroup接口
  auto move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node, planning_group_name);
  
  // 加载机器人模型
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  
  // 创建机器人状态
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  
  // 获取规划组的关节名称
  std::vector<std::string> joint_names = move_group_interface->getJointNames();
  
  RCLCPP_INFO(logger, "x1_moveit_rviz 初始化完成, 使用规划组: %s", planning_group_name.c_str());
  RCLCPP_INFO(logger, "关节数量: %ld", joint_names.size());
  for (const auto & joint_name : joint_names) {
    RCLCPP_INFO(logger, "关节: %s", joint_name.c_str());
  }
  
  // 创建Joycon服务服务器
  auto joycon_service = node->create_service<x1_moveit_proto::srv::JoyconCommand>(
    "joycon_command",
    [node, move_group_interface, robot_model, robot_state, planning_group_name, joint_names]
    (const std::shared_ptr<x1_moveit_proto::srv::JoyconCommand::Request> request,
     std::shared_ptr<x1_moveit_proto::srv::JoyconCommand::Response> response) {
      handleJoyconCommand(request, response, node, move_group_interface, robot_model, robot_state, 
                    planning_group_name, joint_names);
    }
  );
  
  RCLCPP_INFO(logger, "服务器已启动，等待服务请求...");
  
  // 运行节点
  rclcpp::spin(node);
  
  // 关闭ROS
  rclcpp::shutdown();
  return 0;
}
