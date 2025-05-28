#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/parameter.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  // 创建ROS节点，关闭automatically_declare_parameters_from_overrides选项
  auto const node = std::make_shared<rclcpp::Node>(
    "x1_moveit_rviz"
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("x1_moveit_rviz");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  // 声明参数并设置默认值（使用double类型）
  node->declare_parameter("x", 0.0);
  node->declare_parameter("y", 0.0);
  node->declare_parameter("z", 0.0);
  
  // 获取参数值
  double x = node->get_parameter("x").as_double();
  double y = node->get_parameter("y").as_double();
  double z = node->get_parameter("z").as_double();

  
  // 输出当前使用的规划组名称
  RCLCPP_INFO(logger, "使用规划组: %s", move_group_interface.getName().c_str());
  
  RCLCPP_INFO(logger, "目标位置: x=%.2f, y=%.2f, z=%.2f", x, y, z);
  
  // 设置目标姿态
  auto const target_pose = [x, y, z]{
    geometry_msgs::msg::Pose msg;
    // 设置位置
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    
    // 计算四元数
    msg.orientation.w = 0.500;
    msg.orientation.x = 0.500;
    msg.orientation.y = 0.500;
    msg.orientation.z = 0.500;
    
    return msg;
  }();
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