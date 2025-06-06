#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/parameter.hpp>
#include <fstream> // 用于文件输出

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

  // 声明姿态参数并设置默认值（使用double类型）
  node->declare_parameter("qx", 0.0);
  node->declare_parameter("qy", 0.0);
  node->declare_parameter("qz", 0.0);
  node->declare_parameter("qw", 1.0); // 通常qw默认为1表示无旋转

  double qx = node->get_parameter("qx").as_double();
  double qy = node->get_parameter("qy").as_double();
  double qz = node->get_parameter("qz").as_double();
  double qw = node->get_parameter("qw").as_double();

  
  // 输出当前使用的规划组名称
  RCLCPP_INFO(logger, "使用规划组: %s", move_group_interface.getName().c_str());
  
  RCLCPP_INFO(logger, "目标位置: x=%.2f, y=%.2f, z=%.2f", x, y, z);
  
  // 设置目标姿态
  auto const target_pose = [x, y, z, qx, qy, qz, qw]{
    geometry_msgs::msg::Pose msg;
    // 设置位置
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    
    // 计算四元数
    msg.orientation.w = qw;
    msg.orientation.x = qx;
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    
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
    // 将规划结果写入文件
    std::ofstream plan_file("plan_details_rviz.log");
    if (plan_file.is_open()) {
      for (size_t i = 0; i < plan.trajectory_.joint_trajectory.joint_names.size(); ++i) {
        plan_file << plan.trajectory_.joint_trajectory.joint_names[i];
        if (i < plan.trajectory_.joint_trajectory.joint_names.size() - 1) {
          plan_file << ",";
        }
      }
      plan_file << "\n";

      for (size_t i = 0; i < plan.trajectory_.joint_trajectory.points.size(); ++i) {
        const auto& point = plan.trajectory_.joint_trajectory.points[i];
        
        for (size_t j = 0; j < point.positions.size(); ++j) {
          plan_file << point.positions[j];
          if (j < point.positions.size() - 1) {
            plan_file << ",";
          }
        }
        plan_file << "\n";
      }
      plan_file.close();
      RCLCPP_INFO(logger, "Plan details written to plan_details_rviz.log");
    } else {
      RCLCPP_ERROR(logger, "Unable to open plan_details_rviz.log for writing");
    }

    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}