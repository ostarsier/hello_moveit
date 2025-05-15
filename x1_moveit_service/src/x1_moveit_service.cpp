#include <memory>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
  
  // 确保 MoveIt 演示环境已经启动
  RCLCPP_INFO(logger, "等待MoveIt演示环境准备就绪...");
  std::this_thread::sleep_for(std::chrono::seconds(3));


  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "left_arm");
  
  // 输出机械臂当前状态信息
  RCLCPP_INFO(logger, "规划组名称: %s", move_group_interface.getName().c_str());
  RCLCPP_INFO(logger, "参考坐标系: %s", move_group_interface.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "末端执行器链接: %s", move_group_interface.getEndEffectorLink().c_str());

  // 等待一段时间，确保MoveIt系统已经准备好
  RCLCPP_INFO(logger, "等待2秒，确保系统准备就绪...");
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  // 获取关节名称
  std::vector<std::string> joint_names = move_group_interface.getJointNames();
  RCLCPP_INFO(logger, "规划组中的关节数量: %ld", joint_names.size());
  
  for (const auto& joint_name : joint_names) {
    RCLCPP_INFO(logger, "关节名称: %s", joint_name.c_str());
  }
  
  // 输出当前机械臂位置信息
  RCLCPP_INFO(logger, "获取当前机械臂位置信息...");
  
  try {
    // 获取当前关节值
    auto current_joint_values = move_group_interface.getCurrentJointValues();
    
    for (size_t i = 0; i < joint_names.size() && i < current_joint_values.size(); ++i) {
      RCLCPP_INFO(logger, "关节 %s 当前值: %.3f", joint_names[i].c_str(), current_joint_values[i]);
    }
    
    // 设置一个简单的目标 - 将第一个关节旋转一小角度
    if (!current_joint_values.empty()) {
      current_joint_values[0] += 0.1;  // 增加第一个关节的角度
      RCLCPP_INFO(logger, "设置关节 %s 的目标位置为: %.3f", 
                  joint_names[0].c_str(), current_joint_values[0]);
      
      move_group_interface.setJointValueTarget(current_joint_values);
      RCLCPP_INFO(logger, "已设置关节目标位置");
    } else {
      // 如果无法获取当前关节值，则使用随机目标
      RCLCPP_INFO(logger, "关节值列表为空，设置随机目标位置...");
      move_group_interface.setRandomTarget();
      RCLCPP_INFO(logger, "已设置随机目标位置");
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "获取当前关节值失败: %s", e.what());
    
    // 如果出错，尝试设置随机目标
    RCLCPP_INFO(logger, "设置随机目标位置...");
    move_group_interface.setRandomTarget();
    RCLCPP_INFO(logger, "已设置随机目标位置");
  }
  
  // 设置更长的规划时间和更多的尝试次数
  move_group_interface.setPlanningTime(15.0);
  move_group_interface.setNumPlanningAttempts(10);
  
  // 输出规划尝试信息
  RCLCPP_INFO(logger, "尝试规划到随机目标位置...");
  
  // 创建到目标位置的运动计划
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  
  // 输出规划结果
  if(success) {
    RCLCPP_INFO(logger, "规划成功！轨迹点数量: %ld", plan.trajectory_.joint_trajectory.points.size());
  }

  // 执行计划
  if(success) {
    RCLCPP_INFO(logger, "开始执行机械臂运动计划...");
    
    // 设置执行速度和加速度
    move_group_interface.setMaxVelocityScalingFactor(0.1); // 降低速度以确保安全
    move_group_interface.setMaxAccelerationScalingFactor(0.1); // 降低加速度
    
    // 打印轨迹信息
    RCLCPP_INFO(logger, "轨迹包含 %ld 个点", plan.trajectory_.joint_trajectory.points.size());
    if (!plan.trajectory_.joint_trajectory.points.empty()) {
      RCLCPP_INFO(logger, "第一个点的时间戳: %.3f", 
                  plan.trajectory_.joint_trajectory.points[0].time_from_start.sec + 
                  plan.trajectory_.joint_trajectory.points[0].time_from_start.nanosec / 1e9);
      
      if (plan.trajectory_.joint_trajectory.points.size() > 1) {
        auto last_point = plan.trajectory_.joint_trajectory.points.back();
        RCLCPP_INFO(logger, "最后一个点的时间戳: %.3f", 
                    last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9);
      }
    }
    
    // 执行计划
    RCLCPP_INFO(logger, "开始执行...");
    auto execution_result = move_group_interface.execute(plan);
    
    if(execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "机械臂运动计划执行完成");
    } else {
      RCLCPP_ERROR(logger, "机械臂运动计划执行失败，错误码: %d", execution_result.val);
      RCLCPP_INFO(logger, "错误码含义：");
      RCLCPP_INFO(logger, "-1: FAILURE");
      RCLCPP_INFO(logger, "-2: PLANNING_FAILED");
      RCLCPP_INFO(logger, "-3: INVALID_MOTION_PLAN");
      RCLCPP_INFO(logger, "-4: MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE");
      RCLCPP_INFO(logger, "-5: CONTROL_FAILED");
      RCLCPP_INFO(logger, "-6: UNABLE_TO_ACQUIRE_SENSOR_DATA");
      RCLCPP_INFO(logger, "-7: TIMED_OUT");
      RCLCPP_INFO(logger, "-8: PREEMPTED");
      RCLCPP_INFO(logger, "-9: START_STATE_IN_COLLISION");
      RCLCPP_INFO(logger, "-10: START_STATE_VIOLATES_PATH_CONSTRAINTS");
    }
  } else {
    RCLCPP_ERROR(logger, "规划失败！尝试使用不同的目标位置或检查机器人配置");
    
    // 输出可能的错误原因
    RCLCPP_INFO(logger, "可能的原因：");
    RCLCPP_INFO(logger, "1. 目标位置超出机械臂工作空间");
    RCLCPP_INFO(logger, "2. 没有正确加载运动学插件");
    RCLCPP_INFO(logger, "3. 机械臂配置文件中缺少必要的碰撞几何信息");
    RCLCPP_INFO(logger, "4. 没有运行demo.launch.py启动MoveIt演示环境");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}