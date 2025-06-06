#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vector>
#include <algorithm> // for std::min_element
#include <limits> // for std::numeric_limits

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2_ik_demo");

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit2_ik_demo_node");
  // 参数服务器上可以看到kinematics.yaml的值，但执行会报错找不到，所以这里显示设置
  node->declare_parameter("robot_description_kinematics.manipulator.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
  node->declare_parameter("robot_description_kinematics.manipulator.kinematics_solver_timeout", 0.005);
  // 对于KDL，增加尝试次数和超时时间可能有助于找到解
  node->declare_parameter("robot_description_kinematics.manipulator.kinematics_solver_attempts", 100); // 增加尝试次数
  node->declare_parameter("robot_description_kinematics.manipulator.kinematics_solver_search_resolution", 0.005); // 设置搜索分辨率

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

  // 获取规划组
  const std::string PLANNING_GROUP = "manipulator";
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
  target_pose.position.x = -0.016063;
  target_pose.position.y = -0.25475;
  target_pose.position.z = 0.17403;
  // 给定的四元数 (0.5, 0.5, 0.5, 0.5) 对应的 RPY 约为 (1.047, 1.047, 1.047) rad 或 (60, 60, 60) 度
  target_pose.orientation.x = 0.0052177;
  target_pose.orientation.y = 0.009862;
  target_pose.orientation.z = 0.46937;
  target_pose.orientation.w = 0.88293;

  RCLCPP_INFO(LOGGER, "Attempting to solve IK for target pose:");
  RCLCPP_INFO(LOGGER, "  Position: x=%f, y=%f, z=%f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
  RCLCPP_INFO(LOGGER, "  Orientation: x=%f, y=%f, z=%f, w=%f", target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

  // 确保这个链接是你的规划组中的末端执行器
  const std::string& end_effector_link = joint_model_group->getLinkModelNames().back(); // 假设最后一个链接是末端执行器
  RCLCPP_INFO(LOGGER, "End effector link: %s", end_effector_link.c_str());

  // IK 参数设置
  double timeout = 0.5; // 超时时间（秒），可以根据需要调整

  // 设置种子状态 (Seed State)
  moveit::core::RobotState seed_state = *kinematic_state; // 从当前机器人状态作为种子
  // 或者你可以手动设置种子状态的关节值：
  // std::vector<double> custom_seed_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 零位姿态
  // seed_state.setJointGroupPositions(joint_model_group, custom_seed_joint_values);
  // 如果你知道一个比较好的起始姿态，可以设置在这里，例如机器人的“home”姿态。

  // 将geometry_msgs::msg::Pose转换为Eigen::Isometry3d
  Eigen::Isometry3d target_pose_eigen;
  tf2::fromMsg(target_pose, target_pose_eigen);
  
  std::vector<double> joint_values;

  // 初始化变量，用于保存多个IK解
  const int num_attempts = 10; // 尝试次数
  // solutions = {
  //   {0.1, -0.2, 0.3, 0.4, -0.5, 0.6},  // 第1个IK解
  //   {0.2, -0.1, 0.4, 0.3, -0.6, 0.5}   // 第2个IK解
  // };
  std::vector<std::vector<double>> solutions;
  std::vector<double> initial_joint_values;
  
  // 获取当前关节状态作为参考
  kinematic_state->copyJointGroupPositions(joint_model_group, initial_joint_values);
  
  RCLCPP_INFO(LOGGER, "当前关节状态（参考姿态）:");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    RCLCPP_INFO(LOGGER, "  %s: %f", joint_names[i].c_str(), initial_joint_values[i]);
  }
  
  // 尝试多次求解IK，收集所有解
  for (int i = 0; i < num_attempts; ++i) {
    // 为每次尝试设置一个稍微不同的随机种子状态
    moveit::core::RobotState random_state = *kinematic_state;
    random_state.setToRandomPositionsNearBy(joint_model_group, *kinematic_state, 0.1); // 在当前状态附近随机采样
    
    // 获取随机种子关节值
    std::vector<double> random_seed_values;
    random_state.copyJointGroupPositions(joint_model_group, random_seed_values);
    
    // 从随机种子状态开始求解
    moveit::core::RobotState test_state = *kinematic_state;
    test_state.setJointGroupPositions(joint_model_group, random_seed_values);
    
    // 调用setFromIK求解
    bool this_found_ik = test_state.setFromIK(joint_model_group, target_pose_eigen, end_effector_link, timeout);
    
    if (this_found_ik) {
      // 如果找到解，保存到解集合中
      std::vector<double> solution;
      test_state.copyJointGroupPositions(joint_model_group, solution);
      solutions.push_back(solution);
      
      RCLCPP_INFO(LOGGER, "找到第 %d 个IK解:", static_cast<int>(solutions.size()));
      for (std::size_t j = 0; j < joint_names.size(); ++j) {
        RCLCPP_INFO(LOGGER, "  %s: %f", joint_names[j].c_str(), solution[j]);
      }
    }
  }
  
  bool found_ik = !solutions.empty();
  
  if (found_ik) {
    // 找到了至少一个解，选择关节变化最小的那个
    size_t best_solution_idx = 0;
    double min_distance = std::numeric_limits<double>::max();
    
    // 计算每个解与初始状态的欧几里德距离
    for (size_t i = 0; i < solutions.size(); ++i) {
      double distance = 0.0;
      for (size_t j = 0; j < initial_joint_values.size(); ++j) {
        double diff = solutions[i][j] - initial_joint_values[j];
        distance += diff * diff;
      }
      distance = std::sqrt(distance);
      
      RCLCPP_INFO(LOGGER, "解 %zu 与初始状态的距离: %f", i+1, distance);
      
      if (distance < min_distance) {
        min_distance = distance;
        best_solution_idx = i;
      }
    }
    
    RCLCPP_INFO(LOGGER, "选择解 %zu 作为最佳解（距离: %f）", best_solution_idx+1, min_distance);
    
    // 将最佳解应用到机器人状态
    // kinematic_state->setJointGroupPositions(joint_model_group, solutions[best_solution_idx]);
    std::vector<double> joint_values = solutions[best_solution_idx];
    RCLCPP_INFO(LOGGER, "IK solution found!");
    // 打印最佳解的关节角度
    RCLCPP_INFO(LOGGER, "最佳IK解的关节角度:");
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      RCLCPP_INFO(LOGGER, "  %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    
    // 验证IK解
    // const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector_link);
    // geometry_msgs::msg::Pose current_pose;
    // current_pose = tf2::toMsg(end_effector_state);
    
    // RCLCPP_INFO(LOGGER, "Verification - End-effector pose from IK solution:");
    // RCLCPP_INFO(LOGGER, "  Position: x=%f, y=%f, z=%f", 
    //           current_pose.position.x, current_pose.position.y, current_pose.position.z);
    // RCLCPP_INFO(LOGGER, "  Orientation: x=%f, y=%f, z=%f, w=%f", 
    //           current_pose.orientation.x, current_pose.orientation.y, 
    //           current_pose.orientation.z, current_pose.orientation.w);
  } else {
    RCLCPP_WARN(LOGGER, "Could not find an IK solution for the target pose.");
    // 如果没有找到解，可能需要检查以下几点：
    // 1. 目标位姿是否在机器人工作空间内？
    // 2. 目标位姿是否会导致自碰撞或与环境碰撞？
    // 3. kinematics.yaml 中的参数是否合适？ (timeout, attempts, resolution)
    // 4. URDF/SRDF 模型是否正确？关节限制是否合理？
  }

  rclcpp::shutdown();
  return 0;
}