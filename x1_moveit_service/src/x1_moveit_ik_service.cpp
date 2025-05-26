#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <x1_moveit_proto/srv/ik_solve.hpp>
#include <vector>
#include <algorithm>
#include <limits>
#include <memory>
#include <chrono>
#include <functional>

class IkSolveService : public rclcpp::Node
{
public:
  IkSolveService() : Node("ik_solve_service")
  {
    // 设置运动学求解器参数
    this->declare_parameter("robot_description_kinematics.manipulator.kinematics_solver", 
                           "kdl_kinematics_plugin/KDLKinematicsPlugin");
    this->declare_parameter("robot_description_kinematics.manipulator.kinematics_solver_timeout", 0.005);
    this->declare_parameter("robot_description_kinematics.manipulator.kinematics_solver_attempts", 100);
    this->declare_parameter("robot_description_kinematics.manipulator.kinematics_solver_search_resolution", 0.005);

    // 初始化完成后的定时器，用于延迟加载机器人模型
    // 这是因为在构造函数中不能使用shared_from_this()
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&IkSolveService::initializeRobotModel, this));
    
    RCLCPP_INFO(this->get_logger(), "初始化中...");
  }

private:
  // 初始化机器人模型和服务
  void initializeRobotModel()
  {
    // 取消定时器，防止多次初始化
    timer_->cancel();
    
    RCLCPP_INFO(this->get_logger(), "加载机器人模型...");
    
    try {
      // 使用shared_from_this()创建机器人模型加载器
      robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this());
      robot_model_loader_->loadKinematicsSolvers();
      
      robot_model_ = robot_model_loader_->getModel();
      if (!robot_model_)
      {
        RCLCPP_ERROR(this->get_logger(), "加载机器人模型失败。请确保'robot_description'参数已设置。");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "机器人模型已加载: %s", robot_model_->getModelFrame().c_str());
      
      // 获取规划组
      planning_group_ = "manipulator";
      joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
      if (!joint_model_group_)
      {
        RCLCPP_ERROR(this->get_logger(), "获取关节模型组'%s'失败。请检查SRDF配置。", planning_group_.c_str());
        return;
      }
      RCLCPP_INFO(this->get_logger(), "规划组'%s'已加载。", planning_group_.c_str());

      // 创建机器人状态
      kinematic_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
      
      // 获取末端执行器链接
      end_effector_link_ = joint_model_group_->getLinkModelNames().back();
      RCLCPP_INFO(this->get_logger(), "末端执行器链接: %s", end_effector_link_.c_str());

      // 创建服务
      service_ = this->create_service<x1_moveit_proto::srv::IkSolve>(
        "ik_solve",
        std::bind(&IkSolveService::handleIkSolveRequest, this, std::placeholders::_1, std::placeholders::_2)
      );
      
      RCLCPP_INFO(this->get_logger(), "IK求解服务已启动，等待请求...");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "初始化失败: %s", e.what());
    }
  }
  
  void handleIkSolveRequest(
    const std::shared_ptr<x1_moveit_proto::srv::IkSolve::Request> request,
    std::shared_ptr<x1_moveit_proto::srv::IkSolve::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "收到IK求解请求: x=%f, y=%f, z=%f", 
               request->x, request->y, request->z);
    
    // 设置目标位姿
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = request->x;
    target_pose.position.y = request->y;
    target_pose.position.z = request->z;
    
    // 设置方向为固定值 (0.5, 0.5, 0.5, 0.5)
    // 注意：这里可以根据需要修改，或者扩展服务接口包含方向信息
    target_pose.orientation.x = 0.500;
    target_pose.orientation.y = 0.500;
    target_pose.orientation.z = 0.500;
    target_pose.orientation.w = 0.500;
    
    // 将geometry_msgs::msg::Pose转换为Eigen::Isometry3d
    Eigen::Isometry3d target_pose_eigen;
    tf2::fromMsg(target_pose, target_pose_eigen);
    
    // 初始化变量，用于保存多个IK解
    const int num_attempts = 10; // 尝试次数
    std::vector<std::vector<double>> solutions;
    std::vector<double> initial_joint_values;
    
    // 获取当前关节状态作为参考
    kinematic_state_->copyJointGroupPositions(joint_model_group_, initial_joint_values);
    
    // 尝试多次求解IK，收集所有解
    for (int i = 0; i < num_attempts; ++i) {
      // 为每次尝试设置一个稍微不同的随机种子状态
      moveit::core::RobotState random_state = *kinematic_state_;
      random_state.setToRandomPositionsNearBy(joint_model_group_, *kinematic_state_, 0.1);
      
      // 获取随机种子关节值
      std::vector<double> random_seed_values;
      random_state.copyJointGroupPositions(joint_model_group_, random_seed_values);
      
      // 从随机种子状态开始求解
      moveit::core::RobotState test_state = *kinematic_state_;
      test_state.setJointGroupPositions(joint_model_group_, random_seed_values);
      
      // 调用setFromIK求解
      double timeout = 0.5; // 超时时间（秒）
      bool this_found_ik = test_state.setFromIK(joint_model_group_, target_pose_eigen, end_effector_link_, timeout);
      
      if (this_found_ik) {
        // 如果找到解，保存到解集合中
        std::vector<double> solution;
        test_state.copyJointGroupPositions(joint_model_group_, solution);
        solutions.push_back(solution);
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
        
        RCLCPP_INFO(this->get_logger(), "解 %zu 与初始状态的距离: %f", i+1, distance);
        
        if (distance < min_distance) {
          min_distance = distance;
          best_solution_idx = i;
        }
      }
      
      // RCLCPP_INFO(this->get_logger(), "选择解 %zu 作为最佳解（距离: %f）", best_solution_idx+1, min_distance);
      
      // 设置响应
      response->success = true;
      response->message = "IK求解成功";
      
      // 获取关节名称和值
      const std::vector<std::string>& joint_names = joint_model_group_->getVariableNames();
      std::vector<double> joint_values = solutions[best_solution_idx];
      
      // 设置响应中的关节名称和值
      response->joint_names.assign(joint_names.begin(), joint_names.end());
      response->joint_values.assign(joint_values.begin(), joint_values.end());
      
      // 日志输出
      RCLCPP_INFO(this->get_logger(), "最佳IK解的关节角度:");
      for (std::size_t i = 0; i < joint_names.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "  %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
    } else {
      // 未找到解
      response->success = false;
      response->message = "无法找到IK解";
      RCLCPP_WARN(this->get_logger(), "无法找到目标位姿的IK解");
    }
  }

  // 成员变量
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  std::shared_ptr<moveit::core::RobotState> kinematic_state_;
  std::string planning_group_;
  const moveit::core::JointModelGroup* joint_model_group_ = nullptr;
  std::string end_effector_link_;
  rclcpp::Service<x1_moveit_proto::srv::IkSolve>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IkSolveService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
