// 包含必要的头文件
#include <memory>              // 用于智能指针
#include <rclcpp/rclcpp.hpp>  // ROS2 C++ 客户端库
#include <moveit/move_group_interface/move_group_interface.h>  // MoveIt 2 接口
#include <hiredis/hiredis.h>  // Redis 客户端库
#include <jsoncpp/json/json.h>  // JSON 处理库
#include <string>  // 字符串处理
#include <vector>  // 向量容器
#include <tf2/LinearMath/Matrix3x3.h>  // 用于四元数到欧拉角的转换
#include <tf2/LinearMath/Quaternion.h>  // 用于四元数操作

// 将位姿数据转换为JSON字符串（使用数组格式：[x,y,z,roll,pitch,yaw]）
std::string poseToJson(const geometry_msgs::msg::Pose& pose) {
  // 将四元数转换为欧拉角
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  Json::Value root(Json::arrayValue);
  root.append(pose.position.x);
  root.append(pose.position.y);
  root.append(pose.position.z);
  root.append(roll);
  root.append(pitch);
  root.append(yaw);
  
  Json::StreamWriterBuilder writer_builder;
  return Json::writeString(writer_builder, root);
}

// 从JSON字符串解析位姿数据（从数组格式：[x,y,z,roll,pitch,yaw]）
geometry_msgs::msg::Pose jsonToPose(const std::string& json_str) {
  Json::Value root;
  Json::CharReaderBuilder reader_builder;
  std::string errors;
  std::istringstream json_stream(json_str);
  
  bool parsingSuccessful = Json::parseFromStream(reader_builder, json_stream, &root, &errors);
  
  geometry_msgs::msg::Pose pose;
  if (parsingSuccessful && root.isArray() && root.size() == 6) {
    pose.position.x = root[0].asDouble();
    pose.position.y = root[1].asDouble();
    pose.position.z = root[2].asDouble();
    
    // 将欧拉角转换为四元数
    double roll = root[3].asDouble();
    double pitch = root[4].asDouble();
    double yaw = root[5].asDouble();
    
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
  }
  
  return pose;
}

// 将关节状态转换为JSON字符串（数组格式：[j1,j2,j3,j4,j5,j6]）
std::string jointStatesToJson(const std::vector<double>& joint_values) {
  Json::Value root(Json::arrayValue);
  
  for (size_t i = 0; i < joint_values.size(); ++i) {
    root.append(joint_values[i]);
  }
  
  Json::StreamWriterBuilder writer_builder;
  return Json::writeString(writer_builder, root);
}

// 规划运动并返回关节状态
bool planMotion(moveit::planning_interface::MoveGroupInterface& move_group_interface, 
               const geometry_msgs::msg::Pose& target_pose,
               std::vector<double>& joint_values,
               const rclcpp::Logger& logger) {
  // 将目标位姿设置给运动规划接口
  move_group_interface.setPoseTarget(target_pose);

  // 生成运动规划
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_interface.plan(plan));

  // 如果规划成功，获取关节状态
  if (success) {
    // 获取规划后的关节状态
    joint_values = move_group_interface.getCurrentJointValues();
    return true;
  } else {
    // 如果规划失败，输出错误日志
    RCLCPP_ERROR(logger, "运动规划失败！");
    return false;
  }
}

int main(int argc, char * argv[])
{
  // 初始化 ROS2 客户端库
  rclcpp::init(argc, argv);

  // 创建一个 ROS2 节点
  // automatically_declare_parameters_from_overrides(true) 允许从启动文件自动声明参数
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // 创建一个 ROS2 日志记录器
  auto const logger = rclcpp::get_logger("hello_moveit");

  // 创建 MoveIt 的运动规划接口
  // MoveGroupInterface 是与机器人进行交互的主要接口
  // "panda_arm" 是要控制的机器人规划组的名称
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // 连接到Redis服务器
  redisContext *redis_context = redisConnect("192.168.110.91", 6379);
  if (redis_context == nullptr || redis_context->err) {
    if (redis_context) {
      RCLCPP_ERROR(logger, "Redis连接错误: %s", redis_context->errstr);
      redisFree(redis_context);
    } else {
      RCLCPP_ERROR(logger, "无法分配Redis上下文");
    }
    rclcpp::shutdown();
    return 1;
  }
  
  RCLCPP_INFO(logger, "已连接到Redis服务器");
  
  // 主循环，从Redis获取位姿数据并进行运动规划
  while (rclcpp::ok()) {
    // 从Redis列表中获取位姿数据
    redisReply *reply = (redisReply *)redisCommand(redis_context, "BLPOP vla_moveit_position 5");
    
    if (reply == nullptr) {
      RCLCPP_ERROR(logger, "Redis命令执行失败");
      continue;
    }
    
    // 检查回复类型
    if (reply->type == REDIS_REPLY_NIL) {
      RCLCPP_INFO(logger, "等待位姿数据...");
      freeReplyObject(reply);
      continue;
    } else if (reply->type == REDIS_REPLY_ARRAY && reply->elements == 2) {
      // BLPOP返回一个包含两个元素的数组：键名和值
      std::string json_str = reply->element[1]->str;
      RCLCPP_INFO(logger, "收到位姿数据: %s", json_str.c_str());
      
      // 解析JSON数据为位姿
      geometry_msgs::msg::Pose target_pose = jsonToPose(json_str);
      
      // 规划运动
      std::vector<double> joint_values;
      bool success = planMotion(move_group_interface, target_pose, joint_values, logger);
      
      if (success) {
        // 将关节状态转换为JSON
        std::string joint_json = jointStatesToJson(joint_values);
        RCLCPP_INFO(logger, "规划成功，关节状态: %s", joint_json.c_str());
        
        // 将关节状态发送到Redis
        redisReply *push_reply = (redisReply *)redisCommand(redis_context, 
                                                         "RPUSH vla_moveit_joint %s", 
                                                         joint_json.c_str());
        if (push_reply == nullptr) {
          RCLCPP_ERROR(logger, "无法将关节状态发送到Redis");
        } else {
          RCLCPP_INFO(logger, "已将关节状态发送到Redis");
          freeReplyObject(push_reply);
        }
      }
    } else {
      RCLCPP_ERROR(logger, "收到意外的Redis回复类型");
    }
    
    freeReplyObject(reply);
    
    // 处理ROS2事件
    rclcpp::spin_some(node);
  }
  
  // 释放Redis连接
  redisFree(redis_context);
  
  // 关闭 ROS2 客户端库
  rclcpp::shutdown();
  return 0;
}