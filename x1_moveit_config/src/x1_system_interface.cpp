#include "x1_moveit_config/x1_system_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace x1_hardware_interface
{

// 析构函数，释放Redis连接资源
X1SystemInterface::~X1SystemInterface()
{
  if (redis_context_ != nullptr) {
    redisFree(redis_context_);
    redis_context_ = nullptr;
    RCLCPP_INFO(rclcpp::get_logger("X1SystemInterface"), "Redis连接已释放");
  }
}
hardware_interface::CallbackReturn X1SystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 初始化硬件资源
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // X1机器人应该支持位置控制
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("X1SystemInterface"),
        "Joint '%s' has %zu command interfaces found, but 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 检查状态接口
    if (joint.state_interfaces.size() != 2 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("X1SystemInterface"),
        "Joint '%s' has %zu state interfaces, but 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // 初始化Redis连接
  struct timeval timeout = { 1, 500000 }; // 1.5秒超时
  redis_context_ = redisConnectWithTimeout(redis_host_.c_str(), redis_port_, timeout);
  
  if (redis_context_ == nullptr || redis_context_->err) {
    if (redis_context_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("X1SystemInterface"),
        "Redis连接错误: %s", redis_context_->errstr);
      redisFree(redis_context_);
      redis_context_ = nullptr;
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("X1SystemInterface"),
        "无法分配 Redis内存");
    }
    // 虽然Redis连接失败，但我们不阻止系统启动，而是会记录错误
  } else {
    RCLCPP_INFO(rclcpp::get_logger("X1SystemInterface"), 
               "成功连接到Redis %s:%d", redis_host_.c_str(), redis_port_);
  }
  
  RCLCPP_INFO(rclcpp::get_logger("X1SystemInterface"), "Successfully initialized");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn X1SystemInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 配置硬件资源
  // 例如：建立连接，设置参数等
  
  // 初始化关节位置为默认值或从参数中读取
  for (uint i = 0; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
    }
    if (std::isnan(hw_velocities_[i])) {
      hw_velocities_[i] = 0;
    }
    if (std::isnan(hw_commands_[i])) {
      hw_commands_[i] = hw_positions_[i];
    }
  }
  
  RCLCPP_INFO(rclcpp::get_logger("X1SystemInterface"), "Successfully configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn X1SystemInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 清理资源
  // 例如：关闭连接等
  
  RCLCPP_INFO(rclcpp::get_logger("X1SystemInterface"), "Successfully cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn X1SystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 激活硬件
  // 例如：启用电机，进入操作模式等
  
  // 确保命令与当前位置一致
  for (uint i = 0; i < hw_positions_.size(); i++) {
    hw_commands_[i] = hw_positions_[i];
  }
  
  RCLCPP_INFO(rclcpp::get_logger("X1SystemInterface"), "Successfully activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn X1SystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 停用硬件
  // 例如：禁用电机，进入安全模式等
  
  RCLCPP_INFO(rclcpp::get_logger("X1SystemInterface"), "Successfully deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> X1SystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> X1SystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  
  return command_interfaces;
}

hardware_interface::return_type X1SystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 从实际硬件读取当前状态
  // 例如：读取编码器数据，通过通信协议获取关节位置等
  
  // 这里是示例代码，实际实现需要根据硬件接口修改
  // 在真实系统中，您需要实现与硬件的通信代码
  // 例如通过串口/CAN/以太网等与实际硬件通信
  
  // 为了演示，这里简单模拟一个逐渐靠近目标位置的系统
  for (uint i = 0; i < hw_positions_.size(); i++) {
    // 简单的位置更新模拟，真实系统应替换为实际读取
    double error = hw_commands_[i] - hw_positions_[i];
    hw_positions_[i] += error * 0.1;  // 简单的P控制器模拟
    hw_velocities_[i] = error * 0.1;  // 速度估计
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type X1SystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 将关节数组写入Redis
  RCLCPP_DEBUG(rclcpp::get_logger("X1SystemInterface"), "Writing joint positions to Redis");
  
  if (redis_context_ != nullptr && !redis_context_->err) {
    // 将关节位置数组转换为JSON字符串
    std::stringstream json_stream;
    json_stream << "[";
    
    // 只处理右手关节（假设右手关节从索引0开始）
    for (size_t i = 0; i < hw_commands_.size(); ++i) {
      if (i > 0) {
        json_stream << ",";
      }
      json_stream << hw_commands_[i];
      
      RCLCPP_DEBUG(rclcpp::get_logger("X1SystemInterface"), 
                  "Joint %zu command: %f", i, hw_commands_[i]);
    }
    
    json_stream << "]";
    std::string json_array = json_stream.str();
    
    // 将数组存储到Redis中
    redisReply *reply = (redisReply *)redisCommand(redis_context_, "SET %s %s", 
                                                 joint_position_key_.c_str(), 
                                                 json_array.c_str());
    
    if (reply == nullptr) {
      RCLCPP_ERROR(rclcpp::get_logger("X1SystemInterface"),
                 "Redis命令执行失败: %s", redis_context_->errstr);
      return hardware_interface::return_type::ERROR;
    } else {
      if (reply->type == REDIS_REPLY_ERROR) {
        RCLCPP_ERROR(rclcpp::get_logger("X1SystemInterface"),
                    "Redis错误: %s", reply->str);
        freeReplyObject(reply);
        return hardware_interface::return_type::ERROR;
      }
      
      RCLCPP_DEBUG(rclcpp::get_logger("X1SystemInterface"),
                  "已将关节位置发送到Redis，键: '%s': %s",
                  joint_position_key_.c_str(), json_array.c_str());
      
      freeReplyObject(reply);
    }
  } else {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("X1SystemInterface"),
                       *rclcpp::Clock::make_shared(), 5000,
                       "Redis客户端未初始化或连接错误，跳过关节位置更新");
  }
  
  return hardware_interface::return_type::OK;
}

}  // namespace x1_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  x1_hardware_interface::X1SystemInterface,
  hardware_interface::SystemInterface
)
