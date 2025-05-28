#ifndef X1_SYSTEM_INTERFACE_HPP_
#define X1_SYSTEM_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// hiredis客户端库
#include <hiredis/hiredis.h>
#include <string.h>
#include <sstream>

namespace x1_hardware_interface
{

class X1SystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(X1SystemInterface);
  
  // 析构函数
  ~X1SystemInterface();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 存储关节位置和速度的状态
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  
  // hiredis客户端
  redisContext* redis_context_ = nullptr;
  std::string redis_host_ = "127.0.0.1";
  int redis_port_ = 6379;
  std::string joint_position_key_ = "joint_position";
  
  // 在实际项目中，您可能需要添加与硬件通信的成员变量，例如：
  // std::unique_ptr<SerialPort> serial_connection_;
  // std::string device_name_;
  // int baudrate_;
};

}  // namespace x1_hardware_interface

#endif  // X1_SYSTEM_INTERFACE_HPP_
