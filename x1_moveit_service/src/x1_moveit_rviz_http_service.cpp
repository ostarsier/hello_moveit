#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <thread>
#include <chrono>
#include <ctime>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

// 使用单文件HTTP库
#define CPPHTTPLIB_THREAD_POOL_COUNT 4
#include "../include/httplib.h"

class MoveItRvizHttpServer {
public:
  MoveItRvizHttpServer(const std::shared_ptr<rclcpp::Node>& node)
  : node_(node), logger_(rclcpp::get_logger("x1_moveit_rviz_http_service")) {
    // 创建MoveIt接口
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "manipulator");
    
    // 输出当前使用的规划组名称
    RCLCPP_INFO(logger_, "使用规划组: %s", move_group_interface_->getName().c_str());
  }

  void startServer(int port = 8080) {
    // 创建一个指向Server对象的智能指针，以便在整个对象生命周期中持有它
    server_ptr_ = std::make_shared<httplib::Server>();
    auto& svr = *server_ptr_;


    // 健康检查端点
    svr.Get("/health", [this](const httplib::Request& req, httplib::Response& res) {
      res.set_content("OK", "text/plain");
    });

    // 获取机器人当前状态
    svr.Get("/status", [this](const httplib::Request& req, httplib::Response& res) {
      auto current_state = move_group_interface_->getCurrentState();
      auto current_pose = move_group_interface_->getCurrentPose();
      
      std::stringstream ss;
      ss << "{\n";
      ss << "  \"planning_group\": \"" << move_group_interface_->getName() << "\",\n";
      ss << "  \"current_pose\": {\n";
      ss << "    \"position\": {\n";
      ss << "      \"x\": " << current_pose.pose.position.x << ",\n";
      ss << "      \"y\": " << current_pose.pose.position.y << ",\n";
      ss << "      \"z\": " << current_pose.pose.position.z << "\n";
      ss << "    },\n";
      ss << "    \"orientation\": {\n";
      ss << "      \"x\": " << current_pose.pose.orientation.x << ",\n";
      ss << "      \"y\": " << current_pose.pose.orientation.y << ",\n";
      ss << "      \"z\": " << current_pose.pose.orientation.z << ",\n";
      ss << "      \"w\": " << current_pose.pose.orientation.w << "\n";
      ss << "    }\n";
      ss << "  }\n";
      ss << "}\n";
      
      res.set_content(ss.str(), "application/json");
    });

    // 创建移动请求处理函数
    auto handle_move = [this](const httplib::Request& req, httplib::Response& res) {
      try {
        // 输出请求内容以调试
        RCLCPP_INFO(logger_, "接收到请求: %s %s", req.method.c_str(), req.path.c_str());
        
        // 输出所有参数
        RCLCPP_INFO(logger_, "请求参数数量: %zu", req.params.size());
        for (const auto& param : req.params) {
          RCLCPP_INFO(logger_, "参数: %s = %s", param.first.c_str(), param.second.c_str());
        }
        
        // 解析请求参数
        double x = 0.0, y = 0.0, z = 0.0;
        double qx = 0.5, qy = 0.5, qz = 0.5, qw = 0.5;
        
        // 检查参数并转换
        if (req.has_param("x")) {
          std::string x_str = req.get_param_value("x");
          RCLCPP_INFO(logger_, "解析x参数: %s", x_str.c_str());
          x = std::stod(x_str);
        }
        if (req.has_param("y")) {
          std::string y_str = req.get_param_value("y");
          RCLCPP_INFO(logger_, "解析y参数: %s", y_str.c_str());
          y = std::stod(y_str);
        }
        if (req.has_param("z")) {
          std::string z_str = req.get_param_value("z");
          RCLCPP_INFO(logger_, "解析z参数: %s", z_str.c_str());
          z = std::stod(z_str);
        }
        if (req.has_param("qx")) {
          std::string qx_str = req.get_param_value("qx");
          RCLCPP_INFO(logger_, "解析qx参数: %s", qx_str.c_str());
          qx = std::stod(qx_str);
        }
        if (req.has_param("qy")) {
          std::string qy_str = req.get_param_value("qy");
          RCLCPP_INFO(logger_, "解析qy参数: %s", qy_str.c_str());
          qy = std::stod(qy_str);
        }
        if (req.has_param("qz")) {
          std::string qz_str = req.get_param_value("qz");
          RCLCPP_INFO(logger_, "解析qz参数: %s", qz_str.c_str());
          qz = std::stod(qz_str);
        }
        if (req.has_param("qw")) {
          std::string qw_str = req.get_param_value("qw");
          RCLCPP_INFO(logger_, "解析qw参数: %s", qw_str.c_str());
          qw = std::stod(qw_str);
        }

        RCLCPP_INFO(logger_, "目标位置: x=%.2f, y=%.2f, z=%.2f", x, y, z);
        RCLCPP_INFO(logger_, "目标方向: qx=%.2f, qy=%.2f, qz=%.2f, qw=%.2f", qx, qy, qz, qw);
        
        // 设置目标姿态
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        target_pose.orientation.x = qx;
        target_pose.orientation.y = qy;
        target_pose.orientation.z = qz;
        target_pose.orientation.w = qw;
        
        // 设置规划时间和尝试次数
        move_group_interface_->setPlanningTime(10.0); // 设置1秒规划时间
        move_group_interface_->setNumPlanningAttempts(10); // 设置规划尝试次数
        
        // 使用TrajOpt规划器，它是基于优化的规划器，对相同输入更可能产生相同输出
        move_group_interface_->setPlannerId("trajopt");
        RCLCPP_INFO(logger_, "使用TrajOpt规划器");
        
        // 设置目标姿态
        move_group_interface_->setPoseTarget(target_pose);
        
        // 设置速度缩放因子（范围0.1-1.0），设置为0.3表示速度减慃30%
        move_group_interface_->setMaxVelocityScalingFactor(0.3);
        move_group_interface_->setMaxAccelerationScalingFactor(0.3);
        RCLCPP_INFO(logger_, "已设置速度缩放因子为0.3，加速度缩放因子为0.3");
        
        // 创建规划
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        RCLCPP_INFO(logger_, "开始规划...");
        auto start_time = std::chrono::steady_clock::now();
        bool success = static_cast<bool>(move_group_interface_->plan(plan));
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        RCLCPP_INFO(logger_, "规划%s，耗时: %ld 毫秒", success ? "成功" : "失败", duration);

        // 执行规划
        std::string result;
        if(success) {
          // 将规划轨迹点保存到日志文件
          savePlanToLogFile(plan);
          
          // 执行规划
          move_group_interface_->execute(plan);
          result = "{\n  \"status\": \"success\",\n  \"message\": \"规划和执行成功\"\n}";
        } else {
          result = "{\n  \"status\": \"error\",\n  \"message\": \"规划失败\"\n}";
          RCLCPP_ERROR(logger_, "规划失败!");
        }
        
        res.set_content(result, "application/json");
      }
      catch (const std::exception& e) {
        std::string error = "{\n  \"status\": \"error\",\n  \"message\": \"" + std::string(e.what()) + "\"\n}";
        res.set_content(error, "application/json");
        RCLCPP_ERROR(logger_, "执行出错: %s", e.what());
      }
    };
    
    // 注册移动请求处理程序，同时支持GET和POST方法
    svr.Get("/move", handle_move);

    RCLCPP_INFO(logger_, "HTTP服务器启动，监听端口: %d", port);
    
    // 设置一些服务器参数以提高响应能力
    svr.set_keep_alive_max_count(20); // 设置keep-alive连接的最大数量
    svr.set_read_timeout(5); // 读取超时，秒
    svr.set_write_timeout(5); // 写入超时，秒
    
    // 在启动前设置调试信息
    RCLCPP_INFO(logger_, "使用的HTTP库版本: %s", CPPHTTPLIB_VERSION);
    RCLCPP_INFO(logger_, "注册的端点: /health, /status, /move");
    
    // 测试端点，确保日志记录运行正常
    svr.Get("/test", [this](const httplib::Request&, httplib::Response& res) {
      RCLCPP_INFO(logger_, "接收到测试请求");
      res.set_content("Test OK", "text/plain");
    });
    
    // 使用新线程启动服务器
    // 注意这里我们复制了server_ptr_而不是引用，这样可以避免生命周期问题
    auto server = server_ptr_;
    server_thread_ = std::thread([server, port, logger = logger_]() {
      RCLCPP_INFO(logger, "HTTP服务器线程开始运行");
      if (!server->listen("0.0.0.0", port)) {
        RCLCPP_ERROR(logger, "HTTP服务器启动失败！");
      }
    });
    
    // 给服务器一点时间启动
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 清理资源
  ~MoveItRvizHttpServer() {
    // 确保线程关闭
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

private:
  // 将规划轨迹点写入日志文件
  void savePlanToLogFile(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    // 获取当前时间作为文件名的一部分
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream filename;
    filename << "/tmp/moveit_plan_" << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S") << ".log";
    
    // 打开日志文件
    std::ofstream logfile(filename.str());
    if (!logfile.is_open()) {
      RCLCPP_ERROR(logger_, "无法创建日志文件: %s", filename.str().c_str());
      return;
    }
    
    RCLCPP_INFO(logger_, "将规划轨迹点保存到文件: %s", filename.str().c_str());
    
    // 写入轨迹信息的标题
    logfile << "# MoveIt规划轨迹点 - " << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S") << std::endl;
    logfile << "# 规划组: " << move_group_interface_->getName() << std::endl;
    logfile << "# 轨迹点总数: " << plan.trajectory_.joint_trajectory.points.size() << std::endl;
    logfile << "# 关节名称: ";
    for (const auto& joint_name : plan.trajectory_.joint_trajectory.joint_names) {
      logfile << joint_name << " ";
    }
    logfile << std::endl << std::endl;
    
    // 轨迹头部信息
    logfile << "# 轨迹头部信息" << std::endl;
    logfile << "Frame ID: " << plan.trajectory_.joint_trajectory.header.frame_id << std::endl;
    logfile << "时间戳: " << plan.trajectory_.joint_trajectory.header.stamp.sec << "." 
            << std::setfill('0') << std::setw(9) << plan.trajectory_.joint_trajectory.header.stamp.nanosec << std::endl;
    logfile << std::endl;
    
    // 写入每个轨迹点的详细信息
    logfile << "# 轨迹点详细信息" << std::endl;
    logfile << "# 格式: 点索引, 时间(秒), [位置...], [速度...], [加速度...]" << std::endl;
    
    // 遍历所有轨迹点
    for (size_t i = 0; i < plan.trajectory_.joint_trajectory.points.size(); ++i) {
      const auto& point = plan.trajectory_.joint_trajectory.points[i];
      
      // 写入点索引和时间
      logfile << i << ", " << point.time_from_start.sec << "." 
              << std::setfill('0') << std::setw(9) << point.time_from_start.nanosec;
      
      // 写入位置
      logfile << ", [";
      for (size_t j = 0; j < point.positions.size(); ++j) {
        logfile << point.positions[j];
        if (j < point.positions.size() - 1) logfile << ", ";
      }
      logfile << "]";
      
      // 写入速度
      logfile << ", [";
      for (size_t j = 0; j < point.velocities.size(); ++j) {
        logfile << point.velocities[j];
        if (j < point.velocities.size() - 1) logfile << ", ";
      }
      logfile << "]";
      
      // 写入加速度
      logfile << ", [";
      for (size_t j = 0; j < point.accelerations.size(); ++j) {
        logfile << point.accelerations[j];
        if (j < point.accelerations.size() - 1) logfile << ", ";
      }
      logfile << "]";
      
      logfile << std::endl;
    }
    
    logfile.close();
    RCLCPP_INFO(logger_, "成功保存%zu个轨迹点到日志文件", plan.trajectory_.joint_trajectory.points.size());
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<httplib::Server> server_ptr_;
  rclcpp::Logger logger_;
  std::thread server_thread_; // HTTP服务器线程
};

// 注意：我们现在从include目录包含了完整的httplib.h

int main(int argc, char * argv[])
{
  // 初始化ROS
  rclcpp::init(argc, argv);
  
  // 创建ROS节点
  auto const node = std::make_shared<rclcpp::Node>("x1_moveit_rviz_http_service");
  
  // 获取端口参数
  node->declare_parameter("port", 8080);
  int port = node->get_parameter("port").as_int();
  
  // 创建HTTP服务器
  MoveItRvizHttpServer server(node);
  
  // 在单独的线程中运行HTTP服务器
  std::thread http_thread([&server, port]() {
    server.startServer(port);
  });
  
  // 运行ROS节点
  rclcpp::spin(node);
  
  // 清理
  rclcpp::shutdown();
  http_thread.join();
  
  return 0;
}
