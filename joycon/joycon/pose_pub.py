import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from joyconrobotics import JoyconRobotics
from x1_moveit_proto.srv import JoyconCommand     # 导入服务消息
import time

class ServiceClientNode(Node):

    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.cli = self.create_client(JoyconCommand, "joycon_command")   # 创建服务客户端
        glimit = [[-0.2, -0.2,  -0.006, -3.1, -1.5, -1.5], 
                       [0.2,  0.5,  0.3,  3.1,  1.5,  1.5]]
        self.joyconrobotics_right = JoyconRobotics(
                    device="right", 
                    horizontal_stick_mode='yaw_diff',  # 水平摇杆模式
                    close_y=True,  # 关闭Y轴控制
                    limit_dof=True,  # 限制自由度
                    offset_position_m=[0.002, -0.199, -0.006],  # 位置偏移量(初始位姿)
                    glimit=glimit,  # 全局限位
                    dof_speed=[1.0, 1.0, 1.0, 1.0, 1.0, 0.5],  # 各自由度的速度系数
                    common_rad=False,  # 不使用通用弧度
                    lerobot=False,  # 使用LeRobot模式
                    pitch_down_double=True  # 俯仰向下双倍速度
                )
        self.get_logger().info('等待服务器可用...')
        
        # 等待服务可用
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务器可用...')
        self.get_logger().info('服务器已就绪')

    def send_request(self, pose, gripper, control_button):
        request = JoyconCommand.Request()
        request.pose = pose
        request.gripper = gripper
        request.control_button = control_button
        
        # 同步调用服务
        future = self.cli.call_async(request)
        return future
        
    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'响应: {response.success}, 消息: {response.message}')
            return response
        except Exception as e:
            self.get_logger().error(f'调用服务失败: {str(e)}')
            return None

    def run(self):
        # 在循环中调用服务
        try:
            while rclpy.ok():
                # 获取Joycon控制数据
                pose, gripper, control_button = self.joyconrobotics_right.get_control()
                
                # 发送服务请求
                future = self.send_request(pose, gripper, control_button)
                
                # 等待服务返回并处理响应
                rclpy.spin_until_future_complete(self, future)
                self.handle_response(future)
                
                # 等待0.1秒
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.get_logger().info('用户中断了服务客户端')

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ServiceClientNode("joycon_command_client")  # 创建ROS2节点对象并进行初始化
    node.run()                                        # 运行客户端主循环
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口