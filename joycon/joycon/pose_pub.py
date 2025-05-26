import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from joyconrobotics import JoyconRobotics
from x1_moveit_proto.srv import IkSolve     # 导入服务消息
import time
from sensor_msgs.msg import JointState
import redis
import json
# 注释掉不再使用的MoveIt导入
# from moveit_commander import MoveGroupCommander, RobotCommander

class ServiceClientNode(Node):

    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        # 创建IK求解服务客户端
        self.ik_cli = self.create_client(IkSolve, "ik_solve")
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
        
        # 初始化Redis客户端
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        
        # 定义目标关节顺序
        self.target_joints = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
                         "right_elbow_pitch_joint", "right_elbow_yaw_joint",
                         "right_wrist_pitch_joint", "right_wrist_roll_joint"]
        
        # 等待服务可用
        while not self.ik_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务器可用...')
        self.get_logger().info('服务器已就绪')

    def send_ik_request(self, pose):
        """发送IK求解请求，获取关节角度（同步调用）"""
        # 创建请求对象
        request = IkSolve.Request()
        
        # 只取前三个元素（x, y, z）
        request.x = float(pose[0])  # 取位置x坐标
        request.y = float(pose[1])  # 取位置y坐标
        request.z = float(pose[2])  # 取位置z坐标
        
        self.get_logger().info(f'发送IK请求: x={request.x}, y={request.y}, z={request.z}')
        
        try:
            # 确保服务可用（使用较短的超时时间）
            if not self.ik_cli.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn('服务不可用，跳过这次请求')
                return None
            
            # 同步调用IK求解服务（设置超时时间）
            future = self.ik_cli.call_async(request)
            
            # 等待响应，但最多等待1秒
            timeout_sec = 1.0
            start_time = time.time()
            
            while not future.done() and (time.time() - start_time) < timeout_sec:
                # 处理ROS回调
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if future.done():
                response = future.result()
                self.get_logger().info(f'IK求解成功获取响应')
                return response
            else:
                self.get_logger().warn(f'IK求解超时')
                return None
                
        except Exception as e:
            self.get_logger().error(f'IK求解调用异常: {str(e)}')
            return None
    
    def handle_ik_response(self, response):
        """处理IK求解服务的响应并将关节角度存储到Redis中"""
        if response is None:
            self.get_logger().warn('收到空的IK响应')
            return None
            
        if response.success:
            self.get_logger().info(f'IK求解成功')
            
            # 创建关节值字典，将响应的关节值按指定顺序存储
            joint_values_dict = {}
            for i, name in enumerate(response.joint_names):
                joint_values_dict[name] = response.joint_values[i]
                self.get_logger().info(f'  {name}: {response.joint_values[i]}')
            
            # 按照目标关节顺序重新组织关节值
            ordered_joint_values = []
            for joint_name in self.target_joints:
                # 如果有对应的关节值就添加，否则添加0（默认值）
                if joint_name in joint_values_dict:
                    ordered_joint_values.append(joint_values_dict[joint_name])
                else:
                    ordered_joint_values.append(0.0)
                    self.get_logger().warn(f'缺少关节 {joint_name} 的值，使用默认值0')
            
            # 将关节值列表存入Redis
            try:
                self.redis_client.set('joint_position', json.dumps(ordered_joint_values))
                self.get_logger().info(f'成功将关节位置存储到Redis: {ordered_joint_values}')
            except Exception as e:
                self.get_logger().error(f'存储到Redis失败: {str(e)}')
            
            return response
        else:
            self.get_logger().warn(f'IK求解失败: {response.message}')
            return None
    
    # 不再需要此方法，已由Redis替代
    # def move_to_joint_position(self, joint_angles):
    #     # 设置目标关节角度
    #     self.move_group.set_joint_value_target(joint_angles)
    #     
    #     # 规划并执行运动
    #     plan = self.move_group.go(wait=True)
    #
    #     # 清除目标
    #     self.move_group.stop()
    #     self.move_group.clear_pose_targets()
    #
    #     return plan


    def run(self):
        # 在循环中调用服务
        try:
            # 设置请求频率限制
            rate_limit = 0.2  # 每200ms发送一次请求
            last_request_time = time.time()
            
            while rclpy.ok():
                current_time = time.time()
                
                # 限制请求频率
                if current_time - last_request_time >= rate_limit:
                    # 获取Joycon控制数据
                    pose, gripper, control_button = self.joyconrobotics_right.get_control()
                    
                    try:
                        # 发送IK求解服务请求
                        response = self.send_ik_request(pose)
                        
                        # 如果有响应，处理响应
                        if response is not None:
                            self.handle_ik_response(response)
                        
                        # 更新上次请求时间
                        last_request_time = current_time
                    except Exception as e:
                        self.get_logger().error(f'调用服务时发生错误: {str(e)}')
                
                # 处理ROS回调，保持节点活跃
                rclpy.spin_once(self, timeout_sec=0.01)
                
                # 短暂休眠，减少CPU使用
                time.sleep(0.01)
        except KeyboardInterrupt:
            self.get_logger().info('用户中断了服务客户端')

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ServiceClientNode("joycon_command_client")  # 创建ROS2节点对象并进行初始化
    node.run()                                        # 运行客户端主循环
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口