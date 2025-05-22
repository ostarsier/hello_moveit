import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from std_msgs.msg import String                  # 字符串消息类型
from joyconrobotics import JoyconRobotics
from x1_moveit_proto.msg import Joycon 

class PublisherNode(Node):

    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.pub = self.create_publisher(Joycon, "joycon_pose", 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        self.timer = self.create_timer(0.01, self.timer_callback)  # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        self.joyconrobotics_right = JoyconRobotics("right")

    def timer_callback(self):                                     # 创建定时器周期执行的回调函数
        pose, gripper, control_button = self.joyconrobotics_right.get_control()
        msg = Joycon()  
        msg.pose = pose
        msg.gripper = gripper
        msg.control_button = control_button
        self.pub.publish(msg)                                     # 发布话题消息
        self.get_logger().info('Publishing: "%s"' % msg)     # 输出日志信息，提示已经完成话题发布

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = PublisherNode("joycon_pose_pub")     # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口