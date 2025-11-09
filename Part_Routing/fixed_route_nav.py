import math
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class FixedRouteNav(Node):
    def __init__(self):
        super().__init__('fixed_route_nav')

        # 发布控制指令
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 读取路径点配置文件参数
        self.declare_parameter('waypoints_file', '/home/ubuntu/ros2_ws/src/mosquito_car/waypoints.yaml')
        self.waypoints = self.load_waypoints()
        self.current_index = 0

        # 机器人状态变量
        self.pose = Pose2D()
        self.lookahead_dist = 0.3    # 追踪前瞻距离
        self.linear_speed = 0.15     # 前进速度
        self.angular_gain = 2.0      # 转向灵敏度

        # 订阅里程计（或定位）
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 定时控制循环
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("✅ 固定路线导航节点启动成功！")

    def load_waypoints(self):
        """加载固定路径点"""
        file_path = self.get_parameter('waypoints_file').value
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        self.get_logger().info(f"加载路径点：{len(data['waypoints'])} 个")
        return data['waypoints']

    def odom_callback(self, msg):
        """里程计更新，转换为Pose2D形式"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.pose.x = pos.x
        self.pose.y = pos.y
        self.pose.theta = yaw

    def control_loop(self):
        """路径跟踪控制主循环"""
        if self.current_index >= len(self.waypoints):
            self.stop_robot()
            self.get_logger().info("🎯 所有路径点已到达，导航完成！")
            return

        target = self.waypoints[self.current_index]
        dx = target['x'] - self.pose.x
        dy = target['y'] - self.pose.y
        distance = math.hypot(dx, dy)

        # 判断是否到达当前目标点
        if distance < 0.15:
            self.get_logger().info(f"✅ 到达路径点 {self.current_index + 1}")
            self.current_index += 1
            self.stop_robot()
            return

        # 计算转向角
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.pose.theta)

        # 纯追踪算法控制
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = self.angular_gain * angle_error
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def normalize_angle(angle):
        """角度归一化到 (-pi, pi)"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main():
    rclpy.init()
    node = FixedRouteNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
