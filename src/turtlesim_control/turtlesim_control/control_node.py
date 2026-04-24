import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim_control_interfaces.srv import SwitchMode


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # 声明参数
        self.declare_parameter('mode', 0)
        self.declare_parameter('linear_speed', 1.5)
        self.declare_parameter('angular_speed', 0.0)
        self.declare_parameter('sine_linear_speed', 1.0)
        self.declare_parameter('sine_amplitude', 3.0)
        self.declare_parameter('sine_frequency', 0.5)
        self.declare_parameter('square_linear_speed', 1.0)
        self.declare_parameter('square_side_length', 3.0)
        self.declare_parameter('square_turn_speed', 1.5)
        self.declare_parameter('boundary_min', 1.0)
        self.declare_parameter('boundary_max', 10.0)
        self.declare_parameter('boundary_turn_speed', 2.0)

        # 读取参数
        self.load_params()

        # 状态变量
        self.current_pose = None
        self.mode = self.get_parameter('mode').value
        self.paused = False
        self.start_x = 0.0
        self.start_y = 0.0
        self.square_side_count = 0
        self.square_turning = False

        # 发布速度指令
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 订阅位姿
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, QoSProfile(depth=10))

        # 创建服务：模式切换
        self.srv = self.create_service(
            SwitchMode, 'switch_mode', self.switch_mode_callback)

        # 定时器：20Hz 控制循环
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('========================================')
        self.get_logger().info('控制节点已启动')
        self.get_logger().info(f'当前模式: {self.mode_name(self.mode)}')
        self.get_logger().info('服务已就绪: /switch_mode')
        self.get_logger().info('========================================')

    def load_params(self):
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.sine_linear_speed = self.get_parameter('sine_linear_speed').value
        self.sine_amplitude = self.get_parameter('sine_amplitude').value
        self.sine_frequency = self.get_parameter('sine_frequency').value
        self.square_linear_speed = self.get_parameter('square_linear_speed').value
        self.square_side_length = self.get_parameter('square_side_length').value
        self.square_turn_speed = self.get_parameter('square_turn_speed').value
        self.boundary_min = self.get_parameter('boundary_min').value
        self.boundary_max = self.get_parameter('boundary_max').value
        self.boundary_turn_speed = self.get_parameter('boundary_turn_speed').value

    def mode_name(self, mode):
        names = {0: '直线', 1: '正弦', 2: '方波'}
        return names.get(mode, '未知')

    def pose_callback(self, msg):
        self.current_pose = msg

    def switch_mode_callback(self, request, response):
        new_mode = request.mode
        if new_mode in [0, 1, 2]:
            self.mode = new_mode
            self.square_side_count = 0
            self.square_turning = False
            if self.current_pose:
                self.start_x = self.current_pose.x
                self.start_y = self.current_pose.y
            response.success = True
            response.message = f'已切换到模式 {new_mode}: {self.mode_name(new_mode)}'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f'无效模式 {new_mode}，支持: 0=直线, 1=正弦, 2=方波'
            self.get_logger().warn(response.message)
        return response

    def is_at_boundary(self):
        if self.current_pose is None:
            return False
        x = self.current_pose.x
        y = self.current_pose.y
        return (x < self.boundary_min or x > self.boundary_max or
                y < self.boundary_min or y > self.boundary_max)

    def control_loop(self):
        if self.current_pose is None:
            return

        msg = Twist()

        # 边界检测：碰到边界先转向
        if self.is_at_boundary():
            msg.linear.x = 0.5
            msg.angular.z = self.boundary_turn_speed
            self.cmd_vel_pub.publish(msg)
            return

        # 根据模式生成运动
        if self.mode == 0:
            msg = self.mode_straight()
        elif self.mode == 1:
            msg = self.mode_sine()
        elif self.mode == 2:
            msg = self.mode_square()

        self.cmd_vel_pub.publish(msg)

    def mode_straight(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        return msg

    def mode_sine(self):
        msg = Twist()
        msg.linear.x = self.sine_linear_speed
        # 正弦角速度 = A * 2π * f * cos(2π * f * t)
        t = self.get_clock().now().nanoseconds / 1e9
        freq = self.sine_frequency
        amp = self.sine_amplitude
        msg.angular.z = amp * 2.0 * math.pi * freq * math.cos(2.0 * math.pi * freq * t)
        return msg

    def mode_square(self):
        msg = Twist()

        if not self.square_turning:
            # 直行阶段
            dx = self.current_pose.x - self.start_x
            dy = self.current_pose.y - self.start_y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance >= self.square_side_length:
                # 走完一条边，开始转弯
                self.square_turning = True
                self.turn_start_theta = self.current_pose.theta
                self.start_x = self.current_pose.x
                self.start_y = self.current_pose.y
            else:
                msg.linear.x = self.square_linear_speed
                msg.angular.z = 0.0
        else:
            # 转弯阶段：转90度
            d_theta = self.current_pose.theta - self.turn_start_theta
            # 归一化到 [-pi, pi]
            d_theta = math.atan2(math.sin(d_theta), math.cos(d_theta))

            if abs(d_theta) >= math.pi / 2.0:
                # 转完90度，继续直行
                self.square_turning = False
                self.square_side_count += 1
                self.start_x = self.current_pose.x
                self.start_y = self.current_pose.y
            else:
                msg.linear.x = 0.0
                msg.angular.z = self.square_turn_speed

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
