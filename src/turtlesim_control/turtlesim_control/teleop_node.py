import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim_control_interfaces.srv import SwitchMode


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        # 发布速度指令
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 创建客户端：切换模式
        self.client = self.create_client(SwitchMode, 'switch_mode')

        # 速度参数
        self.linear_speed = 1.0
        self.angular_speed = 1.5

        self.print_help()

    def print_help(self):
        self.get_logger().info('========================================')
        self.get_logger().info('遥控器已启动')
        self.get_logger().info('========================================')
        self.get_logger().info('  W : 前进')
        self.get_logger().info('  S : 后退')
        self.get_logger().info('  A : 左转')
        self.get_logger().info('  D : 右转')
        self.get_logger().info('  X : 停止')
        self.get_logger().info('  1 : 切换到正弦模式')
        self.get_logger().info('  2 : 切换到方波模式')
        self.get_logger().info('  0 : 切换到直线模式')
        self.get_logger().info('  Q : 退出')
        self.get_logger().info('========================================')

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def switch_mode(self, mode):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('服务 /switch_mode 未就绪')
            return

        request = SwitchMode.Request()
        request.mode = mode
        future = self.client.call_async(request)
        future.add_done_callback(self.switch_mode_done)

    def switch_mode_done(self, future):
        result = future.result()
        if result.success:
            self.get_logger().info(result.message)
        else:
            self.get_logger().warn(result.message)

    def run(self):
        while rclpy.ok():
            key = self.get_key().lower()

            if key == 'w':
                self.send_cmd(self.linear_speed, 0.0)
                self.get_logger().info('前进')
            elif key == 's':
                self.send_cmd(-self.linear_speed, 0.0)
                self.get_logger().info('后退')
            elif key == 'a':
                self.send_cmd(0.0, self.angular_speed)
                self.get_logger().info('左转')
            elif key == 'd':
                self.send_cmd(0.0, -self.angular_speed)
                self.get_logger().info('右转')
            elif key == 'x':
                self.send_cmd(0.0, 0.0)
                self.get_logger().info('停止')
            elif key == '0':
                self.switch_mode(0)
                self.get_logger().info('请求切换到直线模式')
            elif key == '1':
                self.switch_mode(1)
                self.get_logger().info('请求切换到正弦模式')
            elif key == '2':
                self.switch_mode(2)
                self.get_logger().info('请求切换到方波模式')
            elif key == 'q':
                self.send_cmd(0.0, 0.0)
                self.get_logger().info('退出遥控器')
                break
            else:
                self.send_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
