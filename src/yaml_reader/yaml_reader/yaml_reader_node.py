import rclpy
from rclpy.node import Node


class YamlReaderNode(Node):
    def __init__(self):
        super().__init__('yaml_reader_node')

        # 声明所有参数
        self.declare_parameter('frames/odom', '')
        self.declare_parameter('frames/baselink', '')
        self.declare_parameter('frames/lidar', '')
        self.declare_parameter('frames/imu', '')

        # 读取并打印
        self.get_logger().info('========== YAML 参数内容 ==========')

        use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(f'use_sim_time : {use_sim_time}')

        frames = ['frames/odom', 'frames/baselink', 'frames/lidar', 'frames/imu']
        for frame in frames:
            value = self.get_parameter(frame).value
            self.get_logger().info(f'{frame} : {value}')

        self.get_logger().info('====================================')


def main(args=None):
    rclpy.init(args=args)
    node = YamlReaderNode()
    # 打印完即可退出，也可以改成 spin 保持运行
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
