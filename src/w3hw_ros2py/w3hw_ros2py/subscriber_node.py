#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from w3hw_ros2py_interfaces.srv import AddTwoInts

class HelloWorldSubscriber(Node):
    def __init__(self):
        super().__init__('hello_world_subscriber')

        # 创建订阅者，订阅 "hello_world" 话题
        self.subscription = self.create_subscription(
            String,
            'hello_world',
            self.listener_callback,
            10)

        self.get_logger().info('HelloWorld订阅者已启动，等待接收消息...')

    def listener_callback(self, msg):
        # 当收到消息时，输出接收到的词条
        self.get_logger().info(f'收到消息: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
