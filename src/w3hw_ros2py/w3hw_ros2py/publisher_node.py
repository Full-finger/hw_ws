#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_world_publisher')

        # 创建发布者，发布到 "hello_world" 话题，消息类型为 String
        self.publisher_ = self.create_publisher(String, 'hello_world', 10)

        # 设置发布频率为10Hz（每0.1秒发布一次）
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.counter = 0
        self.get_logger().info('HelloWorld发布者已启动，频率10Hz')

    def timer_callback(self):
        msg = String()
        msg.data = 'helloworld'
        self.publisher_.publish(msg)
        self.counter += 1
        if self.counter % 10 == 0:  # 每10次（1秒）打印一次日志
            self.get_logger().info(f'发布第 {self.counter} 次: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
