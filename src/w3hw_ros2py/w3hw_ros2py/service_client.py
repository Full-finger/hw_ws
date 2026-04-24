import sys
import rclpy
from rclpy.node import Node
from w3hw_ros2py_interfaces.srv import AddTwoInts


class AddClient(Node):
    def __init__(self):
        super().__init__('add_client')
        # 创建客户端
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        # 等待服务端上线
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务端启动...')
        self.get_logger().info('服务端已就绪，发送请求...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        # 异步发送请求
        future = self.client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('用法: ros2 run w3hw_ros2py client <数字a> <数字b>')
        print('示例: ros2 run w3hw_ros2py client 3 5')
        rclpy.shutdown()
        return

    a = int(sys.argv[1])
    b = int(sys.argv[2])

    node = AddClient()
    future = node.send_request(a, b)

    # 等待服务端返回结果
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    node.get_logger().info(f'计算结果: {a} + {b} = {result.sum}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
