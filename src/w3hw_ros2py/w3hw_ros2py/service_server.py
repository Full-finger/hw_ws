import rclpy
from rclpy.node import Node
from w3hw_ros2py_interfaces.srv import AddTwoInts


class AddServer(Node):
    def __init__(self):
        super().__init__('add_server')
        # 创建服务，服务名 add_two_ints，收到请求时调用 callback
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.callback)
        self.get_logger().info('加法服务端已启动，等待客户端请求...')

    def callback(self, request, response):
        # 计算加法并返回结果
        response.sum = request.a + request.b
        self.get_logger().info(f'收到请求: {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
