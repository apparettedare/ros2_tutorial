import rclpy
from hello_interfaces.srv import AddTwoInts
from rclpy.node import Node

# サーバー
class MyServer(Node):
    # 初期化
    def __init__(self):
        super().__init__("my_service")

        # サービスの生成
        self.srv = self.create_service(
            AddTwoInts, "add_two_ints", self.listener_callback
        )

    # リクエストの受信時に呼ばれる
    def listener_callback(self, request, response):
        response.sum = request.a + request.b
        return response

# メイン
def main(args=None):
    # ROS通信の初期化
    rclpy.init(args=args)

    # サービスの生成
    server = MyServer()

    # ノード終了まで待機
    rclpy.spin(server)

    # ROS通信のシャットダウン
    rclpy.shutdown()

if __name__ == "__main__":
    main()
