import rclpy
from hello_interfaces.srv import AddTwoInts
from rclpy.node import Node

# クライアント
class MyClient(Node):
    # 初期化
    def __init__(self):
        super().__init__("my_client")

        # クライアントの生成
        self.cli = self.create_client(AddTwoInts, "add_two_ints")

        # サーバー接続まで待機
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available...")

        # リクエストの生成
        self.req = AddTwoInts.Request()

    # リクエストの送信
    def send_request(self):
        self.req.a = 1
        self.req.b = 2
        self.future = self.cli.call_async(self.req)

# メイン
def main(args=None):
    # ROS通信の初期化
    rclpy.init(args=args)

    # クライアントの生成
    client = MyClient()

    # リクエストの送信
    client.send_request()

    # レスポンスの受信
    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
                
                # ログ出力
                client.get_logger().info(
                    "%d + %d = %d"
                    % (client.req.a, client.req.b, response.sum)
                )
            except Exception as e:
                client.get_logger().info("Error: %r" % (e,))
            break

    # ノードを明示的に破棄
    client.destroy_node()

    # ROS通信のシャットダウン
    rclpy.shutdown()

if __name__ == "__main__":
    main()