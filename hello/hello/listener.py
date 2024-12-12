import rclpy
from hello_interfaces.msg import MyString
from rclpy.node import Node

# サブスクライバー
class MySubscriber(Node):
    # 初期化
    def __init__(self):
        super().__init__("my_subscriber")

        # サブスクライバーの生成
        self.subscription = self.create_subscription(
            MyString, "chatter", self.listener_callback, 10
        )
        self.subscription  # 未使用変数の警告回避

    # メッセージの受信時に呼ばれる
    def listener_callback(self, msg):
        # ログ出力
        self.get_logger().info("Subscribe " + msg.data)

# メイン
def main(args=None):
    # ROS通信の初期化
    rclpy.init(args=args)

    # サブスクライバーの生成
    subscriber = MySubscriber()

    # ノード終了まで待機
    rclpy.spin(subscriber)

    # ノードを明示的に破棄
    subscriber.destroy_node()

    # ROS通信のシャットダウン
    rclpy.shutdown()

if __name__ == "__main__":
    main()
