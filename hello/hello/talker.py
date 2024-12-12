import rclpy
from hello_interfaces.msg import MyString
from rclpy.node import Node

# パブリッシャー
class MyPublisher(Node):
    # 初期化
    def __init__(self):
        super().__init__("my_publisher")

        # パブリッシャーの生成
        self.publisher_ = self.create_publisher(MyString, "chatter", 10)

        # タイマーの生成
        self.timer = self.create_timer(1, self.timer_callback)

    # 1秒毎に呼ばれる
    def timer_callback(self):
        # メッセージの生成
        msg = MyString()
        msg.data = "Hello World!"

        # メッセージの送信
        self.publisher_.publish(msg)

        # ログ出力
        self.get_logger().info("Publish " + msg.data)

# メイン
def main(args=None):
    # ROS通信の生成
    rclpy.init(args=args)

    # パブリッシャーの生成
    publisher = MyPublisher()

    # ノード終了まで待機
    rclpy.spin(publisher)

    # ノードを明示的に破棄
    publisher.destroy_node()

    # ROS通信のシャットダウン
    rclpy.shutdown()

if __name__ == "__main__":
    main()