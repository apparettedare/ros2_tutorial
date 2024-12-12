import rclpy
import time
from rclpy.action import ActionServer
from rclpy.node import Node
from hello_interfaces.action import AddTwoInts

# サーバー
class MyServer(Node):
    # 初期化
    def __init__(self):
        super().__init__('my_server')

        # サーバーの生成
        self.server = ActionServer(self,
            AddTwoInts, "add_two_ints", self.listener_callback)

    # リクエストの受信時に呼ばれる
    def listener_callback(self, goal_handle):
        # フィードバックの返信
        for i in range(10):
            feedback = AddTwoInts.Feedback()
            feedback.rate = i * 0.1
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)

        # レスポンスの返信
        goal_handle.succeed()
        result = AddTwoInts.Result()
        result.sum = 3
        return result

# メイン
def main(args=None):
    # ROS通信の初期化
    rclpy.init(args=args)

    # サーバーの生成
    server = MyServer()

    # ノード終了の待機
    rclpy.spin(server)

if __name__ == '__main__':
    main()