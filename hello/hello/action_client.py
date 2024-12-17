import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from hello_interfaces.action import AddTwoInts

# クライアント
class MyClient(Node):
    # 初期化
    def __init__(self):
        super().__init__("my_client")
        self.client = ActionClient(self, AddTwoInts, 'add_two_ints')

    # リクエストの送信
    def send_request(self):
        # メッセージの生成
        self.a = 14
        self.b = 2
        goal = AddTwoInts.Goal()
        goal.a = self.a
        goal.b = self.b

        # サーバー接続の待機
        self.client.wait_for_server()

        # リクエストの送信
        self.future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.response_callback)

    # フィードバックの受信時に呼ばれる
    def feedback_callback(self, feedback):
        print("feedback :", feedback.feedback.rate)

    # レスポンスの受信時に呼ばれる
    def response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
           return

        # 結果の取得
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    # 結果の受信時に呼ばれる
    def result_callback(self, future):
        result = future.result().result
        print("%s + %s = %s"%(self.a, self.b, result.sum))
        rclpy.shutdown()

# メイン
def main(args=None):
    # ROS通信の初期化
    rclpy.init(args=args)

    # クライアントの生成
    client = MyClient()

    # リクエストの送信
    client.send_request()

    # ノード終了の待機
    rclpy.spin(client)

if __name__ == '__main__':
    main()