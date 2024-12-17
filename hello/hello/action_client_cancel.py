import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from hello_interfaces.action import AddTwoInts

# クライアント
class MyClient(Node):
    # 初期化
    def __init__(self):
        super().__init__("my_client")
        self.client = ActionClient(self, AddTwoInts, 'add_two_ints')

    # 結果の受信時に呼ばれる
    def result_callback(self, future):
        result = future.result().result
        print("%s + %s = %s"%(self.a, self.b, result.sum))
        rclpy.shutdown()

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        rclpy.shutdown()

    # レスポンスの受信時に呼ばれる
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self._goal_handle = goal_handle

        self.get_logger().info('Goal accepted :)')

        # Start a 2 second timer
        self._timer = self.create_timer(2.0, self.timer_callback)

        # 結果の取得
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)
        
    # フィードバックの受信時に呼ばれる
    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.rate))

    def timer_callback(self):
        self.get_logger().info('Canceling goal')
        # Cancel the goal
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

        # Cancel the timer
        self._timer.cancel()

    # リクエストの送信
    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        # サーバー接続の待機
        self.client.wait_for_server()
        # メッセージの生成
        self.a = 14
        self.b = 2
        goal = AddTwoInts.Goal()
        goal.a = self.a
        goal.b = self.b

        self.get_logger().info('Sending goal request...')

        # リクエストの送信
        self._send_goal_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

# メイン
def main(args=None):

    try:
        # ROS通信の初期化
        rclpy.init(args=args)

        # クライアントの生成
        client = MyClient()

        # リクエストの送信
        client.send_goal()

        # ノード終了の待機
        rclpy.spin(client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
if __name__ == '__main__':
    main()