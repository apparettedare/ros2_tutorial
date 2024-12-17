import rclpy
import time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from hello_interfaces.action import AddTwoInts

# サーバー
class MyServer(Node):
    # 初期化
    def __init__(self):
        super().__init__('my_server')

        # サーバーの生成
        self.server = ActionServer(
            self,
            AddTwoInts,
            "add_two_ints",
            execute_callback=self.listener_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    # リクエストの受信時に呼ばれる
    async def listener_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback = AddTwoInts.Feedback()
        # フィードバックの返信
        for i in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return AddTwoInts.Result()
            feedback.rate = i * 0.1
            self.get_logger().info('Publishing feedback: {0}'.format(feedback.rate))
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)

        # レスポンスの返信
        goal_handle.succeed()
        result = AddTwoInts.Result()
        result.sum = goal_handle.request.a + goal_handle.request.b
        self.get_logger().info('Returning result: {0}'.format(result.sum))

        return result

# メイン
def main(args=None):
    try:
        # ROS通信の初期化
        rclpy.init(args=args)
        # サーバーの生成
        server = MyServer()

        # Use a MultiThreadedExecutor to enable processing goals concurrently
        executor = MultiThreadedExecutor()

        rclpy.spin(server, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()