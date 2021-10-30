# TODO: add cancel move option

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rj_msgs.action import Move

"""
send_goal() -> send goal to server -> response -> goal_response_callback() ->
future result -> get_result_callback()
"""
class MoveActionClient(Node):
    def __init__(self):
        super().__init__('move_action_client')
        self._action_client = ActionClient(self, Move, 'move')

    def send_goal(self, PathTargetMotionCommand):
        goal_msg = Move.Goal()
        goal_msg.PathTargetMotionCommand = PathTargetMotionCommand

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future): 
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal not accepted by server.')
            return

        self.get_logger().info('Goal accepted by server!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, ...):
        result = future.result().result
        self.get_logger().info('Result:', result)
        # rclpy.shutdown()

    def feedback_callback(self, feedback_msg): 
        feedback = feedback_msg.feedback
        self.get_logger().info('Got feedback:', feedback)

def main(args=None):
    rclpy.init(args=args)
    action_client = MoveActionClient()
    # action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
