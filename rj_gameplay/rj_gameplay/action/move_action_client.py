# TODO: add cancel move option

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rj_msgs.action import Move

from rj_msgs.msg import ServerIntent


class MoveActionClient(Node):
    def __init__(self, robot_id):
        super().__init__('move_action_client_{:02d}'.format(robot_id))
        self._robot_id = robot_id
        self._action_client = ActionClient(self, Move, 'move')

    def send_goal(self, intent: ServerIntent):
        goal_msg = Move.Goal()
        goal_msg.server_intent = intent

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal not accepted by server.')
            return

        self.get_logger().info('Goal accepted by server!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result:', result)
        # rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Got feedback:', feedback)
