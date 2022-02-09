import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rj_msgs.action import Manipulate
from rj_geometry_msgs.msg import Point

from rj_msgs.msg import ServerIntent, RobotIntent


class ManipulateActionClient(Node):
    """The manipulate action client"""
    def __init__(self, robot_id):
        super().__init__("manipulate_action_client_{:02d}".format(robot_id))
        self._robot_id = robot_id
        self._action_client = ActionClient(self, Manipulate, "manipulate")
        self._curr_goal = Manipulate.Goal()
        self._goal_handle = None
        self.curr_feedback = Manipulate.Feedback()

    def send_goal(self, server_intent: ServerIntent):
        goal_msg = Manipulate.Goal()
        goal_msg.server_intent = server_intent
        self._curr_goal = goal_msg

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return

        self._goal_handle = goal_handle

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def cancel_done(self, future):
        cancel_response = future.result()
        self._goal_handle = None
        if len(cancel_response.goals_canceling) > 0:
            # goal successfully canceled
            return cancel_response
        else:
            return cancel_response

    def cancel_goal(self):
        if self._goal_handle is None:
            return
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)
