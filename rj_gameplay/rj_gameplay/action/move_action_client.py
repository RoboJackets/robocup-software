import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rj_msgs.action import Move
from rj_geometry_msgs.msg import Point

from rj_msgs.msg import ServerIntent, RobotIntent, PathTargetMotionCommand


class MoveActionClient(Node):
    """The move action client"""
    def __init__(self, robot_id):
        super().__init__("move_action_client_{:02d}".format(robot_id))
        self._robot_id = robot_id
        self._action_client = ActionClient(self, Move, "move")
        self._curr_goal = Move.Goal()
        self._goal_handle = None
        self.curr_feedback = Move.Feedback()

    def send_goal(self, server_intent: ServerIntent):
        # TODO: check all motion commands, what is the best way?
        if len(server_intent.intent.motion_command.path_target_command) > 0:
            new_target_position: Point = (
                server_intent.intent.motion_command.path_target_command[
                    0
                ].target.position
            )
            if self._curr_goal is not None:
                path_target_command = (
                    self._curr_goal.server_intent.intent.motion_command.path_target_command
                )
                if (
                    len(path_target_command) > 0
                    and path_target_command[0].target.position == new_target_position
                ):
                    return

        goal_msg = Move.Goal()
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
            return cancel_response
        else:
            return cancel_response

    def cancel_goal(self):
        if self._goal_handle is None:
            return
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)
