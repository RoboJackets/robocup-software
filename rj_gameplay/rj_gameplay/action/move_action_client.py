# TODO: add cancel move option

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rj_msgs.action import Move

from rj_msgs.msg import RobotIntent, ServerIntent, PathTargetMotionCommand
from rj_geometry_msgs.msg import Point


class MoveActionClient(Node):
    def __init__(self, robot_id):
        super().__init__('move_action_client_{:02d}'.format(robot_id))
        self._robot_id = robot_id
        self._action_client = ActionClient(self, Move, 'move')

    """
    takes in params for path command, returns ServerIntent
    (ServerIntent = RobotIntent + robot_id)
    """

    def generate_server_intent(self,
                               target_pos,
                               target_vel,
                               ignore_ball=False,
                               face_angle=None,
                               face_point=None):
        path_command = PathTargetMotionCommand()
        path_command.target.position = Point(x=target_pos[0], y=target_pos[1])
        path_command.target.velocity = Point(x=target_vel[0], y=target_vel[1])
        path_command.ignore_ball = ignore_ball

        if (face_angle is not None):
            path_command.override_angle = [face_angle]

        if (face_point is not None):
            path_command.override_face_point = [
                Point(x=face_point[0], y=face_point[1])
            ]

        robot_intent = RobotIntent()
        robot_intent.motion_command.path_target_command = [path_command]
        robot_intent.is_active = True

        server_intent = ServerIntent()
        server_intent.intent = robot_intent
        server_intent.robot_id = self._robot_id
        print(self._robot_id)
        return server_intent

    def send_goal(self, intent):
        goal_msg = Move.Goal()
        goal_msg.server_intent = intent

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        print("?" * 80)
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
