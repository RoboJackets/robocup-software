"""This module contains the interface and action for activate the dribbler."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import stp.rc as rc
import rcl.py
from rclpy.action import ActionClient
from rclpy.node import Node
from rj_msgs.action import Activatedribbler
from action_msgs.msg import GoalStatus
from rj_msgs.msg import RobotIntent
from typing import Optional
from rj_msgs import msg

class ActivateDribbler(Node):
	
	def __init__(self):
	super().__init__('ActivateDribbler')
	self._action_client = ActionClient(self, Activatedribbler, 'activatedribbler')
	
	def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.sequence))

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.sequence))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        # Shutdown after receiving a result
        rclpy.shutdown()

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Activatedribbler.Goal()
        goal_msg.dribbler_speed = 1

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)

    action_client = MinimalActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()

class ActivateDribbler(action.IAction): #add ABC if this fails
    """
    Activate dribbler action
    """
    def __init__(self,
            robot_id : int,
            dribbler_speed : float = 1.0,
            priority : int = 0) -> None:

        self.robot_id = robot_id
        self.dribbler_speed = dribbler_speed

    def tick(self, intent: msg.RobotIntent) -> msg.RobotIntent:
        intent.dribbler_speed = self.dribbler_speed
        intent.is_active = True
        return intent

    def is_done(self) -> bool:
        return False
        
   
