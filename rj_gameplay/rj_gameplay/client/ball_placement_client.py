from rclpy.action import ActionClient
from rclpy.node import Node
from rj_geometry_msgs.msg import Point
from rj_msgs.action import BallPlacement


class BallPlacementClient(Node):
    """The manipulate action client"""

    def __init__(self):
        super().__init__("ball_placement_client")
        self._action_client = ActionClient(self, BallPlacement, "ball_placement")
        self._goal_handle = None
        self.curr_feedback = None
        self.curr_result = None

    def send_goal(self, ball_placement: BallPlacement):
        goal_msg = BallPlacement.Goal()
        goal_msg.ball_placement = ball_placement

        self._action_client.wait_for_server()

        # send goal and tell the AC what to do when it gets feedback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        # call method once we know if the goal was accepted
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        self._goal_handle = goal_handle

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.curr_result = future.result().result

    def feedback_callback(self, feedback_msg):
        self.curr_feedback = feedback_msg.feedback

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
