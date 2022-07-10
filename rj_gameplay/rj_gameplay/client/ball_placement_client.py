from rclpy.action import ActionClient
from rclpy.node import Node
from rj_geometry_msgs.msg import Point
from rj_msgs.action import BallPlacement


class BallPlacementClient(Node):
    """The ball placement action client"""

    def __init__(self):
        super().__init__("ball_placement_client")
        self._action_client = ActionClient(self, BallPlacement, "ball_placement")
        self._goal_handle = None
        self.curr_feedback = None
        self.curr_result = None

        # TODO: save curr goal and do not resend if the points are the same
        self.curr_goal_pt = None

    def send_goal(self, goal_pt: Point):
        goal_msg = BallPlacement.Goal()
        goal_msg.goal_pt = goal_pt

        # save goal_pt so we can check if it's a duplicate
        if self.curr_goal_pt is None:
            self.curr_goal_pt = goal_pt

        self._action_client.wait_for_server()

        # only send if the goal has changed
        seen_goal_before = (
            self.curr_goal_pt is not None
            and self.curr_goal_pt.x == goal_pt.x
            and self.curr_goal_pt.y == goal_pt.y
        )
        if not seen_goal_before:
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
        # TODO: clear state of goal pt when done
        # self.curr_goal_pt = None

    def feedback_callback(self, feedback_msg):
        self.curr_feedback = feedback_msg.feedback

    def cancel_done(self, future):
        cancel_response = future.result()
        self._goal_handle = None
        if len(cancel_response.goals_canceling) > 0:
            # goal successfully canceled
            self.curr_goal_pt = None
            return cancel_response
        else:
            return cancel_response

    def cancel_goal(self):
        if self._goal_handle is None:
            return
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)
        self.curr_goal_pt = None
