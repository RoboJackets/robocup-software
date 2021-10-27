import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rj_msgs.action import Move

class MoveActionClient(Node):
    def __init__(self):
        pass

    def send_goal(self, order):
        pass

    def cancel_goal(self, future):
        # Cancel the goal
        # future = self._goal_handle.cancel_goal_async()
        pass

    def goal_response_callback(self, future):
        pass

    def get_result_callback(self, future):
        pass

    def feedback_callback(self, feedback_msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    action_client = MoveActionClient()
    rclpy.spin(action_client)
    # tactics send goals

if __name__ == '__main__':
    main()
