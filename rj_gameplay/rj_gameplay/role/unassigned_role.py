import stp.rc
import stp.role

from rj_msgs.msg import RobotIntent


class UnassignedRole(stp.role.Role):
    """Role given to help role assignment correctly handle an unassigned robot"""

    def __init__(self, robot: stp.rc.Robot) -> None:
        """Send debug message to prove it is setting UnassignedRole"""
        super().__init__(robot)
        # if robot.visible:
        #     print(f"UnassignedRole created for {robot}!")

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        return None

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return False
