from rj_gameplay import stp
from rj_msgs.msg import RobotIntent
import logging

class UnassignedRole(stp.role.Role):
    """Role given to help role assignment correctly handle an unassigned robot"""

    def __init__(self, robot: stp.rc.Robot) -> None:
        super().__init__(robot)

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """Send debug message to prove it is setting UnassignedRole"""
        print("UnassignedRole ticking!")
        return None

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return False