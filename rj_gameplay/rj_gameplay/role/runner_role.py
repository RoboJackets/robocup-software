import stp.role
import stp.rc
from rj_msgs.msg import RobotIntent
from rj_gameplay.skill import move
import numpy as np

class RunnerRole(stp.role.Role):
    """"""

    def __init__(self, robot: stp.rc.Robot) -> None:
        self.curr = 0
        super().__init__(robot)

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        corners = [world_state.field.our_left_corner, world_state.field.our_right_corner, world_state.field.their_right_corner, world_state.field.their_left_corner]
        pos = np.array([world_state.robots[self.robot.id].pose[0], world_state.robots[self.robot.id].pose[1]])
        if (np.linalg.norm(pos-corners[self.curr]) < 0.1):
            self.curr = (self.curr + 1) % len(corners)
        return move.Move(target_point=corners[self.curr]).tick(world_state)

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return False