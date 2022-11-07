import stp.rc
import stp.role
from rj_gameplay.skill import move
import numpy as np
from rj_msgs.msg import RobotIntent

class RunnerRole(stp.role.Role):

    def __init__(self, robot: stp.rc.Robot) -> None:
        super().__init__(robot)
        self.current_target = 0
        self.target_points = [np.array([-3.0,9.0]), np.array([-3.0,0.0]), np.array([3.0,0.0]), np.array([3.0,9.0])]
        self.current_move = None

    
    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        if self.current_move is None:
            target_point = self.target_points[self.current_target]
            self.current_move = move.Move(robot=self.robot, target_point=target_point)
        elif self.current_move.is_done():
            self.current_target = (self.current_target + 1) % 4
            target_point = self.target_points[self.current_target]
            self.current_move = move.Move(robot=self.robot, target_point=target_point)

        intent = self.current_move.tick(world_state)
        return intent

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return False
