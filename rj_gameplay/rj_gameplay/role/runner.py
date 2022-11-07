import numpy as np
import stp
from rj_msgs.msg import RobotIntent

from rj_gameplay.skill import line_kick, move, pivot_kick, receive

class RunnerRole(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot):
        super().__init__(robot)

        self.move_skill = None
        self.receive_skill = None
        self.pivot_kick_skill = None
        self.counter = 0
    
    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        our_left_corner = world_state.field.our_left_corner
        our_right_corner = world_state.field.our_right_corner
        their_left_corner = world_state.field.their_left_corner
        their_right_corner = world_state.field.their_right_corner

        corners = [our_left_corner, our_right_corner, their_right_corner, their_left_corner]

        # run to our left corner first as the starting point
        if self.move_skill is None:
            self.move_skill = move.Move(robot=self.robot, target_point=our_left_corner)     
        elif self.is_done(world_state):
            self.counter += 1
            index = self.counter % 4
            self.move_skill = move.Move(robot=self.robot, target_point=corners[index])

        return self.move_skill.tick(world_state)

    def is_done(self, world_state) -> bool:
        if self.move_skill is not None:
            return self.move_skill.is_done(world_state)



