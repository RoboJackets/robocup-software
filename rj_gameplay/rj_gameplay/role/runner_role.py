import numpy as np
import stp
from rj_msgs.msg import RobotIntent

# TODO: settle on unified way to define constants in gameplay
from stp.utils.constants import RobotConstants  # , BallConstants

from rj_gameplay.skill import line_kick, move, pivot_kick, receive

# import stp.global_parameters as global_parameters
# from stp.local_parameters import Param


class RunnerRole(stp.role.Role):
    """Role to produce runner behavior, which tracks the ball, moves to block if a shot on goal is taken, stays within the runner box (generally), and clears ball away."""

    def __init__(self, robot: stp.rc.Robot, brick=False):
        super().__init__(robot)

        self.brick = brick

        self.move_skill = None
        self.next_position = 0
        self.receive_skill = None
        self.pivot_kick_skill = None

    # TODO: there is crusty if-else here, use utility AI or behavior tree or FSM
    #       anything more readable than this (behavior tree prob best fit for current logic)
    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:

        order = [
            world_state.field.our_left_corner,
            world_state.field.their_left_corner,
            world_state.field.their_right_corner,
            world_state.field.our_right_corner,
        ]

        if self.move_skill is None or self.move_skill.is_done(world_state):
            if self.next_position == len(order):
                self.next_position = 0
            next_move = order[self.next_position]
            self.next_position += 1
            self.move_skill = move.Move(
                robot=self.robot,
                target_point=next_move,
                face_point=next_move
        )

        return self.move_skill.tick(world_state)


    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        # runner always active
        # TODO: make role end on capture, let passing role take over
        return False
