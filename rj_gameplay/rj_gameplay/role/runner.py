from typing import List

import math

import numpy as np
import stp
from rj_msgs.msg import RobotIntent
from scipy.optimize import minimize

from rj_gameplay.skill import move


class Runner(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot):
        # TODO: type this header
        print("Runner Initiated")
        super().__init__(robot)
        self.move_skill = None
        self.target_point = [3.0, 0.0]
        self._target_point = None
        self.indexOn = 0
        self.points = [[3.0, 0.0], [3.0, 9.0], [-3.0, 9.0], [-3.0, 0.0]]
        self._ticks_since_reassign = 0

    def get_next_corner(self, world_state):
        self.indexOn += 1
        self.indexOn = self.indexOn % 4
        return self.points[self.indexOn]


    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """
        Assume our_robot has ball on init. Then:
         - on init: get every point in the OFFENSE section of field away from their_robots at a certain distance and move there

        """

        # only reassign every so often so robot can reach target pt
        #Figure out how to have robot location

        threshold = 0.3
        if (
            math.sqrt(
                (world_state.our_robots[self.robot.id].pose[0] - self.target_point[0])
                ** 2
                + (world_state.our_robots[self.robot.id].pose[1] - self.target_point[1])
                ** 2
            )
            < threshold
        ):   
            self.target_point = self.get_next_corner(
                world_state
            )
            self._ticks_since_reassign = 0

        self._ticks_since_reassign += 1

        # assign move skill
        print(self.target_point)
        print(self.target_point[1])
        self.move_skill = move.Move(
            robot=self.robot,
            target_point=self.target_point,
            face_point=world_state.ball.pos,
        )

        intent = self.move_skill.tick(world_state)

        return intent

    def is_done(self, world_state) -> bool:
        return self.move_skill.is_done(world_state)
