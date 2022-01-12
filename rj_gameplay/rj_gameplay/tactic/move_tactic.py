"""Contains the stub for the move tactic. """

from dataclasses import dataclass
from typing import List, Optional
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import move
import stp.skill as skill
import numpy as np

from rj_msgs.msg import RobotIntent


class MoveTactic(tactic.Tactic):
    def __init__(self, robot: rc.Robot, target_point, face_point):
        self.robot = robot

        self.target_point = target_point
        self.face_point = face_point

        self.move_skill = None

    def tick(self, world_state: rc.WorldState) -> List[Tuple[int, RobotIntent]]:
        # create skill with correct target & face_point
        if self.move_skill is None:
            self.move_skill = move.Move(
                robot=self.robot,
                target_point=self.target_point,
                face_point=self.face_point,
            )

        # tick skill and return
        intent = self.move_skill.tick(world_state)
        return [(self.robot.id, intent)]

    def is_done(self, world_state):
        return self.move_skill.is_done(world_state)
