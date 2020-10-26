"""Contains the IBallCarrier interface and related data structures.
"""

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
import stp.action as action
import stp.rc as rc
import stp.skill as skill

import rj_gameplay.eval as eval


class IBallCarrier(skill.ISkill, ABC):
    def do_pass(self, pas: eval.Pass, robot: rc.Robot) -> action.IAction:
        """Returns the action corresponding to
        :param pas: The pass to perform.
        :param robot: The robot to perform the pass.
        :return:
        """
        ...

    def do_shoot(self, shot: eval.Shot, robot: rc.Robot) -> action.IAction:
        """Returns the action for shooting.
        :type shot: The shot to perform.
        :return:
        """
        ...


class BallCarrier(IBallCarrier):
    def define(self):
        pass

    def create_request(self) -> skill.role.RoleRequest:
        pass
