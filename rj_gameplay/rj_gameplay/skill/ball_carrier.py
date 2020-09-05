"""Contains the IBallCarrier interface and related data structures.
"""

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
import stp.rc as rc
import stp.skill as skill


# TODO: Add time information.
class Pass:
    """A representation of a pass."""

    __slots__ = ["passer", "receiver", "pt"]

    passer: rc.RobotId
    receiver: rc.RobotId
    pt: np.ndarray

    def __init__(self, passer: rc.RobotId, receiver: rc.RobotId, pt: np.ndarray):
        """Creates a Pass.
        :param passer: The RobotId of the passing robot.
        :param receiver: The RobotId of the receiving robot.
        :param pt: The point at which the pass will be made to.
        """
        self.passer = passer
        self.receiver = receiver
        self.pt = pt


class IBallCarrier(skill.ISkill, ABC):
    @abstractmethod
    def get_pass(self) -> Optional[Pass]:
        ...


class BallCarrier(IBallCarrier):
    def define(self):
        pass

    def get_pass(self) -> Optional[Pass]:
        ...

    def create_request(self) -> skill.role.RoleRequest:
        pass
