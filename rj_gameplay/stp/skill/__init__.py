""" This module contains data structures for the Skills level of STP.
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Type, TypeVar

import stp.role as role
import stp.rc
from rj_msgs.msg import RobotIntent


class ISkill(ABC):
    pass


class Skill(ABC):
    """Atomic single-robot behavior, such as Move or PivotKick. Created and ticked by Tactics. Uses Actions to get RobotIntents."""

    # TODO: update docstring when ActionClients are up and running

    @abstractmethod
    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """Logic for Skill goes here. RobotIntents obtained via Actions.
        self.robot is updated forably and quietly.
        .msg
                :param world_state: Current world state.
                :return: A single RobotIntent.
        """
        self.robot = world_state.our_robots[self.robot.id]
        ...

    @abstractmethod
    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        # TODO: docstring
        ...
