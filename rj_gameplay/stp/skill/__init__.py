""" This module contains data structures for the Skills level of STP.
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Type, TypeVar, Any

import stp.role as role
import stp.rc
from rj_msgs.msg import RobotIntent


class ISkill(ABC):
    pass


class Skill(ABC):
    """Atomic single-robot behavior, such as Move or PivotKick. Created and ticked by Tactics. Uses Actions to get RobotIntents."""

    # TODO: update docstring when ActionClients are up and running

    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]]):
        self.action_client_dict = action_client_dict

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """Logic for Skill goes here. RobotIntents obtained via Actions.

        robot state is updated through super call to this method (i.e. super().tick(world_state))

        .msg
                :param world_state: Current world state.
                :return: A single RobotIntent.
        """
        if self.robot is not None:
            self.robot = world_state.our_robots[self.robot.id]

    @abstractmethod
    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        """True when skill is done; False otherwise."""
        ...
