""" This module contains data structures for the Tactics level of STP.
"""

from abc import ABC, abstractmethod
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar, Any

import stp.action
import stp.rc
import stp.role
import stp.skill
import stp.utils.enum
import stp.utils.typed_key_dict

from rj_msgs.msg import RobotIntent


class ITactic(ABC):
    pass


RoleRequests = Any
RoleResults = Any
SkillEntry = Any


class Tactic(ABC):
    """Complex single-robot role, such as Goalie or Striker. Created and ticked by Plays. Uses Skills to achieve behavior."""

    @abstractmethod
    def __init__(self):
        # TODO: add docstring
        """
        """

    @abstractmethod
    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[Tuple[int, RobotIntent]]:
        """Logic for role goes here. RobotIntents obtained via Skills. (e.g. Goalie can tick Capture to get ball or Intercept to block a shot.)

        :param world_state: Current world state.
        :return: List of (robot_id, robot_intent) tuples.
        """
        ...

    @abstractmethod
    def is_done(
        self,
        world_state: stp.rc.WorldState,
    ) -> bool:
        # TODO: add docstring here
        ...
