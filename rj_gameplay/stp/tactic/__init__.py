""" This module contains data structures for the Tactics level of STP.
"""

from abc import ABC, abstractmethod
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp.action as action
import stp.rc as rc
import stp.role as role
import stp.skill as skill
import stp.utils.enum as enum
import stp.utils.typed_key_dict as tkdict

RoleRequests = Dict[SkillEntry, List[role.RoleRequest]]
RoleResults = Dict[SkillEntry, List[role.RoleResult]]
PropT = TypeVar("PropT")

class ITactic(Generic[PropT], ABC):
    """The interface class for all tactics."""

    @abstractmethod
    def compute_props(self, prev_props: Optional[PropT]) -> PropT:
        """Computes the props(state) required for the current tick.
        :param prev_props: The props from the previous tick, if available.
        :return: The props for the current tick.
        """
        ...

    @abstractmethod
    def get_requests(self, world_state: rc.WorldState, props: PropT) -> RoleRequests:
        """Returns the RoleRequests for this tactic.
        :param world_state: Current world state.
        :param props: The state of the current tactic.
        :return: RoleRequests for the tactic.
        """
        ...

    @abstractmethod
    def tick(self, role_results: RoleResults,
             props: PropT) -> List[action.IAction]:
        """Ticks the tactic, returning a tuple of the actions and the skills executed.
        :param role_results: The results of role assignment.
        :param props: The state of the current tactic.
        :return: A list of actions to be executed.
        """
        ...
