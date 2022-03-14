"""This module contains the interface for role assignment."""

from abc import ABC, abstractmethod
from typing import Dict, Any

import stp.rc
import stp.role as role
import stp.tactic as tactic

# RoleId = Tuple[Type[tactic.ITactic], tactic.SkillEntry, int]
# TODO: delete once role assignment switched over entirely
RoleId = Any

FlatRoleRequests = Dict[RoleId, role.RoleRequest]
FlatRoleResults = Dict[RoleId, role.RoleResult]


class IRoleAssignment(ABC):
    """Interface for role assignment implementations."""

    @staticmethod
    @abstractmethod
    def assign_roles(
        flat_requests: FlatRoleRequests,
        world_state: stp.rc.WorldState,
        prev_results: FlatRoleResults,
    ) -> FlatRoleResults:
        """Assigns roles.
        :param flat_requests: The role requests.
        :param world_state: The current state of the world.
        :param prev_results: The previous results.
        :return: The results of the role assignment.
        """
        ...
