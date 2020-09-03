from abc import abstractmethod, ABC
from typing import Dict, Tuple, Type

import stp.tactic as tactic
import stp.role as role
import stp.rc

RoleId = Tuple[Type[tactic.ITactic], tactic.SkillEntry]

FlatRoleRequests = Dict[RoleId, role.RoleRequest]
FlatRoleResults = Dict[RoleId, role.RoleResult]


class IRoleAssignment(ABC):
    @staticmethod
    @abstractmethod
    def assign_roles(
        flat_requests: FlatRoleRequests,
        game_state: stp.rc.WorldState,
        prev_results: FlatRoleResults,
    ) -> FlatRoleResults:
        """Assigns roles.
        :param flat_requests: The role requests.
        :param game_state: The current state of the game.
        :param prev_results: The previous results.
        :return: The results of the role assignment.
        """
        ...
