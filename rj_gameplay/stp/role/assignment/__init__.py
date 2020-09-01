from abc import abstractmethod, ABC
from typing import Dict, Tuple, Type

import stp.tactic as tactic
import stp.role as role
import stp.game_state

RoleId = Tuple[Type[tactic.ITactic], tactic.SkillEntry]

FlatRoleRequests = Dict[RoleId, role.RoleRequest]
FlatRoleResults = Dict[RoleId, role.RoleResult]


class IRoleAssignment(ABC):
    @abstractmethod
    def assign_roles(
        self,
        requests: FlatRoleRequests,
        prev_results: FlatRoleResults,
        game_state: stp.game_state.GameState,
    ) -> FlatRoleResults:
        """ Assigns roles.
        :param requests: The role requests.
        :param prev_results: The previous results.
        :param game_state: The current state of the game.
        :return: The results of the role assignment.
        """
        ...
