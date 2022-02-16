from abc import ABC, abstractmethod
from typing import List, Tuple, Any

import stp.rc
import stp.role

from rj_msgs.msg import RobotIntent


# TODO: delete this once all tactics have been switched over
class ITactic(ABC):
    pass


RoleRequests = Any
RoleResults = Any
SkillEntry = Any


class Tactic(ABC):
    """High-level construct that coordinates one or more roles. Creates role requests to be handled by Plays."""

    @abstractmethod
    def __init__(self, world_state: stp.rc.WorldState) -> None:
        """Create empty lists for handling role requests. world_state given on init because
        some Tactics need world_state on init.
        """

        # TODO: make tuple = RoleRequest (or make obj with these two params)?
        self._role_requests: List[Tuple[stp.role.Role, stp.role.CostFn]] = []
        # TODO: make these properties too?
        self.assigned_robots = []
        self.assigned_roles = []

    @abstractmethod
    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        """Given assigned robots by the Play, initialize each role of role_requests with its assigned robot."""
        ...

    @abstractmethod
    def tick(
        self, world_state: stp.rc.WorldState
    ) -> List[Tuple[int, RobotIntent]]:  # (id, intent)
        """Tick each Role of the Tactic to get a list of robot_ids and linked RobotIntents for the Play."""

        # TODO: update self.world_state every tick, like Skills?
        ...

    @abstractmethod
    def is_done(
        self,
        world_state: stp.rc.WorldState,
    ) -> bool:
        """True when Tactic is done; False otherwise."""
        ...

    @property
    def role_requests(self) -> List[Tuple[stp.role.Role, stp.role.CostFn]]:
        """Returns self._role_requests. @property allows the getter to be called like this:

        some_tactic = ConcreteTactic()
        role_reqs = some_tactic.role_requests
        """
        return self._role_requests

    # TODO: use @property setter?
    def set_assigned_robots(self, assigned_robots):
        self.assigned_robots = assigned_robots
