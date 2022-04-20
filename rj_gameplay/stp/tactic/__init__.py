import itertools
from abc import ABC, abstractmethod
from typing import Any, List, Tuple

from rj_msgs.msg import RobotIntent

import stp.rc
import stp.role


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

    @property
    def needs_assign(self):
        # never needs assign after init
        return False

    def __repr__(self):
        """
        returns a string with all the roles requested and all the roles assigned.
        """
        text = ""
        text += f"{self.__class__.__name__}:\n"
        text += "Roles Requested: "
        if self._role_requests:
            temp = [
                f"({role.__name__}, {cost.__class__.__name__})"
                for cost, role in self._role_requests
                if role != None and cost != None
            ]
            text += ", ".join(temp)
        text += "\nRoles Assigned: "
        if self.assigned_roles:
            text += ", ".join(
                [
                    f"({role.__class__.__name__}, {robot.id})"
                    for role, robot in itertools.zip_longest(
                        self.assigned_roles, self.assigned_robots
                    )
                    if robot != None and role != None
                ]
            )
        return text
