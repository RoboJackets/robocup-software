from abc import ABC, abstractmethod
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar, Any

import stp

from rj_msgs.msg import RobotIntent


# TODO: delete this
class ITactic(ABC):
    pass


RoleRequests = Any
RoleResults = Any
SkillEntry = Any


class Tactic(ABC):
    # TODO: add docstring here

    @abstractmethod
    def __init__(self, world_state: stp.rc.WorldState) -> None:
        self.world_state = world_state

        # TODO: add docstring here
        # TODO: make tuple = RoleRequest (or make obj with these two params)
        self._role_requests: List[Tuple[stp.role.Role, stp.role.CostFn]] = []
        # TODO: make these properties too?
        self.assigned_robots = []
        self.assigned_roles = []

    @abstractmethod
    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[Tuple[int, RobotIntent]]:  # (id, intent)
        # TODO: add docstring here
        ...

    @abstractmethod
    def is_done(
        self,
        world_state: stp.rc.WorldState,
    ) -> bool:
        # TODO: add docstring here
        ...

    @abstractmethod
    # TODO: change order of methods so this is above tick
    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        ...

    @property
    def role_requests(self) -> List[Tuple[stp.role.Role, stp.role.CostFn]]:
        return self._role_requests

    # TODO: use @property setter?
    def set_assigned_robots(self, assigned_robots):
        self.assigned_robots = assigned_robots
