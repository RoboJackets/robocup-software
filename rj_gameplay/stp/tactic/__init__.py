from abc import ABC, abstractmethod
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar, Any

import stp.action
import stp.rc
import stp.role
import stp.skill
import stp.utils.enum
import stp.utils.typed_key_dict

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
    def __init__(self, world_state: stp.rc.WorldState):
        self.world_state = world_state

        # TODO: add docstring here
        # TODO: make tuple = RoleRequest (or make obj with these two params)
        self._role_requests: List[Tuple[role.Role, role.CostFn]] = []
        self.assigned_robots = []
        self.assigned_roles = []

    @abstractmethod
    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:
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
    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        ...

    @property
    def role_requests(self):
        return self._role_requests
