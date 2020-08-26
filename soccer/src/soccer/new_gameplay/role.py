from typing import List, Set, Optional, Tuple, Type, TypeVar, Callable
from enum import Enum, auto
from abc import abstractmethod, ABC
import robocup

class Role:
    def __init__(self) -> None:
        self.robot = None

    def assign_robot(self, robot: robocup.OurRobot) -> None:
        self.robot = robot 

class RoleRequestPriority(Enum):
    HIGH = 1
    MEDIUM = 2
    LOW = 3

class RoleRequest:
    def __init__(self, cost: Callable[robocup.OurRobot,float], constraints: Callable[robocup.OurRobot, bool], priority: RoleRequestPriority, last_role: Optional[Role] = None) -> None:
        self.cost = cost
        self.constraints = constraints
        self.last_role = last_role
        self.role = Role()
        self.priority = priority

    def is_assigned(self) -> bool:
        return self.role.robot is not None

    def get_role(self) -> Role:
        return self.role

    def assign(self, robot) -> None:
        self.role.assign_robot(robot)

    def unassign(self) -> None:
        self.role.assign_robot(None)

    def get_robot(self) -> robocup.OurRobot:
        return self.role.robot

    def get_priority(self) -> RoleRequestPriority:
        return self.priority

    def set_last_role(self) -> None:
        self.last_role = self.role