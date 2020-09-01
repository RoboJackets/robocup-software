from enum import IntEnum
from typing import Callable, Optional

import stp.game_state as game_state


class Role:
    __slots__ = ["robot"]
    robot: Optional[game_state.Robot]

    def __init__(self, robot: Optional[game_state.Robot]):
        self.robot = robot

    def is_filled(self) -> bool:
        return self.robot is not None

    def __str__(self) -> str:
        if self.robot is None:
            return "Role(Unassigned)"

        return "Role({})".format(self.robot)

    def __repr__(self) -> str:
        return self.__str__()


class Priority(IntEnum):
    LOW = 0
    MEDIUM = 1
    HIGH = 2
    NUM_PRIORITIES = 3


# (robot: Robot, prev_robot: Robot) -> cost.
CostFn = Callable[[game_state.Robot, Optional[game_state.Robot]], float]


class RoleRequest:
    """Role Request."""

    __slots__ = ["priority", "required", "cost_fn"]

    priority: Priority
    required: bool
    cost_fn: CostFn

    def __init__(self, priority: Priority, required: bool, cost_fn: CostFn):
        self.priority = priority
        self.required = required
        self.cost_fn = cost_fn

    def with_priority(self, priority: Priority) -> "RoleRequest":
        self.priority = priority
        return self

    def with_required(self, required: bool) -> "RoleRequest":
        self.required = required
        return self

    def with_cost_fn(self, cost_fn: CostFn) -> "RoleRequest":
        self.cost_fn = cost_fn
        return self

    def __str__(self) -> str:
        return "RoleRequest(priority={:>6}, required={})".format(
            self.priority.name, self.required
        )

    def __repr__(self) -> str:
        return self.__str__()


class RoleResult:
    """The result of role assignment."""

    __slots__ = ["priority", "required", "cost_fn", "cost", "role"]

    priority: Priority
    required: bool
    cost_fn: CostFn
    cost: float
    role: Role

    def __init__(
        self,
        priority: Priority,
        required: bool,
        cost_fn: CostFn,
        cost: float,
        role: Role,
    ):
        self.priority = priority
        self.required = required
        self.cost_fn = cost_fn
        self.cost = cost
        self.role = role

    def is_filled(self) -> bool:
        """Returns true if the role request is filled.
        :return: True if the role request is filled.
        """
        return self.role.is_filled()

    def assign(self, robot: game_state.Robot, cost: float) -> None:
        """Assigns self.role to the passed in robot, updating self.cost to the
        assignment cost.
        :param robot: Robot to use for the role.
        :param cost: The cost of the assignment.
        """
        self.role.robot = robot
        self.cost = cost

    @classmethod
    def from_request(cls, request: RoleRequest) -> "RoleResult":
        return RoleResult(
            request.priority, request.required, request.cost_fn, 0.0, Role(None)
        )

    def __str__(self) -> str:
        return "RoleResult(priority={:>6}, role={})".format(
            self.priority.name, self.role
        )

    def __repr__(self) -> str:
        return self.__str__()
