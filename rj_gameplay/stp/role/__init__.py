"""This module contains data structures for role assignment."""

from enum import IntEnum
from typing import Optional, Protocol

import stp.rc as rc


class Role:
    """This represents a role, ie. an Optional[rc.Robot]."""

    __slots__ = ["robot"]
    robot: Optional[rc.Robot]

    def __init__(self, robot: Optional[rc.Robot]):
        self.robot = robot

    def is_filled(self) -> bool:
        """Returns true if the role is filled.
        :return: True if the role is filled.
        """
        return self.robot is not None

    def __str__(self) -> str:
        if self.robot is None:
            return "Role(Unassigned)"

        return "Role({})".format(self.robot)

    def __repr__(self) -> str:
        return self.__str__()


class CostFn(Protocol):
    """Protocol for CostFn."""

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:
        """Given a robot, the previous role assignment result, and the current world
        state, returns the cost of the assignment.
        :param robot: The current robot to check costs for.
        :param prev_result: The previous role assignment result.
        :param world_state: The current world state.
        :return:
        """
        ...


class ConstraintFn(Protocol):
    """Protocol for ConstraintFn."""

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> bool:
        """Given a robot, the previous role assignment result, and the current world
        state, returns true if the assignment is valid.
        :param robot: The current robot to check costs for.
        :param prev_result: The previous role assignment result.
        :param world_state: The current world state.
        :return: True if the assignment is valid, false otherwise.
        """
        ...


def unconstrained_constraint_fn(
    robot: rc.Robot, prev_result: Optional["RoleResult"], world_state: rc.WorldState
) -> bool:
    """An unconstrained constraint fn, ie it always returns True.
    :param robot: The current robot to check costs for.
    :param prev_result: The previous role assignment result.
    :param world_state: The current world state.
    :return: True.
    """
    return True

Priority = float

class RoleRequest:
    """Role Request."""

    __slots__ = ["priority", "required", "cost_fn", "constraint_fn"]

    priority: Priority
    required: bool
    cost_fn: CostFn
    constraint_fn: ConstraintFn

    def __init__(
        self,
        priority: Priority,
        required: bool,
        cost_fn: CostFn,
        constraint_fn: ConstraintFn = unconstrained_constraint_fn,
    ):
        self.priority = priority
        self.required = required
        self.cost_fn = cost_fn
        self.constraint_fn = constraint_fn

    def with_priority(self, priority: Priority) -> "RoleRequest":
        """Builder style method that modifies the priority and returns the current
        instance.
        :param priority: The priority to set the RoleRequest to.
        :return: self.
        """
        self.priority = priority
        return self

    def with_required(self, required: bool) -> "RoleRequest":
        """Builder style method that modifies required and returns the current
        instance.
        :param required: Whether the tactic will fail if this RoleRequest is not
        fulfilled.
        :return: self.
        """
        self.required = required
        return self

    def with_cost_fn(self, cost_fn: CostFn) -> "RoleRequest":
        """Builder style method that modifies the cost function and returns the current
        instance.
        :param cost_fn: The new cost function to use.
        :return: self.
        """
        self.cost_fn = cost_fn
        return self

    def with_constraint_fn(self, constraint_fn: ConstraintFn) -> "RoleRequest":
        """Builder style method that modifies the cost function and returns the current
        instance.
        :param constraint_fn: The new constraint function to use.
        :return: self.
        """
        self.constraint_fn = constraint_fn
        return self

    def __str__(self) -> str:
        return "RoleRequest(priority={:>6}, required={})".format(
            self.priority.name, self.required
        )

    def __repr__(self) -> str:
        return self.__str__()


class RoleResult:
    """The result of role assignment."""

    __slots__ = ["request", "cost", "role"]

    request: RoleRequest
    cost: float
    role: Role

    def __init__(
        self,
        request: RoleRequest,
        cost: float,
        role: Role,
    ):
        self.request = request
        self.cost = cost
        self.role = role

    def is_filled(self) -> bool:
        """Returns true if the role request is filled.
        :return: True if the role request is filled.
        """
        return self.role.is_filled()

    def assign(self, robot: rc.Robot, cost: float) -> None:
        """Assigns self.role to the passed in robot, updating self.cost to the
        assignment cost.
        :param robot: Robot to use for the role.
        :param cost: The cost of the assignment.
        """
        self.role.robot = robot
        self.cost = cost

    @classmethod
    def from_request(cls, request: RoleRequest) -> "RoleResult":
        """Creates an unfilled RoleResult from a RoleRequest."""
        return RoleResult(request, 0.0, Role(None))

    def __str__(self) -> str:
        return "RoleResult(priority={:>6}, role={})".format(
            self.request.priority.name, self.role
        )

    def __repr__(self) -> str:
        return self.__str__()
