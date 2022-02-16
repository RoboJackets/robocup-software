from enum import IntEnum
from typing import Optional, Protocol, Type, Dict, List, Any

import stp.rc

from rj_msgs.msg import RobotIntent

from abc import ABC, abstractmethod

BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE = 9999


class Role(ABC):
    """Complex single-robot role, such as Goalie or Striker. Uses Skills to achieve behavior."""

    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]], robot: stp.rc.Robot) -> None:
        """All Roles should apply to one robot's behavior; thus, robot is defined as a formal argument here. Concrete Roles should overwrite init with their own fields, but call super()'s init to use this shared code, like so:

        super().__init__(action_client_dict, robot)
        """
        self.action_client_dict: Dict[Type[Any], List[Any]] = action_client_dict
        self._robot: stp.rc.Robot = robot

    @abstractmethod
    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """Handle behavior of Role by handling which Skill is ticked, and with what params. Return the RobotIntent returned from ticking a Skill."""
        ...

    @abstractmethod
    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        """True if Role is done; False otherwise."""
        ...

    @property
    def robot(self) -> stp.rc.Robot:
        """Returns self._robot. @property allows the getter to be called like this:

        some_role = ConcreteRole()
        some_robot = some_role.robot
        """
        return self._robot


# TODO: delete this when role assignment switched over completely
class Priority(IntEnum):
    """An enum to represent priority of the role assignment."""

    LOW = 0
    MEDIUM = 1
    HIGH = 2
    NUM_PRIORITIES = 3


class CostFn(Protocol):
    """Protocol for CostFn."""

    def __call__(
        self,
        robot: stp.rc.Robot,
        world_state: stp.rc.WorldState,
    ) -> float:
        """Given a robot and the current world state, returns the cost of assigning that robot to a given role.
        :param robot: The current robot to check costs for.
        :param world_state: The current world state.
        :return:
        """
        ...

    def unassigned_cost_fn(
        self, prev_results: Optional["RoleResult"], world_state: stp.rc.WorldState
    ) -> float:
        """Given the previous role assigment and current world state,
        returns the cost of not assigning any robot.
        :param prev_result: The previous role assignment result.
        :param world_state: The current world state.
        :return: cost of not assigning
        """
        ...

    # def switch_cost_fn(
    #     self,
    #     prev_results: Optional["RoleResult"],
    #     world_state: stp.rc.WorldState,
    #     sticky_weight) -> float:
    #     """Given the preevious role assignment and current world state,
    #     returns the cost of switch the role.
    #     :param prev_result: The previous role assignment result.
    #     :param world_state: The current world state.
    #     :sticky_weight:
    #     :return: cost of switching the role already assigned

    #     """


class ConstraintFn(Protocol):
    """Protocol for ConstraintFn."""

    def __call__(
        self,
        robot: stp.rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: stp.rc.WorldState,
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
    robot: stp.rc.Robot,
    prev_result: Optional["RoleResult"],
    world_state: stp.rc.WorldState,
) -> bool:
    """An unconstrained constraint fn, ie it always returns True.
    :param robot: The current robot to check costs for.
    :param prev_result: The previous role assignment result.
    :param world_state: The current world state.
    :return: True.
    """
    return True


# TODO: delete this when role assignment switched over completely
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


# TODO: delete this when role assignment switched over completely
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

    def assign(self, robot: stp.rc.Robot, cost: float) -> None:
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
