""" This module contains data structures for the Plays level of STP.
"""

from abc import ABC, abstractmethod
from collections import defaultdict
from typing import (
    Callable,
    Dict,
    Generic,
    Iterator,
    List,
    Optional,
    Tuple,
    Type,
    TypeVar,
)

import stp.action
import stp.skill
import stp.rc
import stp.role
import stp.role.assignment
import stp.tactic
import stp.utils.enum
import stp.utils.typed_key_dict

from rj_msgs.msg import RobotIntent

PropT = TypeVar("PropT")

# TODO: move to stp.role and delete old RoleRequest definition
# RoleRequest = Tuple[
#     stp.tactic.ITactic, stp.role.CostFn
# ]  # (un-initialized Tactic, associated cost fn)


class IPlay(ABC):
    pass


class Play(ABC):
    """Coordinate full-team behaviors via Tactics. Assumes number of roles matches number of robots on the field. See tick() for more details."""

    def __init__(self):
        # TODO: all three of these are required for assign_roles()
        # should I make these abstract to force subclasses to use?
        # https://stackoverflow.com/questions/23831510/abstract-attribute-not-property
        #
        # or should it just be part of a RoleAssign class with passed in params?
        # that kicks the problem down to forcing roles to have a RoleAssign property, I suppose

        self.prioritized_tactics: List[stp.tactic.Tactic] = []
        self.prioritized_roles: List[stp.role.Role] = []

    @abstractmethod
    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:
        """Performs one "tick" of the specified play.

        This should:
            1. Determine if role assignment is necessary.
            2. If so, perform role assignment with self.assign_roles().
            3. Tick tactics to aggregate robot_intents with self.get_robot_intents().

        :param world_state: Current state of the world.
        :return: list of robot intents where index = robot_id
        """
        ...

    def assign_roles(
        self,
        world_state: stp.rc.WorldState,
    ) -> None:
        """Given that all roles are in sorted order of priority, greedily assign the highest-priority role to the lowest-cost robot for that role. Instantiate tactics with the correct robots post-assignment.
        Satisfy constraint that all roles of a tactic must be assigned together.
        """

        # TODO: use hashable Robots directly once PR #1815 merged
        used_robots = set()
        for tactic in self.prioritized_tactics:
            # TODO: handle if tactic requests more roles than exist
            #       by not assigning tactic at all!
            #       and give some default behavior (later)
            robots_for_tactic = []
            for cost_fn, role in tactic.role_requests:
                min_cost = 1e9
                cheapest_robot = None
                for robot in world_state.our_robots:
                    if robot in used_robots:
                        continue
                    if not robot.visible:
                        continue
                    cost = cost_fn(robot, world_state)
                    if cost < min_cost:
                        min_cost = cost
                        cheapest_robot = robot

                if cheapest_robot is None:
                    # TODO: properly error handle if cheapest_robot is None
                    print(f"RoleRequest ({role}, {cost_fn}) was not assigned")

                used_robots.add(cheapest_robot)
                robots_for_tactic.append(cheapest_robot)
            tactic.set_assigned_robots(robots_for_tactic)

        for tactic in self.prioritized_tactics:
            tactic.init_roles(world_state)

    def get_robot_intents(self, world_state: stp.rc.WorldState) -> List[RobotIntent]:
        # TODO: this constant is from gameplay_node, move to a common gameplay params file
        NUM_ROBOTS = 16
        robot_intents = [None for _ in range(NUM_ROBOTS)]
        for tactic in self.prioritized_tactics:
            role_robot_intents = tactic.tick(world_state)
            for robot_id, robot_intent in role_robot_intents:
                robot_intents[robot_id] = robot_intent
        return robot_intents
