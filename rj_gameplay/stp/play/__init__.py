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
from rj_gameplay.role import unassigned_role
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
    """Coordinate full-team behaviors via Tactics. Assumes number of Roles matches number of robots on the field.
    Ends when SituationAnalysis switches the Play, so no is_done() necessary.
    See tick() for more details on behavior.
    """

    def __init__(self):
        # TODO: all three of these are required for assign_roles()
        # should I make these abstract to force subclasses to use?
        # https://stackoverflow.com/questions/23831510/abstract-attribute-not-property
        #
        # or should it just be part of a RoleAssign class with passed in params?
        # that kicks the problem down to forcing roles to have a RoleAssign property, I suppose

        self.prioritized_tactics: List[stp.tactic.Tactic] = []
        self.prioritized_roles: List[stp.role.Role] = []
        self.unassigned_roles: List[UnassignedRole] = []

    @abstractmethod
    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        """Performs one "tick" of the specified play.

        This should:
            1. Determine if role assignment is necessary.
            2. If so, perform role assignment with self.assign_roles().
            3. Tick Tactics to aggregate robot_intents with self.get_robot_intents().

        :param world_state: Current state of the world.
        :return: list of robot intents where index = robot_id
        """
        ...

    def assign_roles(
        self,
        world_state: stp.rc.WorldState,
    ) -> None:
        """Given that all Roles are in sorted order of priority, greedily assign the highest-priority Role to the lowest-cost robot for that Role. Instantiate Tactics with the correct robots post-assignment. (Note that this behavior is largely handled by the init_roles of each Tactic.)
        Satisfy constraint that all Roles of a Tactic must all be assigned at once. If a Tactic's Roles cannot all be filled, do not fill any of its Roles.
        """

        used_robots = set()
        for tactic in self.prioritized_tactics:
            robots_for_tactic = []
            for cost_fn, role in tactic.role_requests:
                min_cost = 1e9
                cheapest_robot = None
                for robot in world_state.our_robots:
                    if robot in used_robots or robot in robots_for_tactic:
                        continue
                    if not robot.visible:
                        continue
                    cost = cost_fn(robot, world_state)
                    if cost < min_cost:
                        min_cost = cost
                        cheapest_robot = robot

                if cheapest_robot is None:
                    print(f"RoleRequest ({role}, {cost_fn}) was not assigned")
                    robots_for_tactic = None
                    break
                robots_for_tactic.append(cheapest_robot)

            if robots_for_tactic is not None:
                used_robots.update(robots_for_tactic)
                tactic.set_assigned_robots(robots_for_tactic)

        for tactic in self.prioritized_tactics:
            tactic.init_roles(world_state)
        for robot in world_state.our_robots:
            if robot not in used_robots:
                self.unassigned_roles.append(unassigned_role.UnassignedRole(robot))

    def get_robot_intents(self, world_state: stp.rc.WorldState) -> List[RobotIntent]:
        """Tick each tactic to get a list of RobotIntents for GameplayNode. Each RobotIntent in this list is at index robot_id, or in Python terms: return_list[robot_id] = robot_intent"""
        # TODO: this constant is from gameplay_node, move to a common gameplay params file
        NUM_ROBOTS = 16
        robot_intents = [None for _ in range(NUM_ROBOTS)]
        for tactic in self.prioritized_tactics:
            role_robot_intents = tactic.tick(world_state)
            for robot_id, robot_intent in role_robot_intents:
                robot_intents[robot_id] = robot_intent
        return robot_intents
