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

import stp.action as action
import stp.skill as skill
import stp.rc as rc
import stp.role as role
import stp.role.assignment as assignment
import stp.tactic as tactic
import stp.utils.enum as enum
import stp.utils.typed_key_dict as tkdict
import stp.role as role
from stp.tactic import SkillEntry

from rj_msgs.msg import RobotIntent 

PropT = TypeVar("PropT")

# TODO: move to stp.role
# roles = un-initialized Tactics
Role = Type[tactic.ITactic]

class Play(ABC):
    """Coordinate full-team behaviors via Tactics. Assumes number of roles matches number of robots on the field. See tick() for more details.
    """

    def __init__(self):
        self.ordered_costs: List[role.costFn] = [] 
        self.ordered_roles: List[Role] = []
        self.ordered_tactics: List[ITactic] = []

    @abstractmethod
    def tick(
        self,
        world_state: rc.WorldState,
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

    # TODO: add typing when finalized
    def assign_roles(
        self,
        world_state: rc.WorldState,
    ) -> None:
        """Given that all roles are in sorted order of priority, greedily assign the highest-priority role to the lowest-cost robot for that role. Instantiate tactics with the correct robots post-assignment.
        """

        assigned_robots = []
        used_robots = set()
        for cost_fn in self.ordered_costs:
            cheapest_robot = min(cost_fn(robot, world_state) for robot in world_state.our_robots if robot not in used_robots)

            if cheapest_robot is None:
                # TODO: properly error handle if cheapest_robot is None
                print(f'cost_fn {cost_fn} was not assigned')

            used_robots.add(cheapest_robot)
            assigned_robots.append(cheapest_robot)

        # trying some things: 
        # https://stackoverflow.com/questions/8421199/create-an-instance-i-already-have-the-type
        # https://stackoverflow.com/questions/55751368/python-how-to-pass-to-a-function-argument-type-of-a-class-object-typing

        # lazy method requires reforming list every time, may be slow
        self.ordered_tactics = []
        for robot, role in zip(assigned_robots, ordered_roles):
            # initialize a tactic now that role has been decided
            new_tactic = role(robot)
            self.ordered_tactics.append(new_tactic)


    def get_robot_intents(self, world_state: rc.WorldState) -> List[RobotIntent]:
        """Given assigned roles in self.ordered_tactics, tick each tactic and aggregate the results in one List, where indices are robot_ids and values are RobotIntents. GameplayNode then sends these back to motion planning via ROS.
        """
        robot_intents = [None for _ in self.ordered_tactics]
        for tactic in self.ordered_tactics:
            robot_id, robot_intent = tactic.tick(world_state)
            robot_intents[robot_id] = robot_intent
        return robot_intents



RoleRequests = Dict[Type[tactic.ITactic], tactic.RoleRequests]
RoleResults = Dict[Type[tactic.ITactic], tactic.RoleResults]
