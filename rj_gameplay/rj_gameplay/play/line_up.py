import stp.play
import stp.tactic

from rj_gameplay.tactic import line_tactic
import stp.skill
import stp.role
import stp.role.cost
from stp.role.assignment.naive import NaiveRoleAssignment
<<<<<<< HEAD
import stp.rc as rc
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar, Any
=======
import stp.rc
from typing import (
    Dict,
    List,
    Tuple,
    Optional,
    Type,
)
>>>>>>> bce13ce53ddb2ecb9696266d980722c34617dc15
import numpy as np
from rj_msgs.msg import RobotIntent

from enum import Enum, auto


<<<<<<< HEAD
    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]]):
        self.left_x = 1.0
        self.right_x = -1.5
        self.start_y = 2.0
        self.y_inc = 0.3
        self.move_right = move_tactic.Move(
            action_client_dict, np.array([self.right_x, self.start_y])
        )
        self.move_left = move_tactic.Move(
            action_client_dict, np.array([self.left_x, self.start_y])
        )
        self.role_assigner = NaiveRoleAssignment()
=======
class State(Enum):
    INIT = auto()
    LINE_UP = auto()
    DONE = auto()
>>>>>>> bce13ce53ddb2ecb9696266d980722c34617dc15


<<<<<<< HEAD
    def tick(
        self,
        world_state: rc.WorldState,
        prev_results: role.assignment.FlatRoleResults,
        props,
    ) -> Tuple[
        Dict[Type[tactic.SkillEntry], List[role.RoleRequest]], List[tactic.SkillEntry]
    ]:
        # Get role requests from all tactics and put them into a dictionary
        role_requests: play.RoleRequests = {}
        if self.move_right.is_done(world_state):
            role_requests[self.move_left] = self.move_left.get_requests(
                world_state, None
            )
        else:
            role_requests[self.move_right] = self.move_right.get_requests(
                world_state, None
            )
        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(
            flat_requests, world_state, prev_results
        )
        role_results = play.unflatten_results(flat_results)
=======
class LineUp(stp.play.Play):
    """Lines up all six robots on the side of the field."""
>>>>>>> bce13ce53ddb2ecb9696266d980722c34617dc15

    def __init__(self):
        super().__init__()

        self._state = State.INIT

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        if self._state == State.INIT:
            self.prioritized_tactics.append(line_tactic.LineTactic(world_state))
            self.assign_roles(world_state)
            self._state = State.LINE_UP
            return self.get_robot_intents(world_state)
        elif self._state == State.LINE_UP:
            if self.prioritized_tactics[0].is_done(world_state):
                self._state = State.DONE
            return self.get_robot_intents(world_state)
        elif self._state == State.DONE:
            # TODO: does this state need to exist?
            return None
