import stp

from rj_gameplay.tactic import wall_tactic, goalie_tactic
import stp.role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc
from typing import Dict, List, Tuple, Type, Any
from rj_gameplay.calculations import wall_calculations

import stp.role.cost
from rj_msgs.msg import RobotIntent

from enum import Enum, auto

<<<<<<< HEAD
    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]]):

        self._action_client_dict = action_client_dict
        self.tactics = [
            wall_tactic.WallTactic(action_client_dict),
            wall_tactic.WallTactic(action_client_dict),
            wall_tactic.WallTactic(action_client_dict),
            nmark_tactic.NMarkTactic(action_client_dict, 2),
            goalie_tactic.GoalieTactic(action_client_dict),
        ]

        self.num_wallers = 3

        self.role_assigner = NaiveRoleAssignment()

    def compute_props(self, prev_props):
        pass

    def tick(
        self,
        world_state: rc.WorldState,
        prev_results: role.assignment.FlatRoleResults,
        props,
    ) -> Tuple[
        Dict[Type[tactic.SkillEntry], List[role.RoleRequest]], List[tactic.SkillEntry]
    ]:
=======

class State(Enum):
    INIT = auto()
    ACTIVE = auto()
>>>>>>> bce13ce53ddb2ecb9696266d980722c34617dc15


<<<<<<< HEAD
        # Get role requests from all tactics and put them into a dictionary
        role_requests: play.RoleRequests = {}
        i = 0
        for tactic in self.tactics:
            if type(tactic) == WallTactic:
                role_requests[tactic] = tactic.get_requests(
                    world_state, wall_pts[i], None
                )
                i += 1
            else:
                role_requests[tactic] = tactic.get_requests(world_state, None)
        """role_requests: play.RoleRequests = {
            tactic: tactic.get_requests(world_state, None)
            for tactic in self.tactics
        }"""
=======
class BasicDefense(stp.play.Play):
    """Play that consists of:
    - 1 Goalie
    - 5 Wallers
    TODO: add 2 aggressive markers, go down to 3 Wallers
    """
>>>>>>> bce13ce53ddb2ecb9696266d980722c34617dc15

    def __init__(self):
        super().__init__()

        self._state = State.INIT

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        if self._state == State.INIT:
            self.prioritized_tactics.append(goalie_tactic.GoalieTactic(world_state, 0))
            self.prioritized_tactics.append(wall_tactic.WallTactic(world_state, 5))
            # TODO: add nmark tactic
            #       and make it go for the ball (rather than stopping in front)
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        elif self._state == State.ACTIVE:
            # return robot intents from assigned tactics back to gameplay node
            return self.get_robot_intents(world_state)
