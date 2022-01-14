import stp.play
import stp.tactic as tactic

from rj_gameplay.tactic import wall_tactic, nmark_tactic, goalie_tactic, move_tactic
import stp.role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc
from typing import Dict, List, Tuple, Type
from rj_gameplay.calculations import wall_calculations

import stp.role.cost
from rj_msgs.msg import RobotIntent


class BasicDefense(stp.play.Play):
    """For when we don't have the ball and are trying to stop the opponent from scoring."""

    def __init__(self):
        super().__init__()

        # super simple FSM
        self.state = "init"

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        if self.state is "init":
            # TODO: add goalie T
            self.prioritized_tactics.append(wall_tactic.WallTactic(world_state, 5))
            self.assign_roles(world_state)
            self.state = "active"
            return self.get_robot_intents(world_state)
        elif self.state is "active":
            # return robot intents from assigned tactics back to gameplay node
            return self.get_robot_intents(world_state)
