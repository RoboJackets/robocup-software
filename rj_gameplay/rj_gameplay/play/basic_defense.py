import stp

from rj_gameplay.tactic import wall_tactic, nmark_tactic, goalie_tactic
import stp.role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc
from typing import Dict, List, Tuple, Type
from rj_gameplay.calculations import wall_calculations

import stp.role.cost
from rj_msgs.msg import RobotIntent


class BasicDefense(stp.play.Play):
    """Play that consists of:
     - 1 Goalie
     - 5 Wallers
     TODO: add aggressive markers, go down to 3 Wallers
    """

    def __init__(self):
        super().__init__()

        # super simple FSM
        # TODO: use FSM class (or at least don't use string literals)
        self.state = "init"

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        if self.state is "init":
            self.prioritized_tactics.append(goalie_tactic.GoalieTactic(world_state, 0))
            self.prioritized_tactics.append(wall_tactic.WallTactic(world_state, 5))
            # TODO: add nmark tactic
            #       and make it go for the ball (rather than stopping in front)
            self.assign_roles(world_state)
            self.state = "active"
            return self.get_robot_intents(world_state)
        elif self.state is "active":
            # return robot intents from assigned tactics back to gameplay node
            return self.get_robot_intents(world_state)
