from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp

from rj_gameplay.tactic import pass_tactic

from rj_msgs.msg import RobotIntent

class Keepaway(stp.play.Play):
    def __init__(self):
        super().__init__()

        # super simple FSM
        # TODO: use FSM class (or at least don't use string literals)
        self.state = "init"

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:
        print("play state:", self.state)

        if self.state == "init":
            self.prioritized_tactics.append(pass_tactic.PassTactic(world_state))
            # TODO: either add seek tactic(s) or unassigned behavior

            self.assign_roles(world_state)
            self.state = "active"
            return self.get_robot_intents(world_state)

        elif self.state == "active":
            for tactic in self.prioritized_tactics:
                if tactic.needs_assign:
                    self.state = "assign_roles"

            return self.get_robot_intents(world_state)

        elif self.state == "assign_roles":
            print("*"*80)
            # duplicate code from init
            self.assign_roles(world_state)
            self.state = "active"
            return self.get_robot_intents(world_state)
