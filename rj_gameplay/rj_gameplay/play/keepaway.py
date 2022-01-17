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
        """
        init: assign one pass tactic, several seekers
        pass_active: execute that one pass
        when tactic needs new roles: assign_roles -> active
        when pass_done: return to init
        (the effect is to pass indefinitely)
        """
        print("play state:", self.state)

        if self.state == "init":
            self.prioritized_tactics = [pass_tactic.PassTactic(world_state)]
            # TODO: either add seek tactic(s) or unassigned behavior

            self.assign_roles(world_state)
            self.state = "active"
            return self.get_robot_intents(world_state)

        elif self.state == "active":
            for tactic in self.prioritized_tactics:
                # TODO: this line/logic is fairly crucial in role assignment,
                # is there a way I can force this to happen as a precondition to assign_roles?
                # maybe call assign_roles() every tick but check tactic for needs_assign before assigning it
                # (this works as the method is in Play superclass)
                if tactic.needs_assign:
                    self.state = "assign_roles"

                # TODO: this line abuses the fact that there's only one tactic rn, fix please
                if tactic.is_done(world_state):
                    self.state = "init"

            return self.get_robot_intents(world_state)

        elif self.state == "assign_roles":
            print("*" * 80)
            # duplicate code from init
            self.assign_roles(world_state)
            self.state = "active"
            return self.get_robot_intents(world_state)
