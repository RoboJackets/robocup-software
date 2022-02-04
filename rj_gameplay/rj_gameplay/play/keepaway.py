from typing import List

import stp

from rj_gameplay.tactic import pass_tactic

from rj_msgs.msg import RobotIntent

from enum import Enum, auto


class State(Enum):
    INIT = auto()
    ACTIVE = auto()
    ASSIGN_ROLES = auto()


class Keepaway(stp.play.Play):
    """Play that passes repeatedly, effectively playing keepaway.
    See tick() for more details.
    """

    def __init__(self):
        super().__init__()

        # super simple FSM
        # TODO: use FSM class (or at least don't use string literals)
        self._state = State.INIT

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

        if self._state == State.INIT:
            self.prioritized_tactics = [pass_tactic.PassTactic(world_state)]
            # TODO: either add seek tactic(s) or unassigned behavior

            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)

        elif self._state == State.ACTIVE:
            # TODO: this loop's logic is fairly crucial in role assignment
            #
            # is there a way I can force this to happen as a precondition to assign_roles?
            # maybe call assign_roles() every tick but check tactic for needs_assign before assigning it
            # (this works as the method is in Play superclass)
            for tactic in self.prioritized_tactics:
                if tactic.needs_assign:
                    self._state = State.ASSIGN_ROLES

            # only one tactic in this play
            tactic = self.prioritized_tactics[0]
            if tactic.is_done(world_state):
                self._state = State.INIT

            return self.get_robot_intents(world_state)

        elif self._state == State.ASSIGN_ROLES:
            # duplicate code from init
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
