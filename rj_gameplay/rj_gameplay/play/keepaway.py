from enum import Enum, auto
from typing import List

import stp
from rj_msgs.msg import RobotIntent

from rj_gameplay.tactic import goalie_tactic, pass_tactic


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
            init_passer_cost = stp.role.cost.PickClosestToPoint(world_state.ball.pos)
            init_receiver_cost = stp.role.cost.PickFarthestFromPoint(
                world_state.ball.pos
            )
            # init_receiver_cost = stp.role.cost.PickClosestInFront(world_state.ball.pos)
            self.prioritized_tactics = [
                goalie_tactic.GoalieTactic(world_state, 0),
                pass_tactic.PassTactic(
                    world_state, init_passer_cost, init_receiver_cost
                ),
            ]
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
