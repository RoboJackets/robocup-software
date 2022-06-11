from enum import Enum, auto
from typing import List

import stp
from rj_msgs.msg import RobotIntent

from rj_gameplay.tactic import dumb_tactic, pass_tactic


class State(Enum):
    INIT = auto()
    ACTIVE_MOVE = auto()
    INIT_PASS = auto()
    ASSIGN_ROLES = auto()
    ACTIVE = auto()
    FINALIZE = auto()


class BallPlacement(stp.play.Play):
    """Ball Placement play"""

    def __init__(self):
        super().__init__()

        self._state = State.INIT
        self._after_assign = State.INIT

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
        game_info = world_state.game_info

        # TODO: use angle between ball
        #  and side of field to figure out how much to add to ball_placement() vector
        #  currently the robot would be on top of the placement goal

        # TODO : clear out a path using global obstacles

        # TODO : make sure "backboard" robot and passer will always be distinct
        if self._state == State.INIT:
            pts = list()
            pts.append(game_info.ball_placement())
            self.prioritized_tactics = [dumb_tactic.DumbTactic(world_state, pts)]

            self.assign_roles(world_state)
            self._state = State.ACTIVE_MOVE

            return self.get_robot_intents(world_state)

        elif self._state == State.ACTIVE_MOVE:
            for tactic in self.prioritized_tactics:
                if tactic.needs_assign:
                    self._after_assign = State.ACTIVE_MOVE
                    self._state = State.ASSIGN_ROLES

            # only one active tactic at a time in this play
            tactic = self.prioritized_tactics[0]

            if tactic.is_done(world_state):
                self._state = State.INIT_PASS

            return self.get_robot_intents(world_state)

        elif self._state == State.INIT_PASS:
            init_passer_cost = stp.role.cost.PickClosestToPoint(world_state.ball.pos)
            init_receiver_cost = stp.role.cost.PickClosestToPoint(
                game_info.ball_placement()
            )

            self.prioritized_tactics = [
                pass_tactic.PassTactic(
                    world_state, init_passer_cost, init_receiver_cost
                )
            ]

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
                    self._after_assign = State.ACTIVE
                    self._state = State.ASSIGN_ROLES

            # only one active tactic at a time in this play
            tactic = self.prioritized_tactics[0]

            # now make the ball is correctly placed
            if tactic.is_done(world_state):
                self._state = State.FINALIZE

            return self.get_robot_intents(world_state)

        elif self._state == State.ASSIGN_ROLES:
            # duplicate code from init
            self.assign_roles(world_state)
            self._state = self._after_assign
            return self.get_robot_intents(world_state)
        elif self._state == State.FINALIZE:
            # TODO: make sure final location of ball is within ssl guidelines
            #  temp solution, just dribble back ROBOT_RADIUS meters

            # TODO : do this by adding a dribble configuration to dumb_tactic
            return self.get_robot_intents(world_state)
