from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp

from rj_gameplay.role import receiver, passer
import numpy as np

import stp.global_parameters as global_parameters

class PassTactic(stp.tactic.Tactic):
    def __init__(self, world_state: stp.rc.WorldState, num_wallers: int):
        super().__init__(world_state)

        # TODO: make FSM class (or at least use enum instead of str literals)
        self._state = "init"

    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            pt = self.wall_pts[i]
            if role is dumb_move.DumbMove:
                self.assigned_roles.append(role(robot, pt, world_state.ball.pos))

    def tick(self, world_state: stp.rc.WorldState):
        # TODO: make this empty RobotIntent so it doesn't crash on no input
        role_intents = []

        if self._state == "init":
            # TODO: have a feeling this will crash when the None intent gets to return
            self._role_requests = [
                    (stp.role.cost.PickClosestRobot(world_state.ball.pos), passer.PasserRole)
                ]
            self._state = "passer_capture"
        elif self._state == "passer_capture":
            # assumes play has given new role_requests
            self.init_roles(world_state)

            passer_role = self.assigned_roles[0] 
            assert passer_role is stp.role.PasserRole
            intent = passer_role.tick(world_state)

            # TODO: no idea if role has robot property
            role_intents = [(passer_role.robot.id, intent)]

            if passer_role.pass_ready:
                self._state = "get_receiver"
        elif self._state == "get_receiver":
            # when does play assign roles? whenever I want it to :)
            # TODO: send signal to play here (set flag to true)
            self._role_requests = [
                    (stp.role.cost.PickClosestRobot(world_state.ball.pos), passer.PasserRole),
                    (stp.role.cost.PickClosestRobot(world_state.field.their_goal_loc), receiver.ReceiverRole)
                ]
            self._state = "execute_pass"
        elif self._state == "execute_pass":
            # assumes play has given new role_requests
            self.init_roles(world_state)

            # TODO: rest of FSM
            # set signal to passer pass
            # state
            # check passer pass
            # state
            # set signal recv go
            # kill passer request
            # state
            # await receiver done
            # done

        return role_intents

    @property
    def needs_assign(self):
        # maybe issues with one-tick flag?
        return self._state == "get_receiver"


    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        pass
