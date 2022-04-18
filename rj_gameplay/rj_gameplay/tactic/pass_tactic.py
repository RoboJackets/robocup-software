from enum import Enum, auto
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import numpy as np
import stp
import stp.global_parameters as global_parameters
from rj_msgs.msg import RobotIntent

from rj_gameplay.role import passer, receiver


class State(Enum):
    INIT = auto()
    INIT_PASSER_CAPTURE = auto()
    PASSER_CAPTURE = auto()
    GET_RECEIVER = auto()
    INIT_EXECUTE_PASS = auto()
    EXECUTE_PASS = auto()
    AWAIT_PASSER_KICK = auto()
    PASS_IN_TRANSIT = auto()
    INIT_AWAIT_RECEIVE = auto()
    EXECUTE_RECEIVE = auto()
    AWAIT_RECEIVE = auto()
    DONE = auto()


class PassTactic(stp.tactic.Tactic):
    def __init__(
        self,
        world_state: stp.rc.WorldState,
        init_passer_cost: stp.role.CostFn,
        init_receiver_cost: stp.role.CostFn,
    ):
        super().__init__(world_state)

        self._state = State.INIT

        self._passer_cost = init_passer_cost
        self._receiver_cost = init_receiver_cost
        self._stop_seeking = False

    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        self.assigned_roles = []
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            if role is passer.PasserRole:
                self.assigned_roles.append(role(robot))
                # print("Passer: ", robot.id)
            elif role is receiver.ReceiverRole:
                self.assigned_roles.append(role(robot))
                # print("Receiver: ", robot.id)

    def tick(
        self, world_state: stp.rc.WorldState
    ) -> List[Tuple[int, RobotIntent]]:  # (id, intent)
        """
        FSM
         - init: request passer
         - on filled req: init_roles > tick passer's capture
         - when passer ready to pass: request receiver
         - on filled req: init_roles > tick passer pass
         - on ball passed: tick receiver, release passer role
         - when receiver done: done
        """

        role_intents = []             

        if self._state == State.INIT:
            self._role_requests = [
                (self._passer_cost, passer.PasserRole)
            ]
            self._needs_assign = True

            self._state = State.INIT_PASSER_CAPTURE

        elif self._state == State.INIT_PASSER_CAPTURE:
            # assumes play has given new role_requests
            self.init_roles(world_state)
            self._state = State.PASSER_CAPTURE

        elif self._state == State.PASSER_CAPTURE:
            # TODO: these lines are a little ugly, any fix?
            passer_role = self.assigned_roles[0]
            intent = passer_role.tick(world_state)

            role_intents = [(passer_role.robot.id, intent)]

            if passer_role.pass_ready:
                self._state = State.GET_RECEIVER

        elif self._state == State.GET_RECEIVER:
            self._receiver_cost = stp.role.cost.PickShortestPositiveReceiver()
            self._role_requests.append(
                (self._receiver_cost, receiver.ReceiverRole)
            )
            self._needs_assign = True

            self._state = State.INIT_EXECUTE_PASS
            # decided to get receiver early to allow more coordination in theory, currently not written in
            # TODO: evaluate whether this is a dumb idea or not

        elif self._state == State.INIT_EXECUTE_PASS:
            # Wait until play assignment assigns the needed 2 roles
            if len(self.assigned_roles) == 2:
                self._state = State.EXECUTE_PASS
                self._stop_seeking = True

        elif self._state == State.EXECUTE_PASS:
            # assumes play has given new role_requests
            # self.init_roles(world_state)

            # TODO: these lines are a little ugly, any fix?
            passer_role = self.assigned_roles[0]
            receiver_role = self.assigned_roles[1]

            # TODO: create func to find good target point
            # TODO: should update receiver_role robot every tick in the role (see skill/capture.py)
            target_point = world_state.our_robots[receiver_role.robot.id].pose[0:2]
            passer_role.update_target_point(target_point)
            passer_role.set_execute_pass()
            # NEED TO HAVE SEEKER STOP SEEKING AT THIS POINT

            role_intents = [
                (passer_role.robot.id, passer_role.tick(world_state)),
                (receiver_role.robot.id, receiver_role.tick(world_state)),
            ]

            self._state = State.AWAIT_PASSER_KICK

        elif self._state == State.AWAIT_PASSER_KICK:
            # TODO: these lines are a little ugly, any fix?
            passer_role = self.assigned_roles[0]
            receiver_role = self.assigned_roles[1]

            # target_point = world_state.our_robots[receiver_role.robot.id].pose[0:2]
            # passer_role.update_target_point(target_point)

            # print("Target Point: ", target_point)

            role_intents = [
                (passer_role.robot.id, passer_role.tick(world_state)),
                (receiver_role.robot.id, receiver_role.tick(world_state)),
            ]

            if passer_role.is_done(world_state):
                receiver_role.set_receive_pass()
                self._state = State.EXECUTE_RECEIVE

        # elif self._state == State.PASS_IN_TRANSIT:
        #     receiver_role = self.assigned_roles[1]

        #     # TODO: here, we assume the initially picked Receiver is the best one
        #     # this may not be true (i.e. when intercept planning is bad and the ball does not get captured by the first receiver)
        #     # the receiver then should be reassigned to closest to ball
        #     # maybe check some is_done()?
        #     self._role_requests = [
        #         (
        #             self._receiver_cost,
        #             receiver.ReceiverRole,
        #         )
        #     ]
        #     self._needs_assign = True

        #     self._state = State.INIT_AWAIT_RECEIVE

        # elif self._state == State.INIT_AWAIT_RECEIVE:
        #     # Wait until play assignment assigns the needed 1 roles
        #     if len(self.assigned_roles) == 1:
        #         self._state = State.EXECUTE_RECEIVE

        # elif self._state == State.EXECUTE_RECEIVE:
        #     # assumes play has given new role_requests
        #     self.init_roles(world_state)

        #     # TODO: these lines are a little ugly, any fix?
        #     receiver_role = self.assigned_roles[1]

        #     role_intents = [(receiver_role.robot.id, receiver_role.tick(world_state))]

        #     self._state = State.AWAIT_RECEIVE

        elif self._state == State.EXECUTE_RECEIVE:
            receiver_role = self.assigned_roles[1]

            role_intents = [(receiver_role.robot.id, receiver_role.tick(world_state))]

            if receiver_role.is_done(world_state):
                self._state = State.DONE
                # end FSM

        return role_intents

    @property
    def needs_assign(self):
        # style is wrong here: this convoluted way of returning self._needs_assign is necessary because we want to set it to False after the call, always
        ret = self._needs_assign == True  # noqa: E712
        self._needs_assign = False
        return ret

    @property
    def stop_seeking():
        return self._stop_seeking

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return self._state == State.DONE
