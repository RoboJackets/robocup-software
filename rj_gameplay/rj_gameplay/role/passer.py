from typing import Dict, Type, List, Any

import stp.role
import stp.rc

from rj_gameplay.skill import receive, pivot_kick  # , line_kick

from rj_msgs.msg import RobotIntent

from enum import Enum, auto


class State(Enum):
    INIT = auto()
    CAPTURING = auto()
    PASS_READY = auto()
    INIT_EXECUTE_PASS = auto()
    EXECUTE_PASS = auto()
    KICK_DONE = auto()


class PasserRole(stp.role.Role):
    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]], robot: stp.rc.Robot) -> None:
        super().__init__(action_client_dict, robot)

        self.receive_skill = None
        self.pivot_kick_skill = None

        self._state = State.INIT

        self._target_point = None

    @property
    def pass_ready(self):
        return self._state == State.PASS_READY

    def set_execute_pass(self, target_point):
        self._state = State.INIT_EXECUTE_PASS
        self._target_point = target_point

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """
        Assume robot does not have ball on init. Then:
         - on init: get ball
         - when got ball: mark pass ready for Tactic, dribble, wait
         - on pass signal from Tactic: pivot_kick to point, let receiver get ball, done
        """

        intent = None
        if self._state == State.INIT:
            self.receive_skill = receive.Receive(action_client_dict=self.action_client_dict,
                                                 robot=self.robot)
            intent = self.receive_skill.tick(world_state)
            self._state = State.CAPTURING
        elif self._state == State.CAPTURING:
            intent = self.receive_skill.tick(world_state)

            if self.receive_skill.is_done(world_state):
                self._state = State.PASS_READY
        elif self._state == State.PASS_READY:
            # TODO: dribble until the receiver is ready
            pass
        # this state transition is done by the PassTactic, which is not canonical FSM
        elif self._state == State.INIT_EXECUTE_PASS:
            # TODO: make these params configurable
            self.pivot_kick_skill = pivot_kick.PivotKick(
                action_client_dict=self.action_client_dict,
                robot=self.robot,
                target_point=self._target_point,
                chip=False,
                kick_speed=4.0,  # TODO: adjust based on dist from target_point
            )
            self._state = State.EXECUTE_PASS
        elif self._state == State.EXECUTE_PASS:
            intent = self.pivot_kick_skill.tick(world_state)

            if self.pivot_kick_skill.is_done(world_state):
                self._state = State.KICK_DONE
                # end FSM

        return intent

    def is_done(self, world_state) -> bool:
        return self._state == State.KICK_DONE
