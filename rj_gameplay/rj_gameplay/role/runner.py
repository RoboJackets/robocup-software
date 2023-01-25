from enum import Enum, auto

import stp.rc
import stp.role
from rj_msgs.msg import RobotIntent

from rj_gameplay.skill import move


class State(Enum):
    INIT = auto()
    RUN_READY = auto()
    EXECUTE_RUN = auto()
    DONE = auto()


class RunnerRole(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot, target_point: Tuple) -> None:
        super().__init__(robot)

        self.move_skill = None

        self._state = State.INIT

        self._target_point = target_point

    @property
    def run_ready(self):
        return self._state == State.RUN_READY

    def set_execute_run(self):
        self._state = State.EXECUTE_RUN
        self.move_skill = move.Move(robot=self.robot)

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """
        Assume passer already has ball on init. Then:
         - on init: continue seeking
         - interrupt signal from Tactic: go get ball
         - when got ball: done
        """

        intent = None

        if self._state == State.INIT:
            self._state = State.RUN_READY
            pass
        elif run_ready():
            self._state = State.EXECUTE_RUN
        elif self._state == State.EXECUTE_RUN:
            intent = self.move_skill.tick(world_state)
            if self.move_skill.is_done(world_state):
                self._state = State.DONE
                # end FSM

        return intent

    def is_done(self, world_state) -> bool:
        return self._state == State.DONE
