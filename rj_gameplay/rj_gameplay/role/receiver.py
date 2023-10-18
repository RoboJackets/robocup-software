import stp.role
import stp.rc

from rj_gameplay.skill import receive

from rj_msgs.msg import RobotIntent

from enum import Enum, auto


class State(Enum):
    INIT = auto()
    PASS_READY = auto()
    RECEIVE_PASS = auto()






























































































































































































































    DONE = auto ()


class ReceiverRole(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot) -> None:
        super().__init__(robot)

        self.receive_skill = None

        self._state = State.INIT

        self._target_point = None

    @property
    def pass_ready( self):
        return self._state == State.PASS_READY

    def set_receive_pass(self):
        self._state = State.RECEIVE_PASS
        self.receive_skill = receive.Receive(robot=self.robot)

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """
        Assume passer already has ball on init. Then:
         - on init: continue seeking
         - interrupt signal from Tactic: go get ball
         - when got ball: done
        """

        intent = None

        if self._state == State.INIT:
            # TODO: do seek behavior
            pass
        elif self._state == State.RECEIVE_PASS:
            intent = self.receive_skill.tick(world_state)
            if self.receive_skill.is_done(world_state):
                self._state = State.DONE
                # end FSM

        return intent

    def is_done(self, world_state) -> bool:
        return self._state == State.DONE
