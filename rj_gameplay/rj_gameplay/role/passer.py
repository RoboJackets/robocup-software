from enum import Enum, auto

import numpy as np
import stp.rc
import stp.role
from rj_msgs.msg import RobotIntent

from rj_gameplay.skill import pivot_kick, receive  # , line_kick

# The final velocity of the ball when it reaches our teammate
FINAL_VELOCITY = 3
# Rolling deceleration of the ball after it has been kicked
BALL_DECELERATION = -0.4


class State(Enum):
    INIT = auto()
    CAPTURING = auto()
    PASS_READY = auto()
    INIT_EXECUTE_PASS = auto()
    EXECUTE_PASS = auto()
    KICK_DONE = auto()


class PasserRole(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot) -> None:
        super().__init__(robot)

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
            self.receive_skill = receive.Receive(robot=self.robot)
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
            # kick_speed is modeled off of the ETDP of ZJUNlict, which can be found in section 5 of https://ssl.robocup.org/wp-content/uploads/2020/03/2020_ETDP_ZJUNlict.pdf
            distance = np.linalg.norm(self._target_point - self.robot.pose[0:2])
            initial_velocity = np.sqrt(
                (FINAL_VELOCITY**2) - (2 * BALL_DECELERATION * distance)
            )
            self.pivot_kick_skill = pivot_kick.PivotKick(
                robot=self.robot,
                target_point=self._target_point,
                chip=False,
                kick_speed=initial_velocity,
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
