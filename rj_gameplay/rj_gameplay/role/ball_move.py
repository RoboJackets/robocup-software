import stp.role
import stp.rc

from rj_gameplay.skill import receive, pivot_kick  # , line_kick

from rj_msgs.msg import RobotIntent

from enum import Enum, auto

import numpy as np

# The final velocity of the ball when it reaches our teammate
FINAL_VELOCITY = 4
# Rolling deceleration of the ball after it has been kicked
BALL_DECELERATION = -0.4
# The robot can move a maximum of 1 meter with the ball, but we are going to give
# some leg room and make the maximum move distance 0.9m
MAXIMUM_MOVEMENT = 0.9


class State(Enum):
    INIT = auto()
    CAPTURING = auto()
    READY = auto()
    DONE = auto()


class BallMoveRole(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot) -> None:
        super().__init__(robot)
        self.robot_id = robot.id

        self.receive_skill = None
        self.pivot_kick_skill = None

        self._state = State.INIT

        self._target_point = None

        self.move_distance = 0.0
        self.initial_position = robot.pose[0:2]

    @property
    def ready(self):
        return self._state == State.READY

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
                self._state = State.READY
        elif self._state == State.READY:
            self.move_distance = np.linalg.norm(world_state.our_robots[self.robot_id].pose[0:2] - self.initial_position)
            # TODO: dribble until the receiver is ready
            if self.move_distance > MAXIMUM_MOVEMENT:
                self._state = State.DONE

        return intent

    def is_done(self, world_state) -> bool:
        return self._state == State.DONE
