from enum import Enum, auto
from typing import List

import stp
import stp.rc
import stp.role
import stp.role.cost
from rj_msgs.msg import RobotIntent

from rj_gameplay.role import runner


class State(Enum):
    INIT = auto()
    GO_TO_EDGE = auto()
    TOP_RIGHT = auto()
    BOTTOM_RIGHT = auto()
    BOTTOM_LEFT = auto()
    TOP_LEFT = auto()
    FINAL_LEG = auto()
    DONE = auto()


class Runner(stp.play.Play):
    """distraction play by running a lap"""

    def __init__(self):
        super().__init__()
        self._mappings = {}
        self._state = State.INIT

        self.width = stp.rc.WorldState._field._width_m
        self.length = stp.rc.WorldState._field._length_m

        self._mappings["start"] = (0.0, self.length)
        self._mappings["topright"] = (self.width / 2.0, self.length)
        self._mappings["bottomright"] = (self.width / 2.0, 0.0)
        self._mappings["bottomleft"] = (-self.width / 2.0, 0.0)
        self._mappings["topleft"] = (-self.width / 2.0, self.length)
        self._mappings["end"] = self._mappings["start"]

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        if self._state == State.INIT:
            self._state = State.GO_TO_EDGE
            return self.get_robot_intents(world_state)
        elif self._state == State.GO_TO_EDGE:
            self.prioritized_tactics = [
                runner.RunnerRole(world_state, self._mappings["start"])
            ]
            self._state = State.TOP_RIGHT
            return self.get_robot_intents(world_state)
        elif self._state == State.TOP_RIGHT:
            self.prioritized_tactics = [
                runner.RunnerRole(world_state, self._mappings["topright"])
            ]
            self._state = State.BOTTOM_RIGHT
            return self.get_robot_intents(world_state)
        elif self._state == State.BOTTOM_RIGHT:
            self.prioritized_tactics = [
                runner.RunnerRole(world_state, self._mappings["bottomright"])
            ]
            self._state = State.BOTTOM_LEFT
            return self.get_robot_intents(world_state)
        elif self._state == State.BOTTOM_LEFT:
            self.prioritized_tactics = [
                runner.RunnerRole(world_state, self._mappings["bottomleft"])
            ]
            self._state = State.TOP_LEFT
            return self.get_robot_intents(world_state)
        elif self._state == State.TOP_LEFT:
            self.prioritized_tactics = [
                runner.RunnerRole(world_state, self._mappings["topleft"])
            ]
            self._state = State.FINAL_LEG
            return self.get_robot_intents(world_state)
        elif self._state == State.FINAL_LEG:
            self.prioritized_tactics = [
                runner.RunnerRole(world_state, self._mappings["end"])
            ]
            self._state = State.DONE
            return self.get_robot_intents(world_state)
        elif self._state == State.DONE:
            # return robot intents from assigned tactics back to gameplay node
            return self.get_robot_intents(world_state)
