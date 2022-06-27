from enum import Enum, auto
from typing import List, Optional

import numpy as np
import stp
import stp.rc
import stp.role
import stp.role.cost
from rclpy.node import Node
from rj_geometry_msgs.msg import Point
from rj_msgs.msg import RobotIntent

from rj_gameplay.client import ball_placement_client


class State(Enum):
    INIT = auto()


class BallPlacement(stp.play.Play):
    """
    Play for ball placement
    This  was supposed to be a wrapper for the action client.
    But it's not a ros node, so it can't call other nodes.
    Kept to give play selector something to work with.
    """

    def __init__(self):
        super().__init__()

        self._state = State.INIT

    def tick(
        self, world_state: stp.rc.WorldState, client: Optional[Node]
    ) -> Optional[List[RobotIntent]]:
        game_info = world_state.game_info
        goal_array: np.array = game_info.ball_placement()
        goal_pt: Point = Point(x=goal_array[0], y=goal_array[1])
        print(client)
        client.send_goal(goal_pt)
        return None
