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


class BallPlacement(stp.play.Play):
    """
    Dummy play for ball placement

    To see where actual ball placement is handled, see rj_gameplay/client/ball_placement_client.py.
    """

    def __init__(self):
        super().__init__()

        self._state = None

    def tick(
        self, world_state: stp.rc.WorldState, client: Optional[Node]
    ) -> Optional[List[RobotIntent]]:
        return None
