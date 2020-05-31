from typing import List, Optional, Tuple
import robocup
from math import sin, cos


def draw_heading(robot: robocup.Robot,
                 debug_drawer: robocup.DebugDrawer,
                 r: float = 0.5,
                 color: Tuple[int, int, int] = (0xff, 0xc1, 0xda)):
    """ Draws a line representing the heading of the robot
    :param robot: Robots to draw heading for
    :param debug_drawer: DebugDrawer
    :param r: How long to draw the heading line
    :param color: What color to draw the heading line
    """
    theta = robot.angle
    endpoint = robot.pos + robocup.Point(r * cos(theta), r * sin(theta))
    heading_line = robocup.Line(robot.pos, endpoint)
    debug_drawer.draw_line(heading_line, color, "Heading")


def draw_robot_viz(our_robots: Optional[List[robocup.OurRobot]],
                   their_robots: Optional[List[robocup.OpponentRobot]],
                   debug_drawer: robocup.DebugDrawer):
    """ Performs robot visualization agnostic to any play
    :param our_robots: List of our robots
    :param their_robots: List of their robots
    :param debug_drawer: Instance of robocup.DebugDrawer
    :return:
    """
    assert our_robots is not None
    assert their_robots is not None

    for robot in our_robots:
        draw_heading(robot, debug_drawer)

    for robot in their_robots:
        draw_heading(robot, debug_drawer)
