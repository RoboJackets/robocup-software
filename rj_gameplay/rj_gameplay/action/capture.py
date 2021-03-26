from stp.action import IFiniteAction
from rj_msgs.msg import RobotIntent

class Capture(IFiniteAction):

    def __init__(self, robot):
        pass

    def tick(self, publisher) -> None:
        pass

    def done(self, world_state) -> bool:
        pass

