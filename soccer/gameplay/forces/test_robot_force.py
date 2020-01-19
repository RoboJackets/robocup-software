import robocup
import forces
from forces import force, robot_force
import main


class TestRobotForce(robot_force.RobotForce):

    def __init__(self, scale=5.0):
        super()
        self.scale = scale
        #self.updateVisibility()

    def sample(self, point ,clipped = True):
        
        self.ourRobots = main.system_state().our_robots
        vector = robocup.Point(0,0)

        for g in self.ourRobots:
            vector += (g.pos - point)

        return robocup.Point((1 / vector.x) * self.scale, (1 / vector.y) * self.scale)


