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
        
        closestBot = None
        closeDist = 9999999
        closeVec = None

        for g in self.ourRobots:
            if((g.pos - point).mag() < closeDist):
                closestBot = g
                closeDist = (g.pos - point).mag() 
                closeVec = (g.pos - point)

        return robocup.Point((1 / closeVec.x) * self.scale, (1 / closeVec.y) * self.scale)


