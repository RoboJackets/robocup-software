import robocup
import forces
from forces import force, robot_force
import main


class LinearOurRobotForce(robot_force.RobotForce):

    def __init__(self, scale=5.0, cap=1.0):
        super()
        self.scale = scale
        self.cap = cap
        #self.updateVisibility()

    def sample(self, point ,clipped = True):
        
        self.ourRobots = main.system_state().our_robots
        vector = robocup.Point(0,0)
        
        closestBot = None
        closeDist = 9999999
        closeVec = None

        for g in self.ourRobots:
            vec = (point - g.pos)
            dist = vec.mag()
            if(dist < closeDist):
                closestBot = g
                closeDist = dist
                closeVec = vec

        
        outVec = robocup.Point((closeVec.x) * self.scale, (closeVec.y) * self.scale)

        outMag = outVec.mag()
        if(outMag > self.cap):
            outVec = robocup.Point(outVec.x / (outMag / self.cap), outVec.y / (outMag / self.cap))


        return outVec

