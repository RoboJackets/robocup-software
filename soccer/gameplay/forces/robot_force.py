import robocup
import forces
from forces import force
import main
from abc import abstractmethod
from forces import direction
from forces import points_force

#So this is just a force that can keep track of robots for you
#Intended to act as a base class for forces that need this functionality
#Extend if you need something more, but actually I think I can make this
#do most of what you would want at a basic level?? IDK.
class RobotForce(points_force.PointsForce):

    systemState = None
    activeRobots = list()

    ## A boolean lambda function for if a robot is a source for this force
    robot_criteria = lambda x : True

    #Examples of robot criteria functions
    #robot_criteria = lambda x : not x.is_ours()
    #robot_criteria = lambda x : x.pos.x > 2
    
    def __init__(self):
        self.systemState = main.system_state()
        self.updateVisibility()        

    #Function to update which robots are currently visable and on the field
    def updateVisibility(self):
        if(self.systemState is None):
            self.systemState = main.system_state()

        self.activeRobots.clear()
        for g in self.robotList:
            if (g.visible):
                self.activeRobots.append(g)

    #So ideally this will call the super function from points force
    def sample(self, point, update_visibility=True):
        updateVisibility()
        self.points = filter(robot_criteria, activeRobots)
        return super(point) 


