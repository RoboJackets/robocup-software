import robocup
import forces
from forces import force
import main
from abc import abstractmethod
from forces import direction
from forces import points_force
from forces import force_utils



#So this is just a force that can keep track of robots for you
#Extend if you need something more, but actually I think I can make this
#do most of what you would want at a basic level with the lambda functions
class RobotForce(points_force.PointsForce):

    systemState = None
    activeRobots = list()
    filtered_robots = list()
    robotList = list()

    ## A boolean lambda function for if a robot is a source for this force
    robot_criteria = lambda s, x : True

    #robot_criteria = lambda s, x : x.shell_id == 3

    responce_direction = direction.Direction.PUSH
    merge_function = lambda s, x : sum(x)
    responce_function = lambda s, x : force_utils.log_responce(x, 1.0, 3.0, threshold=0.1)


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

        self.robotList.clear()    
        self.robotList.extend(self.systemState.our_robots)
        self.robotList.extend(self.systemState.their_robots)

        #for g in self.systemState.our_robots:
        #    self.robotList.append(g)
        #for g in self.systemState.their_robots:
        #    self.robotList.append(g)

        self.activeRobots.clear()
        for g in self.robotList:
            if (g.visible):
                self.activeRobots.append(g)

    def updateFilter(self):
        self.filtered_robots.clear()
        self.filtered_robots = list(filter(self.robot_criteria, self.activeRobots))

    def updatePoints(self):
        self.points.clear()
        for g in self.filtered_robots:
            self.add_point_raw(g.pos)


    #Right now it's doing all of the updates every frame, this should probably at least be changed to every couple frames or not at all.
    #Correction? I think it's doing it once and not again
    def sample(self, point):
        if(self.initalized is False):
            self.updateVisibility()
            self.updateFilter()
            self.updatePoints()

        return super().sample(point) 


