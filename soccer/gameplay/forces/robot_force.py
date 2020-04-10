import robocup
import forces
from forces import force
import main
from abc import abstractmethod


#So this is just a force that can keep track of robots for you
#Intended to act as a base class for forces that need this functionality
#Extend if you need something more, but actually I think I can make this
#do most of what you would want at a basic level?? IDK.
class RobotForce(force.Force):

    systemState = None
    activeRobots = list()

    ## A boolean lambda function for if a robot is a source for this force
    robot_criteria = lambda x : 
    ## The responce function 
    responce_function = lambda x :   

    ##I feel like I just remembered lambda functions exist and now want to use them for everything
    ##This is questionable?
    ##The main alternative would be sum I guess, but you could have more complex functions pretty easily
    merge_fucntion = lambda x : max(x)
    #Does this existing mean that I need to calculate for every robot? I guess the responces function should be pretty much trivial so it should hardly be slower.

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


        self.ourRobots = list()
        self.theirRobots = list()
        
        for g in self.systemState.our_robots:
            self.ourRobots.append(g)
        for g in self.systemState.their_robots:
            self.theirRobots.append(g)

    def sample(self, point, update_visibility=True):
        target_bots = filter(robot_criteria, activeRobots)




