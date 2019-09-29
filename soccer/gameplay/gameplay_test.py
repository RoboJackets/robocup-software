## @brief A test description containing information about how to initialize and run a specific test
import play
import robocup

class GameplayTest():

    name = "longdescriptivename"
    ourRobots = []
    theirRobots = []
    ballPosition = robocup.Point(0, 0)
    play = play.Play

class testRobot():
    pos = robocup.Point(0,0)
    vel = robocup.Point(0,0)

    def __init__(self, position = None, velocity = robocup.Point(0, 0)):
        self.pos = position
        self.vel = velocity
