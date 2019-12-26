## @brief A test description containing information about how to initialize and run a specific test
import play
import robocup
import constants


# Class to define robot locations/velocities
class testRobot():
    def __init__(self, pos=robocup.Point(0,0), angle=0):

        ## TODO: Do this coordinate transform in c++, closer to being sent to grsim
        ##
        self.pos = robocup.Point(pos.y - (constants.Field.Length / 2), -1 * pos.x)
        ## 

        self.angle = angle


'''
# Enum of possible referee commands
robocup.Command:
    halt = 0
    stop = 1
    normal_start = 2
    force_start = 3
    prepare_kickoff_yellow = 4
    prepare_kickoff_blue = 5
    prepare_penalty_yellow = 6
    prepare_penalty_blue = 7
    direct_free_yellow = 8
    direct_free_blue = 9
    indirect_free_yellow = 10
    indirect_free_blue = 11
    timeout_yellow = 12
    timeout_blue = 13
    goal_yellow = 14
    goal_blue = 15
    ball_placement_yellow = 16
    ball_placement_blue = 17
'''


class GameplayTest():
    def __init__(self):

        # Variables to be set by user

        # Note this name is not the one that will appear in the tab itself, this is only for the results output
        self.name = "descriptive name"

        # ALL POSITIONS ARE SPECIFIED IN WORLD COORDINATES
        #    ((0,0) is the center of the field with +x being up and +y being left)
        # Don't forget that goalie will probably be pulled from ourRobots
        self.ourRobots = []
        self.theirRobots = []

        self.ballPosition = robocup.Point(0, 0)
        self.ballVelocity = robocup.Point(0, 0)

        # List of plays to enable e.g. ["stopped", "offense/basic_122"]
        # You can also add the name of a playbook file i.e. 
        # ["comp2019.pbk", "testing/test_pivot_kick"]
        # If the list is empty, the enabled play list will remain as it was 
        # before the test was run
        self.play_list = []

        # List of referee commands to run at the start (will run 1 each frame)
        # Default value is seen here
        self.start_commands = [
            robocup.Command.halt,
            robocup.Command.force_start
        ] # yapf: disable
