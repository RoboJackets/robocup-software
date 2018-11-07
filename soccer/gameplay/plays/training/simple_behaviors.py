import robocup
import constants
import play
import skills
import tactics
import skills.pivot_kick

# This is a file where you can learn how skills work!
class SimpleBehaviors(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        # To make a robot move, use skills.move.Move(<point to move to>)
        # To create a point, we initialize a point using 
        # robocup.Point(<x coordinate>, <y coordinate>)
        
        # These lines moves a robot to the point (0, 0)
        
        
        self.linekicks = skills.line_kick.LineKick()
        self.linekicks.target = constants.Field.OurGoalSegment
        self.add_subbehavior(linekicks, "linekicks")



        # Adds behavior to our behavior tree, we will explain this more later
    def execute_running(self):
        
        self.linekicks = self.subbehavior_with_name(linekicks, "linekicks")
        if linekicks.is_done_running():
            self.linekicks.restart()

        