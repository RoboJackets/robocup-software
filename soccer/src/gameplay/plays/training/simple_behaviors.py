import robocup
import constants
import play
import skills
import tactics

# This is a file where you can learn how skills work!
class SimpleBehaviors(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        # To make a robot move, use skills.move.Move(<point to move to>)
        # To create a point, we initialize a point using 
        # robocup.Point(<x coordinate>, <y coordinate>)
        
        # These lines moves a robot to the point (0, 0)
        move_point = robocup.Point(0, 0)
        skill = skills.move.Move(move_point)

        # Adds behavior to our behavior tree, we will explain this more later
        self.add_subbehavior(skill, "skill")