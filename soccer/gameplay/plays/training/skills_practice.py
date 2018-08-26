import robocup
import constants
import play
import skills

# This is a file where you can learn how skills work!
class SkillsPractice(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        # To make a robot move, use skills.move.Move(<point to move to>)
        # To create a point, we initialize a point using 
        # robocup.Point(<x coordinate>, <y coordinate>)
        
        # This line moves a robot to the point (0, 0)
        move_point = robocup.Point(3, 9)
        skill = skills.move.Move(move_point)

        # Adds behavior to our behavior tree, we will explain this more later
        self.add_subbehavior(skill, "skill", required=True)