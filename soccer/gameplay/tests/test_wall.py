import robocup
import constants
import play
import skills
import tactics

# This is a file where you can learn how skills work!
class TestWall(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        tact = tactics.wall.Wall
        self.add_subbehavior(tact, "tactics")