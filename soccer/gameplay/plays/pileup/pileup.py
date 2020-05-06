import robocup
import constants
import play
import skills
import tactics
import main
import evaluation

class Pileup(standard_play.StandardPlay):

    def __init__(self):
        super().__init__(continuous=False)
        
        x = main.ball().x
        y = main.ball().y

        print(x + "," + y)


        

