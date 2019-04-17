import robocup
import constants
import play
import skills
import tactics
import evaluation
import main

class BasicOffense(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        capture = skills.capture.Capture()
        center = robocup.Point(0,4.5)
        point_1 = robocup.Point(-2.17, 6.22)
        point_2 = robocup.Point(2.37, 6.56)
        move_1 = skills.move.Move(point_1)
        move_2 = skills.move.Move(point_2)
        pivot_kick = skills.pivot_kick.PivotKick()

        self.add_subbehavior(move_1, "move 1")
        self.add_subbehavior(move_2, "move 2")
        self.add_subbehavior(capture, "capture")
        '''
        shot_chance = evaluation.shooting.eval_shot(center,main.our_robots())
        pass_1_chance = evaluation.passing.eval_pass(center,point_1,main.our_robots())
        pass_2_chance = evaluation.passing.eval_pass(center, point_2, main.our_robots())
        shot_1_chance = evaluation.shooting.eval_shot(point_1,main.our_robots())
        shot_2_chance = evaluation.shooting.eval_shot(point,main.our_robots())
        if (shot_chance > 0.5): 
            self.add_subbehavior(pivot_kick, "kick")
        elif ()'''
