import robocup
import play
import behavior
import skills.move
import skills.capture
import tactics.coordinated_pass
import constants
import main
import enum
import evaluation

class TriangleOffense(play.Play):
    class State(enum.Enum):
        ## 2 robots get on the corners of a triangle,
        # while a third fetches the ball
        setup = 1
        evaluation = 2
        passing = 3
        shooting = 4


    def __init__(self):
        super().__init__(continuous=True)
        self.passing = False
        # register states - they're both substates of "running"
        print(robocup.Field_Dimensions.CurrentDimensions.__dict__)
        self.add_state(TriangleOffense.State.setup,
                       behavior.Behavior.State.running)
        self.add_state(TriangleOffense.State.evaluation,
                       behavior.Behavior.State.running)
        self.add_state(TriangleOffense.State.passing,
                       behavior.Behavior.State.running)
        self.add_state(TriangleOffense.State.shooting,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            TriangleOffense.State.setup, lambda: True,
                            'immediately')
        self.add_transition(
            TriangleOffense.State.setup, TriangleOffense.State.evaluation, lambda: self.
            all_subbehaviors_completed(), 'all subbehaviors completed')
        #Transition from evaluation to either shooting or passing
        self.add_transition(
            TriangleOffense.State.evaluation, TriangleOffense.State.passing, lambda: self.passing, 'Make Setup Pass')
        self.add_transition(
            TriangleOffense.State.evaluation, TriangleOffense.State.shooting, lambda: not self.passing, 'Shoot immediately')

        self.add_transition(
            TriangleOffense.State.passing, TriangleOffense.State.shooting, lambda: self.
            all_subbehaviors_completed(), 'all subbehaviors completed')

        self.offensive_points = [
            robocup.Point(-1*constants.Field.Width / 3, 3*constants.Field.Length / 5 ),
            robocup.Point(constants.Field.Width / 3, 3*constants.Field.Length / 5)
        ]
        #self.middle_of_their_goal = robocup.Point(0, constants.Field.Length)

    def on_enter_setup(self):
        print('Entering Setup')
        self.add_subbehavior(skills.move.Move(self.offensive_points[0]), 'setupLeft')
        self.add_subbehavior(skills.move.Move(self.offensive_points[1]), 'setupRight')
        self.add_subbehavior(skills.capture.Capture(), 'capture')
        

    def on_exit_setup(self):
        print('Exiting Setup')
        self.remove_all_subbehaviors()

    def on_enter_evaluation(self):
        print('Entering Evaluation')
        shoot_prob, shoot_pt = evaluation.shooting.eval_shot(main.ball().pos , main.our_robots())
        print("Best target for robot is , ", shoot_pt)
        print("Prob is , ", shoot_prob)
        pass0_sp, p0_pt = evaluation.shooting.eval_shot(self.offensive_points[0] , main.our_robots())
        print("Best target for p0 (left) is , ", p0_pt)
        print("Prob is , ", pass0_sp)
        pass1_sp, p1_pt = evaluation.shooting.eval_shot(self.offensive_points[1] , main.our_robots())
        print("Best target for p1 (right) is , ", p1_pt)
        print("Prob is , ", pass1_sp)

        if pass0_sp > shoot_prob:
            pass0_pp = evaluation.passing.eval_pass(main.ball().pos, self.offensive_points[0])
        else:
            pass0_pp = 0
        if pass1_sp > shoot_prob:
            pass1_pp = evaluation.passing.eval_pass(main.ball().pos, self.offensive_points[1])
        else:
            pass1_pp = 0
        prob_0 = pass0_pp*pass0_sp
        prob_1 = pass1_pp*pass1_sp

        max_p = max(shoot_prob, prob_0, prob_1)
        if prob_1 == max_p:
            self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.offensive_points[1]), 'Setup Pass')
            self.passing = True
            self.shot_pt = p1_pt
        elif prob_0 == max_p:
            self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.offensive_points[0]), 'Setup Pass')
            self.passing = True
            self.shot_pt = p0_pt
        else:
            self.shot_pt = shoot_pt

    def execute_evaluation(self):
        pass
        #print('Evaluating')

    def on_exit_evaluation(self):
        print('Exiting Evaluation')
        self.passing = False

    def on_enter_passing(self):
        print('Entering Passing')

    def execute_passing(self):
        pass
        #print('Passing')

    def on_exit_passing(self):
        print('Exiting Passing')

    def on_enter_shooting(self):
        print('Entering Shooting')
        shot = skills.pivot_kick.PivotKick()
        shot.target = self.shot_pt
        self.add_subbehavior(shot, 'Shoot Ball')

    def on_exit_shooting(self):
        print('Exiting Shooting')
        pass

    def execute_shooting(self):
        #print('Shooting')
        pass
