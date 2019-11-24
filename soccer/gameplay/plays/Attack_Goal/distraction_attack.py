import main
import robocup
import behavior
import constants
import enum
import standard_play
import evaluation
import tactics.coordinated_pass
import skills.move
import skills.capture
import situational_play_selection


class Distraction(standard_play.StandardPlay):
    

    _situationList = [
        situational_play_selection.SituationalPlaySelector.Situation.ATTACK_GOAL,
        situational_play_selection.SituationalPlaySelector.Situation.OFFENSIVE_SCRAMBLE
    ] # yapf: disable


    class State(enum.Enum):
        setup = 1, 'capture ball and move distractor and striker into position'
        optional_adjustment = 2, 'capture and setup a pass to the center right'
        center_pass = 3, 'pass to the center right of the field'
        passing = 4, 'pass from first capture to distractor'
        cross = 5, 'pass from distractor to striker'
        shoot = 6, 'shoot at goal'

    def __init__(self):
        super().__init__(continuous=False)

        for s in Distraction.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.ball_is_far = main.ball().pos.y < (0.4 * constants.Field.Length)

        #the first Distraction point
        self.Distraction_point = robocup.Point(0.40 * constants.Field.Width,
                                               0.95 * constants.Field.Length)

        #the second Distraction point
        self.Distraction_recieve_pass_point = robocup.Point(
            0.40 * constants.Field.Width, 0.8 * constants.Field.Length)

        #striker's position
        self.striker_point = robocup.Point(-0.40 * constants.Field.Width,
                                           0.9 * constants.Field.Length)

        #center of field position, used if ball is far
        self.center = robocup.Point(0.5 * constants.Field.Width,
                                    0.5 * constants.Field.Length)

        self.distract_box = robocup.Rect(
            robocup.Point(0.33 * constants.Field.Width / 2,
                          (7 / 9) * constants.Field.Length),
            robocup.Point(constants.Field.Width, constants.Field.Length))

        self.striker_box = robocup.Rect(
            robocup.Point(-0.33 * constants.Field.Width / 2,
                          (7 / 9) * constants.Field.Length),
            robocup.Point(-constants.Field.Width, constants.Field.Length))

        self.distracter_get_close_ball = self.distract_box.contains_point(
            main.ball().pos)
        self.striker_get_close_ball = self.striker_box.contains_point(
            main.ball().pos)

        self.pass_striker_instead = False
        self.dont_shoot = False

        #If ball is not far
        self.add_transition(
            behavior.Behavior.State.start,
            Distraction.State.setup, lambda: not self.ball_is_far,
            'capture ball')

        #If the ball is near the distracter position, then skip steps that get it to that position
        self.add_transition(
            Distraction.State.setup,
            Distraction.State.cross, lambda: self.distracter_get_close_ball and
            self.subbehavior_with_name('capture').is_done_running(),
            'distracter get ball')

        #If the ball is near the strikers position, then skip the steps that get it to that position
        self.add_transition(
            Distraction.State.setup,
            Distraction.State.shoot, lambda: self.striker_get_close_ball and
            self.subbehavior_with_name('capture').is_done_running(),
            'striker get ball')

        #If the ball is far away, then go from capturing the ball to getting robots set up to recieve a closer pass
        self.add_transition(
            behavior.Behavior.State.start,
            Distraction.State.optional_adjustment, lambda: self.ball_is_far,
            'setup close pass')

        #Capture the ball and pass to a distracting robot if the ball is not far away
        self.add_transition(
            Distraction.State.setup, Distraction.State.passing, lambda: self.
            subbehavior_with_name('capture').is_done_running(), 'pass')

        #If the chance to pass and score with the striker is higher than passing to the Distraction recieve pass position
        self.add_transition(
            Distraction.State.setup,
            Distraction.State.cross, lambda: self.distracter_get_close_ball and
            self.subbehavior_with_name('capture').is_done_running(), 'cross')

        #Go from setting up a close pass to making the pass towards the center of the field
        self.add_transition(
            Distraction.State.optional_adjustment,
            Distraction.State.center_pass, lambda: self.subbehavior_with_name(
                'move half').is_done_running() and self.subbehavior_with_name(
                    'capture 2').is_done_running(), 'closer pass')

        #After getting the ball to the center right of the field, pass to the distracting robot
        self.add_transition(
            Distraction.State.center_pass,
            Distraction.State.passing, lambda: self.subbehavior_with_name(
                'center pass').is_done_running(), 'pass upfield')

        #Have the distracrting robot capture the ball and go to passing to the striker
        self.add_transition(
            Distraction.State.passing, Distraction.State.cross, lambda:
            (self.has_subbehavior_with_name('distract pass') and self.
             subbehavior_with_name('distract pass').is_done_running(
             )) or (self.has_subbehavior_with_name('get close ball') and self.
                    subbehavior_with_name('get close ball').is_done_running()),
            'have the distracrting robot capture the ball and go to passing to the striker'
        )

        #The striker gets the ball, either through pass or capturing, and goes to shoot
        self.add_transition(
            Distraction.State.passing, Distraction.State.shoot, lambda: (
                self.has_subbehavior_with_name('striker pass') and self.
                subbehavior_with_name('striker pass').is_done_running()) or
            (self.has_subbehavior_with_name('striker get close ball') and self.
             subbehavior_with_name('striker get close ball').is_done_running(
             )) or (self.has_subbehavior_with_name('get close ball') and self.
                    subbehavior_with_name('get close ball').is_done_running()),
            'stiker get ball')

        #go from the striker receiving the cross to shooting the ball
        self.add_transition(
            Distraction.State.cross,
            Distraction.State.shoot, lambda: self.subbehavior_with_name(
                'pass to striker').is_done_running(), 'cross shoot')

        #Don't shoot if shoot chance is too low, and pass back to distracter
        self.add_transition(Distraction.State.shoot,
                            Distraction.State.passing, lambda: self.dont_shoot,
                            'striker passes ')

        self.add_transition(
            Distraction.State.shoot, Distraction.State.setup, lambda: self.
            subbehavior_with_name('shooting').is_done_running(), 'repeat')

    def on_enter_setup(self):
        self.remove_all_subbehaviors()
        #capture ball and get striker and distractor in position
        self.add_subbehavior(
            skills.capture.Capture(), 'capture', required=True)

        if not self.distracter_get_close_ball:
            self.add_subbehavior(
                skills.move.Move(self.Distraction_recieve_pass_point),
                'distract moves',
                required=True)

        if not self.striker_get_close_ball:
            self.add_subbehavior(
                skills.move.Move(self.striker_point),
                'striker moves',
                required=False,
                priority=10)

    def on_enter_optional_adjustment(self):
        self.remove_all_subbehaviors()
        #if the ball is too far then the distractor moves to the center
        self.add_subbehavior(
            skills.capture.Capture(), 'capture 2', required=True)
        self.add_subbehavior(
            skills.move.Move(self.center), 'move half', required=True)
        self.add_subbehavior(
            skills.move.Move(self.striker_point),
            'make striker stay',
            required=True)

    def on_enter_center_pass(self):
        self.remove_all_subbehaviors()
        #pass the ball to the robot in the center and move a robot to the distract position 
        self.add_subbehavior(
            tactics.coordinated_pass.CoordinatedPass(self.center),
            'center pass',
            required=True)
        self.add_subbehavior(
            skills.move.Move(self.Distraction_recieve_pass_point),
            'move back to distract',
            required=False,
            priority=10)

    def on_enter_passing(self):
        self.remove_all_subbehaviors()
        #either pass to striker or distracter depending on shot chance        
        pass_to_distract_chance = evaluation.passing.eval_pass(
            main.ball().pos, self.Distraction_recieve_pass_point,
            main.our_robots())
        pass_to_striker_chance = evaluation.passing.eval_pass(
            main.ball().pos, self.striker_point, main.our_robots())
        shot_of_striker_chance = evaluation.shooting.eval_shot(
            self.striker_point, main.our_robots())

        if pass_to_distract_chance <= pass_to_striker_chance * shot_of_striker_chance:
            pass_striker_instead = True

        self.add_subbehavior(
            skills.move.Move(self.striker_point),
            'make striker stay again',
            required=True)
        self.add_subbehavior(
            tactics.coordinated_pass.CoordinatedPass(
                self.Distraction_recieve_pass_point),
            'distract pass',
            required=True)

    def on_enter_cross(self):
        self.remove_all_subbehaviors()
        #if the ball is passed to the distractor the ball is passed to the striker, as the third robot moves to the right to distract more
        #add chip here if pass chance is low
        self.add_subbehavior(
            tactics.coordinated_pass.CoordinatedPass(self.striker_point),
            'pass to striker',
            required=True)
        self.add_subbehavior(
            skills.move.Move(self.Distraction_point),
            'shift right',
            required=False,
            priority=10)

    def on_enter_shoot(self):
        self.remove_all_subbehaviors()
        #Depending on shot and pass chances,
        #the striker will shoot
        #or
        #the striker will pass to the distractor and the distractor will shoot
        pass_striker_to_distractor_chance = evaluation.passing.eval_pass(
            self.striker_point, self.Distraction_recieve_pass_point,
            main.our_robots())
        shot_of_striker_chance = evaluation.shooting.eval_shot(
            self.striker_point, main.our_robots())
        shot_of_distractor_chance = evaluation.shooting.eval_shot(
            self.Distraction_recieve_pass_point, main.our_robots())

        if pass_striker_to_distractor_chance * shot_of_distractor_chance > shot_of_striker_chance:
            dont_shoot = True

        self.add_subbehavior(
            skills.move.Move(self.Distraction_recieve_pass_point),
            'make first distractor stay again',
            required=True)
        self.add_subbehavior(
            skills.move.Move(self.Distraction_point),
            'make distracor stay',
            required=False,
            priority=10)
        self.add_subbehavior(
            skills.pivot_kick.PivotKick(), 'shooting', required=True)



    @classmethod
    def score(cls):
        score = super().score()

        #If the score from the super function is valid, use that with some offset
        if (score != float("inf")):
            scoreOffset = 0
            return score + scoreOffset
        else:
            return 10 if main.game_state().is_playing() else float("inf")




