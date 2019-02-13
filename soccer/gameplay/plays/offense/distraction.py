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


class distraction(standard_play.StandardPlay):

    class State (enum.Enum):
        setup = 1, 'capture ball and move distractor and striker into position'
        optional_adjustment = 2, 'setup a pass to the center right'
        cetner_pass = 3, 'pass to the center right of the field'
        passing = 4, 'pass from first capture to distractor'
        cross = 5, 'pass from distractor to striker'
        shoot = 6, 'shoot at goal'
    
    def __init__(self):
        super().__init__(continuous=False)  

        self.ball_is_far = False
        self.distraction_point = robocup.Point(0.40*constants.Field.Width, 0.95*constants.Field.Length) #the first distraction point
        self.distraction_recieve_pass_point = robocup.Point(0.40*constants.Field.Width, 0.8*constants.Field.Length) #the second distraction point
        self.striker_point = robocup.Point(-0.40*constants.Field.Width, 0.9*constants.Field.Length) #striker's position
        self.center = robocup.Point(0.5*constants.Field.Width,0.5*constants.Field.Length) #center of field position, used if ball is far
        self.distract_box_min = robocup.Point(0.33*constants.Field.Width/2,(7/9)*constants.Field.Length)
        self.distract_box_max = robocup.Point(constants.Field.Width/2, constants.Field.Length)
        self.striker_box_min = robocup.Point(-0.33*constants.Field.Width/2,(7/9)*constants.Field.Length)
        self.striker_box_max = robocup.Point(-constants.Field.Width/2, constants.Field.Length)

        states = [distraction.State.setup, distraction.State.optional_adjustment, distraction.State.cetner_pass,
         distraction.State.passing, distraction.State.cross, distraction.State.shoot]

        for i in states:
            self.add_state(i, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                        distraction.State.setup, 
                        lambda: True, 
                        'immediately')

        self.add_transition(distraction.State.setup,
                        distraction.State.passing, 
                        lambda: not self.ball_is_far,
                        'capture the ball and pass to a distracting robot if the ball is not far away')

        #optional_adjustment can be skipped if ball is not far from goal
        self.add_transition(distraction.State.setup, 
                        distraction.State.optional_adjustment, 
                        lambda: self.ball_is_far,
                        'if the ball is far away, then go from capturing the ball to getting robots set up to recieve a closer pass')

        self.add_transition(distraction.State.optional_adjustment,
                        distraction.State.cetner_pass, 
                        lambda: self.subbehavior_with_name('move half').is_done_running() and self.subbehavior_with_name('make striker stay').is_done_running() , 
                        'go from setting up a close pass to making the pass towards the center of the field')

        self.add_transition(distraction.State.cetner_pass,
                        distraction.State.passing, 
                        lambda: self.subbehavior_with_name('center pass').is_done_running(), 
                        'after getting the ball to the center of the field, pass to the distracting robot')

        self.add_transition(distraction.State.passing, 
                        distraction.State.cross, lambda: 
                        (self.has_subbehavior_with_name('distract pass') and self.subbehavior_with_name('distract pass').is_done_running()) or 
                        (self.has_subbehavior_with_name('get close ball') and self.subbehavior_with_name('get close ball').is_done_running()), 
                        'have the distracrting robot capture the ball and go to passing to the striker')

        self.add_transition(distraction.State.passing,
                        distraction.State.shoot, lambda: 
                        (self.has_subbehavior_with_name('striker pass') and self.subbehavior_with_name('striker pass').is_done_running() ) or 
                        (self.has_subbehavior_with_name('striker get close ball') and self.subbehavior_with_name('striker get close ball').is_done_running()),
                        'if the ball is already close to the ball, then go from the striker captureing the ball to shooting the ball')

        self.add_transition(distraction.State.cross,
                        distraction.State.shoot, lambda: 
                        self.subbehavior_with_name('pass to striker').is_done_running() or (self.has_subbehavior_with_name('capture 3') and 
                        self.subbehavior_with_name('capture 3').is_done_running()) , 
                        'go from the striker receiving the pass to shooting the ball')

        self.add_transition(distraction.State.shoot,
                        distraction.State.setup, lambda: 
                        self.has_subbehavior_with_name('shooting') and self.subbehavior_with_name('shooting').is_done_running(), 
                        'repeat')

    def on_enter_setup(self):
        #capture ball and get striker and distractor in position
        self.add_subbehavior(skills.capture.Capture(), 'capture1', required = True)
        self.add_subbehavior(skills.move.Move(self.striker_point), 'striker moves', required = False, priority = 10)
        self.add_subbehavior(skills.move.Move(self.distraction_recieve_pass_point), 'distract moves', required = True)
        #if the ball is too far away to pass across the field
        self.ball_is_far = main.ball().pos.y < (0.4*constants.Field.Length)
        
    def on_exit_setup(self):
        # print(self.distracterbox)
        # print(main.ball().pos == self.distracterbox)
        self.remove_all_subbehaviors()
        
    def on_enter_optional_adjustment(self):
        #if the ball is too far then the distractor moves to the center
        self.add_subbehavior(skills.capture.Capture(), 'capture 2', required = True)
        self.add_subbehavior(skills.move.Move(self.center), 'move half', required = True)
        self.add_subbehavior(skills.move.Move(self.striker_point), 'make striker stay', required = True)
        
    def on_exit_optional_adjustment(self):
        self.remove_all_subbehaviors()

    def on_enter_cetner_pass(self):
        #pass the ball to the robot in the center and move a robot to the distract position 
        self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.center), 'center pass', required = True)
        self.add_subbehavior(skills.move.Move(self.distraction_recieve_pass_point), 'move back to distract', required = False, priority = 10)

    def on_exit_cetner_pass(self):
        self.remove_all_subbehaviors()

    def on_enter_passing(self):
        #either pass to striker or distracter depending on shot chance        
        pass_to_distract_chance = evaluation.passing.eval_pass( main.ball().pos, self.distraction_recieve_pass_point, main.our_robots() )
        pass_to_striker_chance = evaluation.passing.eval_pass( main.ball().pos, self.striker_point, main.our_robots() )
        shot_of_striker_chance1 = evaluation.shooting.eval_shot( self.striker_point, main.our_robots() )

        if self.distract_box_max.x > main.ball().pos.x > self.distract_box_min.x and self.distract_box_max.y > main.ball().pos.y > self.distract_box_min.y:
            #if the ball is already near the distracter, then no pass will occur and the distracter will just cpature the ball
            self.add_subbehavior(skills.capture.Capture(), 'get close ball', required = True)
        elif self.striker_box_max.x < main.ball().pos.x < self.striker_box_min.x and self.striker_box_max.y > main.ball().pos.y > self.striker_box_min.y:
            #if the ball is already near the striker, then no pass will occur and the striker will just cpature the ball
            self.add_subbehavior(skills.capture.Capture(), 'striker get close ball', required = True)
        else:
            if pass_to_distract_chance >= pass_to_striker_chance*shot_of_striker_chance1:
                self.add_subbehavior(skills.move.Move(self.striker_point), 'make striker stay again', required = True)
                self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.distraction_recieve_pass_point), 'distract pass', required = True)
            else:
                self.add_subbehavior(skills.move.Move(self.distraction_point), 'make striker stay again', required = True)
                self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.striker_point), 'striker pass', required = True) 

    def on_exit_passing(self):
        self.remove_all_subbehaviors()

    def on_enter_cross(self):
        #if the ball is passed to the distractor the ball is passed to the striker, as the third robot moves to the right to distract more
        pass_to_striker_chance = evaluation.passing.eval_pass(main.ball().pos, self.striker_point, main.our_robots() )
        if self.striker_box_max.x < main.ball().pos.x < self.striker_box_min.x and self.striker_box_max.y > main.ball().pos.y > self.striker_box_min.y:
            #if the ball is already near the striker, then no pass will occur and the striker will just cpature the ball
            self.add_subbehavior(skills.capture.Capture(), 'capture 3', required = True)
        else:
            self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.striker_point), 'pass to striker', required = True)
            self.add_subbehavior(skills.move.Move(self.distraction_point), 'move distract', required = False, priority = 10)
            self.add_subbehavior(skills.move.Move(self.distraction_recieve_pass_point), 'shift right', required = False, priority = 10)

    def on_exit_cross(self):
        self.remove_all_subbehaviors()

    def on_enter_shoot(self):
        #Depending on shot and pass chances,
        #the striker will shoot
        #or
        #the striker will pass to the distractor and the distractor will shoot
        pass_striker_to_distractor_chance = evaluation.passing.eval_pass( self.striker_point, self.distraction_recieve_pass_point, main.our_robots() )
        shot_of_striker_chance = evaluation.shooting.eval_shot( self.striker_point, main.our_robots() )
        shot_of_distractor_chance = evaluation.shooting.eval_shot( self.distraction_recieve_pass_point, main.our_robots() )

        if pass_striker_to_distractor_chance*shot_of_distractor_chance > shot_of_striker_chance:
                self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.distraction_recieve_pass_point), 'distract pass', required = True)
                self.add_subbehavior(skills.move.Move(self.striker_point), 'make striker stay 3', required = True)
                self.add_subbehavior(skills.pivot_kick.PivotKick(), 'shooting', required = True)
        else:
                self.add_subbehavior(skills.move.Move(self.distraction_recieve_pass_point), 'make striker stay again', required = True)
                self.add_subbehavior(skills.move.Move(self.distraction_point), 'make distracor stay', required = False, priority = 10)
                self.add_subbehavior(skills.pivot_kick.PivotKick(), 'shooting', required = True)

    def on_exit_shoot(self):
        self.remove_all_subbehaviors()
