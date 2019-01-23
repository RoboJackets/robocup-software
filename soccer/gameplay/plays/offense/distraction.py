import robocup
import standard_play
import behavior
import constants
import main
import skills.move
import skills.capture
import enum
import evaluation
import tactics.coordinated_pass
import tactics.defense
import play


class distraction(standard_play.StandardPlay):


    class State (enum.Enum):
        setup = 1
        optional_adjustment = 2
        cetner_pass = 3
        passing = 4
        cross = 5
        shoot = 6
    
    def __init__(self):
        super().__init__(continuous=False)  

        self.ball_is_far = False
        
        self.add_state(distraction.State.setup,
                        behavior.Behavior.State.running)
        self.add_state(distraction.State.optional_adjustment,
                        behavior.Behavior.State.running)
        self.add_state(distraction.State.cetner_pass,
                        behavior.Behavior.State.running)
        self.add_state(distraction.State.passing,
                        behavior.Behavior.State.running)
        self.add_state(distraction.State.cross,
                        behavior.Behavior.State.running)
        self.add_state(distraction.State.shoot,
                        behavior.Behavior.State.running)
        
        
        self.add_transition(behavior.Behavior.State.start,
                        distraction.State.setup, 
                        lambda: True, 
                        'immediately')

        self.add_transition(distraction.State.setup,
                        distraction.State.passing, 
                        lambda: not self.ball_is_far,
                        '1-passing')

        #optional_adjustment can be skipped if ball is not far from goal
        self.add_transition(distraction.State.setup, 
                        distraction.State.optional_adjustment, 
                        lambda: self.ball_is_far and self.subbehavior_with_name('striker moves').is_done_running(),
                        '1-2')

        self.add_transition(distraction.State.optional_adjustment,
                        distraction.State.cetner_pass, 
                        lambda: self.subbehavior_with_name('move half').is_done_running() and self.subbehavior_with_name('stay').is_done_running() , 
                        '2-cetner_pass')

        self.add_transition(distraction.State.cetner_pass,
                        distraction.State.passing, 
                        lambda: self.subbehavior_with_name('center pass').is_done_running(), 
                        'cetner_pass-passing')

        self.add_transition(distraction.State.passing, 
                        distraction.State.cross, lambda: 
                        ((self.has_subbehavior_with_name('distract pass') and self.subbehavior_with_name('distract pass').is_done_running()) or 
                        (self.has_subbehavior_with_name('get close ball') and self.subbehavior_with_name('get close ball').is_done_running())), 
                        'passing-crossing')

        self.add_transition(distraction.State.passing,
                        distraction.State.shoot, lambda: 
                        (self.has_subbehavior_with_name('striker pass') and self.subbehavior_with_name('striker pass').is_done_running() ) or 
                        (self.has_subbehavior_with_name('striker get close ball') and self.subbehavior_with_name('striker get close ball').is_done_running()),
                        'pass-shoot')

        self.add_transition(distraction.State.cross,
                        distraction.State.shoot, lambda: 
                        self.subbehavior_with_name('pass to striker').is_done_running() or (self.has_subbehavior_with_name('capture 3') and 
                        self.subbehavior_with_name('capture 3').is_done_running()) , 
                        'crossing-shooting')

        self.add_transition(distraction.State.shoot,
                        distraction.State.setup, lambda: 
                        self.has_subbehavior_with_name('shooting') and self.subbehavior_with_name('shooting').is_done_running(), 
                        'repeat')

        
        self.d1 = robocup.Point(0.40*constants.Field.Width, 0.95*constants.Field.Length) #the first distraction point
        self.d2 = robocup.Point(0.40*constants.Field.Width, 0.8*constants.Field.Length) #the second distraction point
        self.s1 = robocup.Point(-0.40*constants.Field.Width, 0.9*constants.Field.Length) #striker's position
        self.center = robocup.Point(0.5*constants.Field.Width,0.5*constants.Field.Length) #center of field position, used if ball is far
        self.distract_box_min = robocup.Point(0.33*constants.Field.Width/2,(7/9)*constants.Field.Length)
        self.distract_box_max = robocup.Point(constants.Field.Width/2, constants.Field.Length)
        self.striker_box_min = robocup.Point(-0.33*constants.Field.Width/2,(7/9)*constants.Field.Length)
        self.striker_box_max = robocup.Point(-constants.Field.Width/2, constants.Field.Length)

    def on_enter_setup(self):
        #capture ball and get striker and distractor in position
        print("entered setup")
        self.add_subbehavior(skills.capture.Capture(), 'capture1')
        self.add_subbehavior(skills.move.Move(self.s1), 'striker moves')
        self.add_subbehavior(skills.move.Move(self.d2), 'distract moves')
        #if the ball is too far away to pass across the field
        self.ball_is_far = main.ball().pos.y < (0.4*constants.Field.Length)
        


    def on_exit_setup(self):
        print("exits setup")
        # print(self.distracterbox)
        # print(main.ball().pos == self.distracterbox)
        self.remove_all_subbehaviors()
        

    def on_enter_optional_adjustment(self):
        #if the ball is too far then the distractor moves to the center
        print("entered setup2")
        self.add_subbehavior(skills.capture.Capture(), 'capture 2')
        self.add_subbehavior(skills.move.Move(self.center), 'move half')
        self.add_subbehavior(skills.move.Move(self.s1), 'stay')
        

    def on_exit_optional_adjustment(self):
        print('exits optional adjustment')
        self.remove_all_subbehaviors()

    def on_enter_cetner_pass(self):
        #pass the ball to the robot in the center and move a robot to the distract position 
        self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.center), 'center pass')
        self.add_subbehavior(skills.move.Move(self.d2), "move back to distract")


    def on_exit_cetner_pass(self):
        self.remove_all_subbehaviors()

    def on_enter_passing(self):
        #either pass to striker or distracter depending on shot chance
        print('entered passing')
        
        
        pass_to_distract_chance = evaluation.passing.eval_pass( main.ball().pos, self.d2, main.our_robots() )
        pass_to_striker_chance = evaluation.passing.eval_pass( main.ball().pos, self.s1, main.our_robots() )
        pass_from_distract_to_striker_chance = evaluation.passing.eval_pass( self.d2, self.s1, main.our_robots() )
        shot_of_striker_chance1 = evaluation.shooting.eval_shot( self.s1, main.our_robots() )

        if self.distract_box_max.x > main.ball().pos.x > self.distract_box_min.x and self.distract_box_max.y > main.ball().pos.y > self.distract_box_min.y:

            self.add_subbehavior(skills.capture.Capture(), 'get close ball')
            print("In box")
        elif self.striker_box_max.x < main.ball().pos.x < self.striker_box_min.x and self.striker_box_max.y > main.ball().pos.y > self.striker_box_min.y:
            print("in striker box")
            self.add_subbehavior(skills.capture.Capture(), 'striker get close ball')
        else:
            print("Out box")
            if pass_to_distract_chance >= pass_to_striker_chance*shot_of_striker_chance1:
                self.add_subbehavior(skills.move.Move(self.s1), 'stay2 ')
                
                self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.d2), 'distract pass')
                

            else:
                self.add_subbehavior(skills.move.Move(self.d1), 'stay2')
                
                self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.s1), 'striker pass') 


    def on_exit_passing(self):
        print('exit passing')
        self.remove_all_subbehaviors()

    def on_enter_cross(self):
        #if the ball is passed to the distractor the ball is passed to the striker, as the third robot moves to the right to distract more
        print('enter cross')
        if self.striker_box_max.x < main.ball().pos.x < self.striker_box_min.x and self.striker_box_max.y > main.ball().pos.y > self.striker_box_min.y:
            self.add_subbehavior(skills.capture.Capture(), 'capture 3')
        else:
            self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.s1), 'pass to striker')
            self.add_subbehavior(skills.move.Move(self.d1), 'move distract')
            self.add_subbehavior(skills.move.Move(self.d2), 'shift right')
        

    def on_exit_cross(self):
        print('exit cross')
        self.remove_all_subbehaviors()

    def on_enter_shoot(self):
        #Depending on shot and pass chances,
        #the striker will shoot
        #or
        #the striker will pass to the distractor and the distractor will shoot
        print('enter shoot')
        pass_striker_to_distractor_chance = evaluation.passing.eval_pass( self.s1, self.d2, main.our_robots() )
        shot_of_striker_chance = evaluation.shooting.eval_shot( self.s1, main.our_robots() )
        shot_of_distracotr_chance = evaluation.shooting.eval_shot( self.d2, main.our_robots() )

        if pass_striker_to_distractor_chance*shot_of_distracotr_chance > shot_of_striker_chance:
                
                self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.d2), 'distract pass')
                self.add_subbehavior(skills.move.Move(self.s1), 'stay3')
                self.add_subbehavior(skills.pivot_kick.PivotKick(), 'shooting')
                

        else:
                self.add_subbehavior(skills.move.Move(self.d2), 'stay2')
                self.add_subbehavior(skills.move.Move(self.d1), 'stay3')
                self.add_subbehavior(skills.pivot_kick.PivotKick(), 'shooting')
