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


class distraction1(standard_play.StandardPlay):
    

    setup = 1
    optionaladjustment = 2
    centerpass = 3
    passing = 4
    shoot = 5

    class State (enum.Enum):
        setup = 1
        optionaladjustment = 2
        centerpass = 3
        passing = 4
        cross = 5
        shoot = 6
    
    def __init__(self):
        super().__init__(continuous=False)  

        self.ball_is_far = False
        
        self.add_state(distraction1.State.setup,
                        behavior.Behavior.State.running)
        self.add_state(distraction1.State.optionaladjustment,
                        behavior.Behavior.State.running)
        self.add_state(distraction1.State.centerpass,
                        behavior.Behavior.State.running)
        self.add_state(distraction1.State.passing,
                        behavior.Behavior.State.running)
        self.add_state(distraction1.State.cross,
                        behavior.Behavior.State.running)
        self.add_state(distraction1.State.shoot,
                        behavior.Behavior.State.running)
        
        
        self.add_transition(behavior.Behavior.State.start,
                        distraction1.State.setup, lambda: True, 'immediately')

        self.add_transition(distraction1.State.setup,
                        distraction1.State.passing, lambda: not self.ball_is_far,'1-passing')

        #optionaladjustment can be skipped if ball is not far from goal
        self.add_transition(distraction1.State.setup, 
                        distraction1.State.optionaladjustment, lambda: self.ball_is_far and 
                        self.subbehavior_with_name('striker moves').is_done_running(),'1-2')

        self.add_transition(distraction1.State.optionaladjustment,
                        distraction1.State.centerpass, lambda: self.subbehavior_with_name('move half').is_done_running() and 
                        self.subbehavior_with_name('stay').is_done_running() , '2-centerpass')

        self.add_transition(distraction1.State.centerpass,
                        distraction1.State.passing, lambda: self.subbehavior_with_name('center pass').is_done_running(), 'centerpass-passing')

        self.add_transition(distraction1.State.passing, 
                        distraction1.State.cross, lambda: ((self.has_subbehavior_with_name('distract pass') and self.subbehavior_with_name('distract pass').is_done_running()) 
                            or 
                        (self.has_subbehavior_with_name('get close ball') and self.subbehavior_with_name('get close ball').is_done_running())  )
                        , 'passing-crossing')

        self.add_transition(distraction1.State.passing,
                        distraction1.State.shoot, lambda: (self.has_subbehavior_with_name('striker pass') and self.subbehavior_with_name('striker pass').is_done_running() ),'pass-shoot' )

        self.add_transition(distraction1.State.cross,
                        distraction1.State.shoot, lambda: self.subbehavior_with_name('pass to striker').is_done_running()
                        , 'crossing-shooting')

        self.add_transition(distraction1.State.shoot,
                        distraction1.State.setup, lambda: self.has_subbehavior_with_name('shooting') and self.subbehavior_with_name('shooting').is_done_running(), 'repeat')

        
        self.d1 = robocup.Point(0.40*constants.Field.Width, 0.95*constants.Field.Length) #the first distraction point
        self.d2 = robocup.Point(0.40*constants.Field.Width, 0.8*constants.Field.Length) #the second distraction point
        self.s1 = robocup.Point(-0.40*constants.Field.Width, 0.9*constants.Field.Length) #striker's position
        self.center = robocup.Point(0.5*constants.Field.Width,0.5*constants.Field.Length) #center of field position, used if ball is far
        self.distracterbox = robocup.Circle(self.d1, 2)
        

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
        print(self.distracterbox)
        print(main.ball().pos == self.distracterbox)
        self.remove_all_subbehaviors()
        

    def on_enter_optionaladjustment(self):
        #if the ball is too far then the distractor moves to the center
        print("entered setup2")
        self.add_subbehavior(skills.capture.Capture(), 'capture 2')
        self.add_subbehavior(skills.move.Move(self.center), 'move half')
        self.add_subbehavior(skills.move.Move(self.s1), 'stay')
        

    def on_exit_optionaladjustment(self):
        print('exits optional adjustment')
        self.remove_all_subbehaviors()

    def on_enter_centerpass(self):
        #pass the ball to the robot in the center and move a robot to the distract position 
        self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.center), 'center pass')
        self.add_subbehavior(skills.move.Move(self.d2), "move back to distract")


    def on_exit_centerpass(self):
        self.remove_all_subbehaviors()

    def on_enter_passing(self):
        #either pass to striker or distracter depending on shot chance
        print('entered passing')
        
        
        passchance1 = evaluation.passing.eval_pass( main.ball().pos, self.d2, main.our_robots() )
        passchance2 = evaluation.passing.eval_pass( main.ball().pos, self.s1, main.our_robots() )
        passchance3 = evaluation.passing.eval_pass( self.d2, self.s1, main.our_robots() )
        shotchance1 = evaluation.shooting.eval_shot( self.s1, main.our_robots() )

        if main.ball().pos == self.distracterbox:

            self.add_subbehavior(skills.capture.Capture(), 'get close ball')

        else:

            if passchance1 >= passchance2*shotchance1:
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
        self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.s1),'pass to striker')
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
        passchance1 = evaluation.passing.eval_pass( main.ball().pos, self.d2, main.our_robots() )
        passchance2 = evaluation.passing.eval_pass( main.ball().pos, self.s1, main.our_robots() )
        passchance3 = evaluation.passing.eval_pass( self.d2, self.s1, main.our_robots() )
        passchance4 = evaluation.passing.eval_pass( self.s1, self.d2, main.our_robots() )
        shotchance1 = evaluation.shooting.eval_shot( self.s1, main.our_robots() )
        shotchance2 = evaluation.shooting.eval_shot( self.d2, main.our_robots() )

        if passchance4*shotchance2 > shotchance1:
                
                self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.d2), 'distract pass')
                self.add_subbehavior(skills.move.Move(self.s1), 'stay3')
                self.add_subbehavior(skills.pivot_kick.PivotKick(), 'shooting')
                

        else:
                self.add_subbehavior(skills.move.Move(self.d2), 'stay2')
                self.add_subbehavior(skills.move.Move(self.d1), 'stay3')
                self.add_subbehavior(skills.pivot_kick.PivotKick(), 'shooting')
