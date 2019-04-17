import robocup
import constants
import play
import skills
import tactics
import evaluation
import main
import enum
import behavior

class CrossKick(play.Play):

    class State(enum.Enum):
    	collecting = 0;
    	transitioning = 1;
    	crossing = 2;
    	finishing = 3;
    	retransitioning = 4;
    	recrossing = 5;
    	transitioningLeft = 6;
    	crossingLeft = 7;
    	transitioningDeep = 8;
    	transitioningDL = 9;


    def __init__(self):
        super().__init__(continuous=False)

        for s in CrossKick.State:
            self.add_state(s, behavior.Behavior.State.running)

        ready_to_finish = (main.ball().pos.y >= constants.Field.Length * 0.8)
        isLeft = (main.ball().pos.x < 0) & (main.ball().pos.y >= constants.Field.Length * 0.45)
        isDeep = (main.ball().pos.y < constants.Field.Length * 0.45) & (not isLeft)
        isRegular = (not isLeft) & (not isDeep)
        isDeepLeft = (isLeft) & (isDeep)
        
        #collected = (self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed)


        '''
        self.add_subbehavior(capture, "Capture")
        self.add_subbehavior(move_flank, "move to flank")
        self.add_subbehavior(move_near_box, "move near box")  
        '''         

        self.add_transition(behavior.Behavior.State.start, 
                            CrossKick.State.collecting, lambda: True, 
                            'immediately')
        self.add_transition(CrossKick.State.collecting, 
                            CrossKick.State.transitioning, lambda: (isRegular) & (self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed),
                            'ball collected')
        self.add_transition(CrossKick.State.collecting, 
        	                CrossKick.State.transitioningLeft, lambda: (isLeft) & (self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed), 
        	                'collected on left side')
        self.add_transition(CrossKick.State.collecting, 
        	                CrossKick.State.transitioningDeep, lambda: (isDeep) & (self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed), 
        	                'collected on own side')
        self.add_transition(CrossKick.State.collecting, 
        	                CrossKick.State.transitioningDL, lambda: (isDeepLeft) & (self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed), 
        	                'collected on own side')
        self.add_transition(CrossKick.State.collecting, 
        	                CrossKick.State.finishing, lambda: ready_to_finish, 'ball collected near the goal, ready to finish')
        self.add_transition(CrossKick.State.transitioning, 
                            CrossKick.State.crossing, lambda: self.subbehavior_with_name('pass to flank').state == behavior.Behavior.State.completed, 
                            'robots in position')
        self.add_transition(CrossKick.State.transitioningDeep, 
                            CrossKick.State.transitioning, lambda: self.subbehavior_with_name('pass to flank').state == behavior.Behavior.State.completed, 
                            'robots in position')
        self.add_transition(CrossKick.State.transitioningDL, 
                            CrossKick.State.transitioningLeft, lambda: self.subbehavior_with_name('pass to flank').state == behavior.Behavior.State.completed, 
                            'robots in position')
        self.add_transition(CrossKick.State.transitioningLeft, 
                            CrossKick.State.crossingLeft, lambda: self.subbehavior_with_name('pass to flank').state == behavior.Behavior.State.completed, 
                            'robots in position on left')
        self.add_transition(CrossKick.State.transitioning, 
        	                CrossKick.State.finishing, lambda: ready_to_finish & (self.subbehavior_with_name('pass to flank').state == behavior.Behavior.State.completed), 'why not shoot if you are in range')
        self.add_transition(CrossKick.State.transitioningLeft, 
        	                CrossKick.State.finishing, lambda: ready_to_finish & (self.subbehavior_with_name('pass to flank').state == behavior.Behavior.State.completed), 'why not shoot if you are in range')
        self.add_transition(CrossKick.State.transitioning, 
        	                CrossKick.State.retransitioning, lambda: self.subbehavior_with_name('pass to flank').state == behavior.Behavior.State.failed, 'restart')
        self.add_transition(CrossKick.State.transitioningLeft, 
        	                CrossKick.State.retransitioning, lambda: self.subbehavior_with_name('pass to flank').state == behavior.Behavior.State.failed, 'restart')
        self.add_transition(CrossKick.State.transitioningDeep, 
        	                CrossKick.State.retransitioning, lambda: (self.subbehavior_with_name('pass to flank').state == behavior.Behavior.State.failed), 'restart')
        self.add_transition(CrossKick.State.retransitioning, 
                            CrossKick.State.crossing, lambda: (not isLeft) & (self.subbehavior_with_name('recapture').state == behavior.Behavior.State.completed), 
                            'robots in position')
        self.add_transition(CrossKick.State.retransitioning, 
                            CrossKick.State.transitioningDeep, lambda: (isDeep) & (self.subbehavior_with_name('recapture').state == behavior.Behavior.State.completed), 
                            'robots in position')
        self.add_transition(CrossKick.State.retransitioning, 
                            CrossKick.State.transitioningDL, lambda: (isDeepLeft) & (self.subbehavior_with_name('recapture').state == behavior.Behavior.State.completed), 
                            'robots in position')
        self.add_transition(CrossKick.State.retransitioning, 
                            CrossKick.State.crossingLeft, lambda: isLeft & (self.subbehavior_with_name('recapture').state == behavior.Behavior.State.completed), 
                            'robots in position')
        self.add_transition(CrossKick.State.retransitioning, 
                            CrossKick.State.finishing, lambda: ready_to_finish & (self.subbehavior_with_name('recapture').state == behavior.Behavior.State.completed), 
                            'put it in the back of the net')
        self.add_transition(CrossKick.State.crossing, 
                            CrossKick.State.finishing, lambda: (self.subbehavior_with_name('box pass').state == behavior.Behavior.State.completed),
                            'finishing the play')
        self.add_transition(CrossKick.State.crossingLeft, 
                            CrossKick.State.finishing, lambda: (self.subbehavior_with_name('box pass').state == behavior.Behavior.State.completed),
                            'finishing the play')
        self.add_transition(CrossKick.State.crossing, 
        	                CrossKick.State.recrossing, lambda: self.subbehavior_with_name('box pass').state == behavior.Behavior.State.failed, 'interrupted')
        self.add_transition(CrossKick.State.crossingLeft, 
        	                CrossKick.State.recrossing, lambda: self.subbehavior_with_name('box pass').state == behavior.Behavior.State.failed, 'interrupted')
        self.add_transition(CrossKick.State.crossing,
        	                CrossKick.State.collecting, lambda: False, 'should not restart when in scoring position')
        self.add_transition(CrossKick.State.crossingLeft,
        	                CrossKick.State.collecting, lambda: False, 'should not restart when in scoring position')
        self.add_transition(CrossKick.State.recrossing,
        	                CrossKick.State.collecting, lambda: False, 'should not restart when in scoring position')
        self.add_transition(CrossKick.State.recrossing, 
                            CrossKick.State.finishing, lambda: ready_to_finish & (self.subbehavior_with_name('recapture').state == behavior.Behavior.State.completed), 
                            'scoring position')
        self.add_transition(CrossKick.State.recrossing, 
                            CrossKick.State.transitioning, lambda: (not ready_to_finish) & (self.subbehavior_with_name('recapture').state == behavior.Behavior.State.completed), 
                            'not in scoring position')
        #self.add_transition(CrossKick.State.finishing, 
        	                #behavior.Behavior.State.completed, lambda: self.subbehavior_with_name('shoot').state == behavior.Behavior.State.completed, 'shot my shot')
        '''
        self.add_transition(CrossKick.State.transitioning, 
                            CrossKick.State.crossing, lambda: self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed, 
                            'robots in position')'''
        '''self.add_transition(CrossKick.State.transitioning, 
        	                CrossKick.State.collecting, lambda: self.subbehavior_with_name('pass to flank').state == behavior.Behavior.State.failed, 'restart')'''
        '''self.add_transition(CrossKick.State.crossing, 
                            CrossKick.State.finishing, lambda: self.subbehavior_with_name('capture cross').state == behavior.Behavior.State.completed,
                            'finishing the play')'''
        '''self.add_transition(CrossKick.State.crossing, 
        	                CrossKick.State.collecting, lambda: self.subbehavior_with_name('box pass').state == behavior.Behavior.State.failed, 'restart')'''
        '''self.add_transition(CrossKick.State.finishing, 
        	                behavior.Behavior.State.completed, lambda: self.subbehavior_with_name('shoot').state == behavior.Behavior.State.completed, 'shot my shot')'''

    def on_enter_collecting(self):
        self.remove_all_subbehaviors()

        capture = skills.capture.Capture()
        self.add_subbehavior(capture, 'capture', required=True) 

    def on_exit_collecting(self):
        #self.captureRobot = self.subbehavior_with_name('capture')
        self.remove_all_subbehaviors()

    
    def on_enter_transitioning(self): 
        self.remove_all_subbehaviors()
        
        point_flank = robocup.Point(constants.Field.Width * 0.4, constants.Field.Length *.78)
        #move_flank = skills.move.Move(point_flank)
        pass_to_flank = tactics.coordinated_pass.CoordinatedPass(point_flank)
        #capture = skills.capture.Capture()

        self.add_subbehavior(pass_to_flank, 'pass to flank', required=True)
        #self.add_subbehavior(move_flank, 'move to flank', required=True)
        #self.add_subbehavior(capture, 'capture', required=True) 

    def on_exit_transitioning(self):
        self.remove_all_subbehaviors()

    def on_enter_transitioningLeft(self): 
        self.remove_all_subbehaviors()
        
        point_flank = robocup.Point(constants.Field.Width * -0.4, constants.Field.Length *.78)
        #move_flank = skills.move.Move(point_flank)
        pass_to_flank = tactics.coordinated_pass.CoordinatedPass(point_flank)
        #capture = skills.capture.Capture()

        self.add_subbehavior(pass_to_flank, 'pass to flank', required=True)
        #self.add_subbehavior(move_flank, 'move to flank', required=True)
        #self.add_subbehavior(capture, 'capture', required=True) 

    def on_exit_transitioningLeft(self):
        self.remove_all_subbehaviors()
   
    def on_enter_transitioningDeep(self): 
        self.remove_all_subbehaviors()
        
        point_flank = robocup.Point(constants.Field.Width * 0.4, constants.Field.Length *.5)
        #move_flank = skills.move.Move(point_flank)
        pass_to_flank = tactics.coordinated_pass.CoordinatedPass(point_flank)
        #capture = skills.capture.Capture()

        self.add_subbehavior(pass_to_flank, 'pass to flank', required=True)
        #self.add_subbehavior(move_flank, 'move to flank', required=True)
        #self.add_subbehavior(capture, 'capture', required=True) 

    def on_exit_transitioningDeep(self):
        self.remove_all_subbehaviors()

    def on_enter_transitioningDL(self): 
        self.remove_all_subbehaviors()
        
        point_flank = robocup.Point(constants.Field.Width * -0.4, constants.Field.Length *.5)
        #move_flank = skills.move.Move(point_flank)
        pass_to_flank = tactics.coordinated_pass.CoordinatedPass(point_flank)
        #capture = skills.capture.Capture()

        self.add_subbehavior(pass_to_flank, 'pass to flank', required=True)
        #self.add_subbehavior(move_flank, 'move to flank', required=True)
        #self.add_subbehavior(capture, 'capture', required=True) 

    def on_exit_transitioningDL(self):
        self.remove_all_subbehaviors()
    
    def on_enter_crossing(self):
        self.remove_all_subbehaviors()
        
        #capture = skills.capture.Capture()
        #point_near_box = robocup.Point(-.4,9*.83)
        point_in_box = robocup.Point(constants.Field.Width * -0.1, constants.Field.Length * .875)
        #move_near_box = skills.move.Move(point_near_box)
        pass_into_box = tactics.coordinated_pass.CoordinatedPass(point_in_box)
        
        #self.add_subbehavior(move_near_box, "move near box") 
        self.add_subbehavior(pass_into_box, 'box pass', required=True)
        #self.add_subbehavior(capture, 'capture cross') 

    
    def on_exit_crossing(self):
        self.remove_all_subbehaviors()

    def on_enter_crossingLeft(self):
        self.remove_all_subbehaviors()
        
        #capture = skills.capture.Capture()
        #point_near_box = robocup.Point(-.4,9*.83)
        point_in_box = robocup.Point(constants.Field.Width * 0.1, constants.Field.Length * .875)
        #move_near_box = skills.move.Move(point_near_box)
        pass_into_box = tactics.coordinated_pass.CoordinatedPass(point_in_box)
        
        #self.add_subbehavior(move_near_box, "move near box") 
        self.add_subbehavior(pass_into_box, 'box pass', required=True)
        #self.add_subbehavior(capture, 'capture cross') 

    
    def on_exit_crossingLeft(self):
        self.remove_all_subbehaviors()
   
    def on_enter_finishing(self):
        self.remove_all_subbehaviors()
        
        shoot = skills.pivot_kick.PivotKick()

        self.add_subbehavior(shoot, 'shoot', required=True)
        
    def on_exit_finishing(self):
        self.remove_all_subbehaviors()    

    def on_enter_retransitioning(self):
        self.remove_all_subbehaviors()

        capture = skills.capture.Capture()
        self.add_subbehavior(capture, 'recapture', required=True) 
        
    def on_exit_retransitioning(self):
        self.remove_all_subbehaviors()  

    def on_enter_recrossing(self):
        self.remove_all_subbehaviors()

        capture = skills.capture.Capture()
        self.add_subbehavior(capture, 'recapture', required=True) 
        
    def on_exit_recrossing(self):
        self.remove_all_subbehaviors()   

  