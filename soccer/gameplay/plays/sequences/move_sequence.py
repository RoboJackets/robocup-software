import tactics.behavior_sequence.BehaviorSequence

class MoveSequence(tactics.behavior_sequence.BehaviorSequence):
    
    def __init__(self):
        super().__init__()
        
        #Add test move behaviors
        self.behaviors = [skills.move.Move(robocup.Point(0, 1)),
         skills.move.Move(robocup.Point(0, 2)),
         skills.move.Move(robocup.Point(0, 3)),
         skills.move.Move(robocup.Point(-3, 0))]

