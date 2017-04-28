import single_robot_sequence
import role_assignment
import skills.move
import behavior

#Moves a SINGLE robot in a sequence of points
class MoveSequence(single_robot_sequence.SingleRobotSequence):
    # @param positions_that_are_in_sequence list of positions to move to, in order 
    def __init__(self, positions_that_are_in_sequence = []):
        super().__init__()
        self.positions_that_are_in_sequence = positions_that_are_in_sequence

    def execute_start(self):
        super().on_enter_start()
        if len(self.positions_that_are_in_sequence) > 0:
            #translate pos list into behaviors list
            self.behaviors = list(map(lambda pos: skills.move.Move(pos), self.positions_that_are_in_sequence))

    @property
    def positions_that_are_in_sequence(self):
        return self._positions_that_are_in_sequence

    #Note: restarts sequence when called
    @positions_that_are_in_sequence.setter
    def positions_that_are_in_sequence(self, value):
        self._positions_that_are_in_sequence = value
        self.transition(behavior.Behavior.State.start)
