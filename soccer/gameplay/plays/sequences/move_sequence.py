import behavior_sequence
import skills.move

class MoveSequence(behavior_sequence.BehaviorSequence):
    def __init__(self, positions = None):
        super().__init__()
        self._positions = [] if positions == None else positions

    def on_enter_start(self):
        super().on_enter_start()
        for pos in self.positions:
            self.behaviors.append(skills.move.Move(pos))

    @property
    def positions(self):
        return self._positions

    #restarts sequence
    @positions.setter
    def positions(self, value):
        self._positions = value
        self.transition(behavior.Behavior.State.start) 
