import main
import robocup
import math
import constants
import evaluation.defensive_positioning

class OffensiveDefense(composite_behavior.CompositeBehavior):
    class State(Enum):
        # Block shots/passes
        blocking = 1
        # Collect the ball when it is lost
        collecting = 2

    # defender_priorities should have a length of two and contains the priorities for the two defender
    def __init__(self):
        super().__init__(continuous=True)

        self.marks = []
        self.floating_def = []

        self.free_pos = robocup.Point(0,0)
        self.mark_bots = [None, None]

        self.block_dist = 0.3
        self.block_angle_coeff = 0.5

        for s in OffensiveDefense.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            OffensiveDefense.State.blocking,
                            lambda: True,
                            'immediately')

        self.add_transition(OffensiveDefense.State.blocking,
                            OffensiveDefense.State.collecting,
                            lambda: False,
                            'Collect Ball')

        self.add_transition(OffensiveDefense.State.collecting,
                            OffensiveDefense.State.blocking,
                            lambda: False,
                            'Block again')

    def on_enter_blocking(self):
        self.free_pos, self.mark_bots[0], self.mark_bots[1] = evaluation.defensive_positioning(self.floating_def)

        self.marks.extend(skills.mark.Mark())
        self.marks.extend(skills.mark.Mark())

        # Set mark robot
        # Set position to block at
        for i in range(0,1):
            self.marks[i].mark_robot(self.mark_bots[i])
            point = get_block_pos(self.marks[i].mark_robot())

    def execute_blocking(self):
        # Updates block pos
        for mark in self.marks:
            point = get_block_pos(mark.mark_robot())


    def on_exit_blocking(self):
        pass

    def get_block_pos(bot):
        # Get predicted angle of shot
        # Get goal to bot
        # Place point x dist away
        predicted = predict_kick_direction(bot)
        actual = bot.pos.angle()

        angle = self.block_angle_coeff*predicted + (1-self.block_angle_coeff)*actual

        # Get line between bot  at angle
        # get point X dist away
        pass