import main
import robocup
import behavior
import constants
import enum
import math

import composite_behavior
import evaluation.defensive_positioning
import skills.mark

class OffensiveDefense(composite_behavior.CompositeBehavior):
    class State(enum.Enum):
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

        self.block_dist = 0.01
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
        self.free_pos, self.mark_bots[0], self.mark_bots[1] = evaluation.defensive_positioning.find_defense_positions()

        names = ['mark_main', 'mark_sub']

        for i in range(0, 2):
            self.marks.extend([skills.mark.Mark()])
            self.add_subbehavior(self.marks[i], names[i], required=True)

            self.marks[i].mark_robot = self.mark_bots[i]
            point = self.get_block_pos(self.mark_bots[i])
            self.marks[i].mark_point = point

        self.floating_def = skills.mark.Mark()
        self.floating_def.mark_point = self.free_pos

    def execute_blocking(self):
        # Updates block pos
        self.free_pos, self.mark_bots[0], self.mark_bots[1] = evaluation.defensive_positioning.find_defense_positions(self.mark_bots)

        #for i in range(0, 2):
        #    self.marks[i].mark_robot = self.mark_bots[i]
            #point = self.get_block_pos(self.mark_bots[i])
            #self.marks[i].mark_point = point

        self.floating_def.mark_point = self.free_pos

        pass

    def on_exit_blocking(self):
        self.remove_all_subbehaviors()

    def get_block_pos(self, bot):
        # Get predicted angle of shot
        # Get goal to bot
        # Place point x dist away
        predicted = evaluation.defensive_positioning.predict_kick_direction(bot)
        actual = bot.pos.angle()

        angle = self.block_angle_coeff*predicted + (1-self.block_angle_coeff)*actual

        x = math.cos(angle)
        y = math.sin(angle)
        pos = robocup.Point(x, y).normalized() + bot.pos

        return None # pos * self.block_dist