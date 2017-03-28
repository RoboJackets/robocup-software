import main
import robocup
import behavior
import constants
import enum
import math

import composite_behavior
import evaluation.defensive_positioning
import tactics.submissive_defensive_forward

# Finds the positions to place each defender
class DefensiveForward(composite_behavior.CompositeBehavior):
    class State(enum.Enum):
        defending = 1

    def __init__(self):
        super().__init__(continuous=True)

        self.marks = []
        self.floating_def = []

        self.free_pos = robocup.Point(0,0)
        self.mark_bots = [None, None]

        self.block_dist = 0.01
        self.block_angle_coeff = 0.5

        for s in DefensiveForward.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            DefensiveForward.State.defending,
                            lambda: True,
                            'immediately')

    # Set the marking locations
    def on_enter_defending(self):
        self.free_pos, self.mark_bots[0], self.mark_bots[1] = evaluation.defensive_positioning.find_defense_positions()

        names = ['mark_main', 'mark_sub']

        for i in range(0, 2):
            self.marks.extend([tactics.submissive_defensive_forward.SubmissiveDefensiveForward()])
            self.add_subbehavior(self.marks[i], names[i], required=True)

            self.marks[i].mark_robot = self.mark_bots[i]
            point = self.get_block_pos(self.mark_bots[i])
            self.marks[i].mark_point = point

        self.floating_def = tactics.submissive_defensive_forward.SubmissiveDefensiveForward()
        self.add_subbehavior(self.floating_def, 'mark_float', required=True)
        self.floating_def.mark_point = self.free_pos

    # Update marking positions
    def execute_defending(self):
        our_bots = [self.marks[0].robot, self.marks[1].robot, self.floating_def.robot]
        self.free_pos, self.mark_bots[0], self.mark_bots[1] = evaluation.defensive_positioning.find_defense_positions(our_bots)

        for i in range(0, 2):
            self.marks[i].mark_robot = self.mark_bots[i]
            #point = self.get_block_pos(self.mark_bots[i])
            #self.marks[i].mark_point = point

        self.floating_def.mark_point = self.free_pos

    def on_exit_defending(self):
        self.remove_all_subbehaviors()

    # Uses their predicted kick direction to block
    def get_block_pos(self, bot):
        # Get predicted angle of shot
        # Get goal to bot
        # Place point x dist away
        predicted = evaluation.defensive_positioning.predict_kick_direction(bot)
        actual = bot.angle

        angle = self.block_angle_coeff*predicted + (1-self.block_angle_coeff)*actual

        x = math.cos(angle)
        y = math.sin(angle)
        pos = robocup.Point(x, y).normalized() + bot.pos

        return None # pos * self.block_dist