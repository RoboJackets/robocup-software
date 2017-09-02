import main
import robocup
import behavior
import constants
import enum
import math

import composite_behavior
import evaluation.ball
import evaluation.defensive_positioning
import evaluation.path
import skills.mark
import skills.capture


# Finds the positions to place each defender
class DefensiveForward(composite_behavior.CompositeBehavior):
    class State(enum.Enum):
        # Everyone is defending
        blocking = 1
        # Closest defender is collecting, all others are blocking
        collecting = 2

    def __init__(self):
        super().__init__(continuous=True)

        # Robot queue
        self.defenders = [None, None, None]
        # Single robot collecting
        self.collector = None

        self.zone_def_pos = robocup.Point(0, 0)
        self.mark_bots = [None, None]

        self.block_dist = 0.01
        self.block_angle_coeff = 0.5
        self.names = ['mark_main', 'mark_sub', 'mark_zone']

        self.time_since_call = 0
        # Estimates defensive positions every X ticks
        self.cache_amnt = 5

        for s in DefensiveForward.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            DefensiveForward.State.blocking, lambda: True,
                            'immediately')

        self.add_transition(DefensiveForward.State.blocking,
                            DefensiveForward.State.collecting, lambda:
                            evaluation.path.can_collect_ball_before_opponent(),
                            'Collecting')

        self.add_transition(
            DefensiveForward.State.collecting, DefensiveForward.State.blocking,
            lambda: not evaluation.path.can_collect_ball_before_opponent(),
            'Back to blocking')

        self.add_transition(
            DefensiveForward.State.collecting,
            behavior.Behavior.State.completed,
            lambda: self.collector is not None and \
                    self.collector.robot is not None and \
                    evaluation.ball.robot_has_ball(self.collector.robot),
            'Ball collected')

        # Create list of defenders and start the marking
    def on_enter_blocking(self):
        self.zone_def_pos, self.mark_bots[0], self.mark_bots[
            1] = evaluation.defensive_positioning.find_defense_positions()

        self.set_mark_targets()

        self.defenders[2] = skills.mark.Mark()
        self.add_subbehavior(self.defenders[2], self.names[2], required=True)
        self.defenders[2].mark_point = self.zone_def_pos

        # Reset time since call
        self.time_since_call = 0

    # Continue updating the mark positions
    def execute_blocking(self):
        self.time_since_call = (self.time_since_call + 1) % self.cache_amnt

        # You don't want to test if its 0 because it would execute instantly
        if (self.time_since_call == self.cache_amnt - 1):
            self.zone_def_pos, self.mark_bots[0], self.mark_bots[1] = \
                evaluation.defensive_positioning.find_defense_positions(
                    [self.defenders[0].robot, self.defenders[1].robot])

            for i in range(0, 2):
                if (self.defenders[i].mark_robot is not self.mark_bots[i]):
                    self.defenders[i].mark_robot = self.mark_bots[i]

            self.defenders[2].mark_point = self.zone_def_pos

    def on_exit_blocking(self):
        self.remove_all_subbehaviors()

    # Create collector method and then
    def on_enter_collecting(self):
        self.zone_def_pos, self.mark_bots[0], self.mark_bots[1] = \
            evaluation.defensive_positioning.find_defense_positions()

        # Take closest robot to collect
        # Leave other two to move into primary blocking positions
        self.collector = skills.capture.Capture()
        self.add_subbehavior(self.collector, 'collector', required=True)

        self.set_mark_targets()

    def on_exit_collecting(self):
        self.remove_all_subbehaviors()
        self.collector = None

    def set_mark_targets(self):
        for i in range(0, 2):
            self.defenders[i] = skills.mark.Mark()
            self.add_subbehavior(
                self.defenders[i],
                self.names[i],
                required=False,
                priority=10 - i)
            self.defenders[i].mark_robot = self.mark_bots[i]

    def we_have_ball(self):
        return any(
            evaluation.ball.robot_has_ball(r) for r in main.our_robots())
