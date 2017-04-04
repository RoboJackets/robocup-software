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
        # Everyone is defending
        blocking = 1
        # Closest defender is collecting, all others are blocking
        collecting = 2

    def __init__(self):
        super().__init__(continuous=True)

        # Robot queue
        self.defenders = []
        # Single robot collecting
        self.collector = None

        self.zone_def_pos = robocup.Point(0,0)
        self.mark_bots = [None, None]

        self.block_dist = 0.01
        self.block_angle_coeff = 0.5

        for s in DefensiveForward.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            DefensiveForward.State.blocking,
                            lambda: True,
                            'immediately')

        self.add_transition(DefensiveForward.State.blocking,
                            DefensiveForward.State.collecting,
                            lambda: self.bot_within_range(),
                            'Collecting')

        self.add_transition(DefensiveForward.State.collecting,
                            DefensiveForward.State.blocking,
                            lambda: not self.bot_within_range(),
                            'Back to blocking')

    def on_enter_blocking(self):
        self.zone_def_pos, self.mark_bots[0], self.mark_bots[1] = evaluation.defensive_positioning.find_defense_positions()

        names = ['mark_main', 'mark_sub', 'mark_zone']

        for i in range(0, 2):
            self.defenders.extend([skills.mark.Mark()])
            self.add_subbehavior(self.defenders[i], names[i], required=True)
            self.defenders[i].mark_robot = self.mark_robot[i]
            # Shift to mark point when "get_block_pos" is fixed

        self.defenders.extend([skills.mark.Mark()])
        self.add_subbehavior(self.defenders[2], names[2], required=True)
        self.defenders[2].marf_point = self.zone_def_pos

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

    # Wether any robot can collect the ball before the opponent
    def within_range(self):
        shortest_opp_dist = 10
        shortest_our_dist = 10
        target_pos = main.ball().pos

        # Find closest opponent robot
        for bot in main.their_robots():
            dist = self.estimate_path_length(bot.pos, target_pos, main.our_robots())
            if (dist < shortest_opp_dist):
                shortest_opp_dist = dist

        # Find closest robot on our team
        for bot in main.our_robots():
            dist = self.estimate_path_length(bot.pos, target_pos, main.their_robots())
            if (dist < shortest_our_dist):
                shortest_our_dist = dist

        # Greater than 1 when we are further away
        return shortest_our_dist / shortest_opp_dist < 1.05

    # Estimates the length of a path given a robot
    def estimate_path_length(self, start, end, blocking_robots):
        total = 0
        next_pt = start
        line = robocup.Segment(start, end)
        iterations = 0
        max_iterations = 10

        # While there is a robot in the way
        blocking_robot = self.find_intersecting_robot(line, blocking_robots)
        while (blocking_robot is not None) and (iterations < max_iterations):
            # Find next point
            # Next point is +-dodge_dist * perp_vect
            robot_vec = (blocking_robot.pos - next_pt)
            perp_vec = robot_vec.perp_cw().normalized()

            pt1 = perp_vec * self.dodge_dist + blocking_robot.pos- next_pt
            pt2 = perp_vec * -self.dodge_dist + blocking_robot.pos - next_pt

            # Find shortest path            
            if (pt1.mag() < pt2.mag()):
                next_pt = pt1
            else:
                next_pt = pt2

            # Add dist to total
            total += (next_pt - start).mag()

            line = robocup.Segment(next_pt, end)
            blocking_robot = self.find_intersecting_robot(line, blocking_robots)
            iterations += 1

        total += (end - next_pt).mag()

        return total

    def find_intersecting_robot(self, line, blocking_robots):
        for bot in blocking_robots:
            if (line.dist_to(bot.pos) < self.dodge_dist):
                return bot

        return None