import main
import robocup
import behavior
import constants
import enum
import math

import composite_behavior
import evaluation.defensive_positioning
import tactics.submissive_defensive_forward
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

        self.zone_def_pos = robocup.Point(0,0)
        self.mark_bots = [None, None]

        self.block_dist = 0.01
        self.block_angle_coeff = 0.5
        self.names = ['mark_main', 'mark_sub', 'mark_zone']
        # Distance offset for path estimation
        self.dodge_dist = .2

        for s in DefensiveForward.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            DefensiveForward.State.blocking,
                            lambda: True,
                            'immediately')

        self.add_transition(DefensiveForward.State.blocking,
                            DefensiveForward.State.collecting,
                            lambda: self.within_range(),
                            'Collecting')

        self.add_transition(DefensiveForward.State.collecting,
                            DefensiveForward.State.blocking,
                            lambda: not self.within_range() or self.subbehavior_with_name('collector').is_done_running(),
                            'Back to blocking')

        # TODO: Finish play when ball is collected

    # Create list of defenders and start the marking
    def on_enter_blocking(self):
        self.zone_def_pos, self.mark_bots[0], self.mark_bots[1] = evaluation.defensive_positioning.find_defense_positions()

        for i in range(0, 2):
            self.defenders[i] = skills.mark.Mark()
            self.add_subbehavior(self.defenders[i], self.names[i], required=True)
            self.defenders[i].mark_robot = self.mark_bots[i]
            # TODO: Shift to mark point when "get_block_pos" is fixed

        self.defenders[2] = skills.mark.Mark()
        self.add_subbehavior(self.defenders[2], self.names[2], required=True)
        self.defenders[2].mark_point = self.zone_def_pos

    # Continue updating the mark positions
    def execute_defending(self):
        self.zone_def_pos, self.mark_bots[0], self.mark_bots[1] = evaluation.defensive_positioning.find_defense_positions()

        for i in range(0, 2):
            self.defenders[i].mark_robot = self.mark_bots[i]
            # TODO: Shift to mark point when "get_block_pos" is fixed

        self.defenders[2].mark_point = self.zone_def_pos

    def on_exit_defending(self):
        self.remove_all_subbehaviors()

        # Reset "queue"
        for i in range(0, 3):
            self.defenders[i] = None

    # Create collector method and then
    def on_enter_collecting(self):
        self.zone_def_pos, self.mark_bots[0], self.mark_bots[1] = evaluation.defensive_positioning.find_defense_positions()

        # Take closest robot to collect
        # Leave other two to move into primary blocking positions
        self.collector = skills.caputre.Capture()
        self.add_subbehavior(self.collector, 'collector', required=True)

        for i in range(0, 2):
            self.defenders[i] = skills.mark.Mark()
            self.add_subbehavior(self.defenders[i], self.names[i], required=True)
            self.defenders[i].mark_robot = self.mark_bots[i]

    def on_exit_collecting(self):
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
        # TODO: Do some sort of prediction as the ball moves
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