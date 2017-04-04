import main
import robocup
import behavior
import constants
import enum
import composite_behavior
import skills.mark

# Alternates between marking and collecting the ball
class SubmissiveDefensiveForward(composite_behavior.CompositeBehavior):
    class State(enum.Enum):
        # Block shots/passes
        blocking = 1
        # Collect the ball when it is lost
        collecting = 2

    def __init__(self):
        super().__init__(continuous=True)

        self.mark = None
        self.capture = None
        self._mark_robot = None
        self._mark_pos = None
        self.robot = None
        self.dodge_dist = .2

        for s in SubmissiveDefensiveForward.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            SubmissiveDefensiveForward.State.blocking,
                            lambda: True,
                            'immediately')

        self.add_transition(SubmissiveDefensiveForward.State.blocking,
                            SubmissiveDefensiveForward.State.collecting,
                            lambda: self.within_range(),
                            'Collect Ball')

        self.add_transition(SubmissiveDefensiveForward.State.collecting,
                            SubmissiveDefensiveForward.State.blocking,
                            lambda: not self.within_range(),
                            'Block')

    # Wether we can collect the ball before the opponent
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

        # While there is a robot in the way
        blocking_robot = self.find_intersecting_robot(line, blocking_robots)
        while (blocking_robot is not None) and (iterations < 10):
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

    # Create mark
    def on_enter_blocking(self):
        self.mark = skills.mark.Mark()
        self.add_subbehavior(self.mark, 'mark', required=True)
        self.mark.mark_robot = self._mark_robot
        self.mark.mark_pos = self._mark_pos
        self.robot = self.mark.robot

    # Update mark based on parent
    def execute_blocking(self):
        self.mark.mark_robot = self._mark_robot
        self.mark.mark_pos = self._mark_pos

    # Clear everything
    def on_exit_blocking(self):
        self.remove_all_subbehaviors()

    # Start collection routine
    def on_enter_collecting(self):
        self.capture = skills.capture.Capture()
        self.robot = self.capture.robot
        self.add_subbehavior(self.capture, 'capture', required=True)

    def on_exit_collecting(self):
        self.remove_all_subbehaviors()


    # Robot to mark
    @property
    def mark_robot(self):
        return self._mark_robot

    @mark_robot.setter
    def mark_robot(self, value):
        self._mark_robot = value

    # Position to mark
    @property
    def mark_pos(self):
        return self._mark_pos

    @mark_pos.setter
    def mark_pos(self, value):
        self._mark_pos = value