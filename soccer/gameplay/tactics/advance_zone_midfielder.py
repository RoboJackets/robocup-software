import main
import robocup
import behavior
import constants
import enum
import math

import composite_behavior
import skills.move
import evaluation.ball
import evaluation.passing_positioning
import evaluation.passing
import evaluation.shooting
import functools
import plays.offense.adaptive_formation


# 2 midfielder rely on the future location of their teammate to pass quickly
class AdvanceZoneMidfielder(composite_behavior.CompositeBehavior):
    # Weights for the general field positioning
    FIELD_POS_WEIGHTS = (0.01, 3, 0.02)  
    # Weights for finding best pass
    PASSING_WEIGHTS = (2, 2, 15, 10)  
    # Initial arguements for the nelder mead optimization in passing positioning
    NELDER_MEAD_ARGS = (robocup.Point(0.75, 1), robocup.Point(0.01, 0.01), 1,
                        1.1, 0.5, 0.9, 100, 1, 0.1)

    class State(enum.Enum):
        # getting ready to recieve a pass from another robot
        pass_set = 1
        # sitting still when we are kicking a ball to goal
        hold = 2

    def __init__(self):
        super().__init__(continuous=True)

        # Sends one robot to best reciever point, and other to the best receiving point
        # of the first midfielder
        self.add_state(AdvanceZoneMidfielder.State.pass_set,
                       behavior.Behavior.State.running)

        self.add_state(AdvanceZoneMidfielder.State.hold,
                       behavior.Behavior.State.running)

        self.moves = [None, None]

        # an abitraty small value to account for small error
        self.epsilon = 10**-6

        # a generic distance that's required to get out of the way if the ideal pos robot has a shot
        self.escape_distance = constants.Robot.Radius * 6

        if main.ball().valid:
            self.passing_point = main.ball().pos
        else:
            self.passing_point = robocup.Point(0, 0)

        self.kick = False

        self.priorities = [1, 2]

        self.names = ['best', 'alternative']
        # intially should be pass_set method and adds conditions to switch to hold on kick
        self.add_transition(behavior.Behavior.State.start,
                            AdvanceZoneMidfielder.State.pass_set, lambda: True,
                            "Immediately")
        self.add_transition(
            AdvanceZoneMidfielder.State.pass_set,
            AdvanceZoneMidfielder.State.hold, lambda: self.kick,
            "When a kick play begins")
        self.add_transition(
            AdvanceZoneMidfielder.State.hold,
            AdvanceZoneMidfielder.State.pass_set, lambda: not self.kick,
            "When not in a kick play")

    def execute_pass_set(self):
        # gets the best position to travel to for ball reception
        best_point = self.passing_point

        # sets the second point
        alt_point, value2 = evaluation.passing_positioning.eval_best_receive_point(
                self.passing_point,
                main.our_robots(), AdvanceZoneMidfielder.FIELD_POS_WEIGHTS,
                AdvanceZoneMidfielder.NELDER_MEAD_ARGS,
                AdvanceZoneMidfielder.PASSING_WEIGHTS)

        # check for futile position i.e the alternate position is in the way of a shot from best position
        if self.in_shot_triangle(best_point, alt_point):
            alt_point = self.remove_obstruction(best_point, alt_point)

        points = [best_point, alt_point]
        # moves the robots and assigns information
        for i in range(len(points)):
            if (self.moves[i] is None):
                self.moves[i] = skills.move.Move(points[i])
                self.add_subbehavior(
                    self.moves[i],
                    self.names[i],
                    required=False,
                    priority=self.priorities[i])
            else:
                self.moves[i].pos = points[i]
    #defaults to a defensive position on kicks
    def execute_hold(self):
        # old methodology (simple zone midfielder) for simple zone midfielder (check comments on that method)
        y_temp_hold = 0.8 * self.passing_point.y

        x_temp_hold = constants.Field.Width / 3

        self.hold_point = [
            robocup.Point(x_temp_hold, y_temp_hold),
            robocup.Point(-x_temp_hold, y_temp_hold)
        ]

        for i in range(len(self.hold_point)):
            if (self.moves[i] is None):
                self.moves[i] = skills.move.Move(self.hold_point[i])
                self.add_subbehavior(
                    self.moves[i], self.names[i], required=False, priority=1)
            else:
                self.moves[i].pos = self.hold_point[i]

    def on_exit_pass_set(self):
        self.remove_all_subbehaviors()

    def on_exit_hold(self):
        self.remove_all_subbehaviors()

    # this method determines if the 2nd midfielders kicking point is actually in the way of the potential shot
    def in_shot_triangle(self, best_point, alt_point):
        # get the two points of the enemies goal
        goalSegment = constants.Field.TheirGoalSegment

        right_post = goalSegment.get_pt(0)
        left_post = goalSegment.get_pt(1)
        # draw the line from the ideal passing position to the goal corners
        main.system_state().draw_line(
            robocup.Line(right_post, best_point), (255, 0, 255), "Shot Range")
        main.system_state().draw_line(
            robocup.Line(left_post, best_point), (255, 0, 255), "Shot Range")
        # angle between the line from ideal pass point and the goal corner and between the line ideal pass point and other goal corner
        shot_angle = (right_post - best_point).angle_between((left_post - best_point))
        # angle between the line from ideal pass point and goal corner 1 and between the line from ideal pass point and 2nd best pass point
        left_post_alt_pos_angle = (best_point - alt_point).angle_between(best_point - right_post)
        # angle between the line from ideal pass point and goal corner 2 and between the line from ideal pass point and 2nd best pass point
        right_post_alt_pos_angle = (best_point - alt_point).angle_between(best_point - left_post)

        # decides which side is closer to point by the smaller theta
        if left_post_alt_pos_angle < right_post_alt_pos_angle:
            self.closest_post = left_post
        else:
            self.closest_post = right_post
        # if interior angles near sum to 0 then robot is inside the the zone. 
        return abs(shot_angle - left_post_alt_pos_angle - right_post_alt_pos_angle) < self.epsilon

    # finds closest point to leave zone and goes four robot radius outside the zone
    def remove_obstruction(self, best_point, alt_point):
        # use line projection (look up vector projection online for more information)
        post_to_best_vector = best_point - self.closest_post
        best_to_alt_vector = best_point - alt_point
        new_vector_length = best_to_alt_vector.dot(post_to_best_vector) / post_to_best_vector.dot(post_to_best_vector)
        proj_post_to_best_vector = post_to_best_vector * new_vector_length
        # find the vector out
        escape_vector = best_to_alt_vector - proj_post_to_best_vector
        # change the point
        escape_point = alt_point + (escape_vector / escape_vector.mag()
                                  ) * self.escape_distance
        return escape_point

    # gets passing point from adaptive formation
    @property
    def passing_point(self):
        return self._passing_point

    @passing_point.setter
    def passing_point(self, value):
        self._passing_point = value

    # gets when we are kicking from adaptive formation
    @property
    def kick(self):
        return self._kick

    @kick.setter
    def kick(self, value):
        self._kick = value

