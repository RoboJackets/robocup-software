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
import evaluation.field
import evaluation.passing
import evaluation.shooting
import functools


#2 offenders are related to do a certain event base off of great code
class AdvanceZoneMidfielder(composite_behavior.CompositeBehavior):
    # Weights for the general field positioning
    FIELD_POS_WEIGHTS = (0.01, 3, 0.02)
    # Weights for finding best pass
    PASSING_WEIGHTS = (2, 2, 15, 1)
    # Initial arguements for the nelder mead optimization in passing positioning
    NELDER_MEAD_ARGS = (robocup.Point(0.5, 2), \
                        robocup.Point(0.01, 0.01), 1, 2, \
                        0.75, 0.5, 50, 1, 0.1)

    class State(enum.Enum):
        ## getting ready to recieve a pass from another robot
        passSet = 1
        ## getting ready to recieve a pass from the dribbling robot backwards for tactical advatage
        passBack = 2

    def __init__(self):
        super().__init__(continuous=True)

        #Sends one robot to best reciever point, and other to the best receiving point
        # of the first midfielder
        self.add_state(AdvanceZoneMidfielder.State.passSet,
                       behavior.Behavior.State.running)
        #To be imblemented later when wanting to play a more conservative 
        self.add_state(AdvanceZoneMidfielder.State.passBack,
                       behavior.Behavior.State.running)
        # old constants in Simple zone midfieler
        self.y_hold_percent = .8
        self.x_positions = [-constants.Field.Width / 3, \
                            constants.Field.Width / 3]
        self.x_positions_alternate = [-constants.Field.Width/2, \
                                      constants.Field.Width/2]
        self.names = ['left_mid', 'right_mid']
        # Which robot would be best to move to first move point
        self.optimal = 0

        self.moves = [None, None]

        for s in AdvanceZoneMidfielder.State:
            self.add_state(s, behavior.Behavior.State.running)

        # more conservative play that should be implemented later
        self.add_transition(
            AdvanceZoneMidfielder.State.passSet,
            AdvanceZoneMidfielder.State.passBack,
            lambda: False,  #not self.should_get_open(),
            "When dribbler is in danger and not about to shoot")

        #intially should be passSet method
        self.add_transition(behavior.Behavior.State.start,
                            AdvanceZoneMidfielder.State.passSet, lambda: True,
                            "Immediately")

    def execute_passBack(self):
        target_y = self.y_hold_percent * main.ball().pos.y

        for i in range(2):
            if (self.moves[i] is None):
                self.moves[i] = skills.move.Move(
                    robocup.Point(self.x_positions[i], target_y))
                self.add_subbehavior(
                    self.moves[i], self.names[i], required=False, priority=1)
            else:
                self.moves[i].pos = robocup.Point(self.x_positions[i],
                                                  target_y)

    def execute_passSet(self):
        #gets the best position to travel to for ball reception
        best_Pos_From_Ball, value = evaluation.passing_positioning.eval_best_receive_point(
            main.ball().pos,
            main.our_robots(), AdvanceZoneMidfielder.FIELD_POS_WEIGHTS,
            AdvanceZoneMidfielder.NELDER_MEAD_ARGS,
            AdvanceZoneMidfielder.PASSING_WEIGHTS)

        points = [robocup.Point(0, 0), robocup.Point(0, 0)]
        self.optimal = 0
        # sets either the left or the right side of the ball to the best ball position
        # and then notes which one is the other
        # left side of the ball
        if (best_Pos_From_Ball.x < 0):
            points[0] = best_Pos_From_Ball
            self.optimal = 0
            other = 1
        #right side of the ball
        else:
            points[1] = best_Pos_From_Ball
            self.optimal = 1
            other = 0

        # sets the second point
        points[
            other], value2 = evaluation.passing_positioning.eval_best_receive_point(
                points[self.optimal],
                main.our_robots(), AdvanceZoneMidfielder.FIELD_POS_WEIGHTS,
                AdvanceZoneMidfielder.NELDER_MEAD_ARGS,
                AdvanceZoneMidfielder.PASSING_WEIGHTS)

        # moves the robots
        for i in range(2):
            if (self.moves[i] is None):
                self.moves[i] = skills.move.Move(points[i])
                self.add_subbehavior(
                    self.moves[i], self.names[i], required=False, priority=1)
            else:
                self.moves[i].pos = points[i]

    def on_exit_passSet(self):
        self.remove_all_subbehaviors()

    def on_exit_passBack(self):
        self.remove_all_subbehaviors()

#Decides whether the dribbler is in danger and the corse of action of midfielder
#probobaly will revise this to be based off of score differential 

    def should_get_open(self):
        if (evaluation.field.field_pos_coeff_at_pos(main.ball().pos) < .85):
            return True
        else:
            return False
