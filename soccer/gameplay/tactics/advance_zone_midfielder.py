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


#2 midfielder rely on the future location of their teammate to pass quickly
class AdvanceZoneMidfielder(composite_behavior.CompositeBehavior):
    # Weights for the general field positioning
    FIELD_POS_WEIGHTS = (0.01, 3, 0.02)#AdaptiveFormation.FIELD_POS_WEIGHTS
    # Weights for finding best pass
    PASSING_WEIGHTS = (2, 2, 15, 1)#AdaptiveFormation.PASSING_WEIGHTS
    # Initial arguements for the nelder mead optimization in passing positioning
    NELDER_MEAD_ARGS = (robocup.Point(0.5, 2), \
                        robocup.Point(0.01, 0.01), 1, 2, \
                        0.75, 0.5, 50, 1, 0.1)#AdaptiveFormation.NELDER_MEAD_ARGS

    class State(enum.Enum):
        ## getting ready to recieve a pass from another robot
        passSet = 1

        hold = 2
        

    def __init__(self):
        super().__init__(continuous=True)

        #Sends one robot to best reciever point, and other to the best receiving point
        # of the first midfielder
        self.add_state(AdvanceZoneMidfielder.State.passSet,
                       behavior.Behavior.State.running)

        self.add_state(AdvanceZoneMidfielder.State.hold, 
                        behavior.Behavior.State.running)

        self.moves = [None, None]


        self.priorities = [1, 2]

        self.passing_point = main.ball().pos

        y_temp_hold = 0.8 * self.passing_point.y

        x_temp_hold = constants.Field.Width / 3

        self.hold_point = [robocup.Point(x_temp_hold,y_temp_hold), robocup.Point(x_temp_hold,y_temp_hold)]

        self.dribble_state = True

        self.kick = False

        for s in AdvanceZoneMidfielder.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.names = ['left','right']
        #intially should be passSet method
        self.add_transition(behavior.Behavior.State.start,
                            AdvanceZoneMidfielder.State.passSet, lambda: True,
                            "Immediately")
        self.add_transition(AdvanceZoneMidfielder.State.passSet,
                            AdvanceZoneMidfielder.State.hold, lambda: False,
                            "When a kick play begins")
        self.add_transition(AdvanceZoneMidfielder.State.hold,
                            AdvanceZoneMidfielder.State.passSet, lambda: True,
                            "When not in a kick play")

    def on_enter_passSet(self):
        print("Entering PassSet");

    def execute_passSet(self):
        #gets the best position to travel to for ball reception
        best_Pos_From_Ball = self.passing_point
        """if (self.dribble_state):
            points = [robocup.Point(0, 0), robocup.Point(0, 0)]
            self.optimal = 1
            # sets either the left or the right side of the ball to the best ball position
            # and then notes which one is the other
            # left side of the ball
            points[self.optimal] = best_Pos_From_Ball
            self.other = 0
        else :
            points = [robocup.Point(0, 0), robocup.Point(0, 0)]
            self.optimal = 1
            # sets either the left or the right side of the ball to the best ball position
            # and then notes which one is the other
            # left side of the ball
            points[self.optimal] = best_Pos_From_Ball
            self.other = 0
        """
        points = [robocup.Point(0, 0), robocup.Point(0, 0)]
        points[0] = best_Pos_From_Ball;

        # sets the second point
        pointer, value2 = evaluation.passing_positioning.eval_best_receive_point(
                best_Pos_From_Ball,
                main.our_robots(), AdvanceZoneMidfielder.FIELD_POS_WEIGHTS,
                AdvanceZoneMidfielder.NELDER_MEAD_ARGS,
                AdvanceZoneMidfielder.PASSING_WEIGHTS)
        print("best point")
        print(points[0].x)
        print(points[0].y)
        print("second point")
        print(pointer)
        print(value2)
        print("diff")
        print(best_Pos_From_Ball - pointer)
        points[1] = pointer
        # moves the robots
        for i in range(2):
            if (self.moves[i] is None):
                self.moves[i] = skills.move.Move(points[i])
                self.add_subbehavior(
                    self.moves[i], self.names[i], required=False, priority= self.priorities[i])
            else:
                self.moves[i].pos = points[i]

    def on_enter_hold(self):
        print("kicking")

    def execute_hold(self):

        y_temp_hold = 0.8 * self.passing_point.y

        x_temp_hold = constants.Field.Width / 3

        self.hold_point = [robocup.Point(x_temp_hold,y_temp_hold), robocup.Point(x_temp_hold,y_temp_hold)]

        for i in range(2):
            if (self.moves[i] is None):
                self.moves[i] = skills.move.Move(self.hold_point[i])
                self.add_subbehavior(self.moves[i],
                                     self.names[i],
                                     required=False,
                                     priority=1)
            else:
                self.moves[i].pos = self.hold_point[i]



    @property
    def passing_point(self):
        return self._passing_point
    @passing_point.setter
    def passing_point(self, value):
        self._passing_point = value

    @property
    def dribble_state(self):
        return self._dribble_state

    @dribble_state.setter
    def dribble_state(self, value):
        self._dribble_state = value

    @property
    def kick(self):
        return self._kick

    @kick.setter
    def kick(self, value):
        self._kick = value
        print("Change Value to kick on")


    def on_exit_passSet(self):
        self.remove_all_subbehaviors()

    def on_exit_passBack(self):
        self.remove_all_subbehaviors()