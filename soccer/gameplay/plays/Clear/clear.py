import main
import robocup
import behavior
import constants
import enum

import standard_play
import evaluation
import situational_play_selection
import tactics.coordinated_pass
import skills.move
import skills.capture
import random

#Based on clear sitation
#    Have ball in defense
#   Want to Kick ball up field and get other robots in position
#


##
# A basic clearing play written in responce to the fact that we didn't have a clearing play
#
# It needs to be renamed as "clear" is the name of the situation, and each play that fufills it
# needs to have a unique name beyond that
#
class Clear(standard_play.StandardPlay):
    class State(enum.Enum):
        get_ball = 1, 'Get the ball and movee other robots up'
        clear_ball = 2, 'Kick the ball upfield'

    _situationList = [
        situational_play_selection.SituationalPlaySelector.Situation.CLEAR,
    ] # yapf: disable

    def __init__(self):
        super().__init__(continuous=False)

        for s in Clear.State:
            self.add_state(s, behavior.Behavior.State.running)

        #Points to move up offense bots to
        self.offense_point_1 = robocup.Point(0.40 * constants.Field.Width,
                                             0.95 * constants.Field.Length)

        self.offense_point_2 = robocup.Point(-0.40 * constants.Field.Width,
                                             0.95 * constants.Field.Length)

        #Points to move midfield bots to
        self.midfield_point_1 = robocup.Point(0.20 * constants.Field.Width,
                                              0.70 * constants.Field.Length)

        self.midfield_point_2 = robocup.Point(-0.40 * constants.Field.Width,
                                              0.60 * constants.Field.Length)

        self.points = [
            self.offense_point_1, self.offense_point_2, self.midfield_point_1,
            self.midfield_point_2
        ]

        self.offense_points = [self.offense_point_1, self.offense_point_2]

        self.midfield_points = [self.midfield_point_1, self.midfield_point_2]

        self.add_transition(behavior.Behavior.State.start,
                            Clear.State.get_ball, lambda: True, 'Immidiatley')

        self.add_transition(
            Clear.State.get_ball,
            Clear.State.clear_ball, lambda: self.subbehavior_with_name(
                'Capture ball').is_done_running(), 'After ball is captured')

    def on_enter_get_ball(self):
        self.remove_all_subbehaviors()
        #Capture the ball and move robot up
        self.add_subbehavior(
            skills.capture.Capture(), 'Capture ball', required=True)

        count = 0
        for i in self.offense_points:
            count += 1
            self.add_subbehavior(
                skills.move.Move(i),
                'move to point ' + str(count),
                required=False)

    def on_enter_clear_ball(self):
        #Chip ball to either offense points, while still moving robots up
        self.remove_all_subbehaviors()

        num = random.randint(0, 1)
        self.add_subbehavior(
            tactics.coordinated_pass.CoordinatedPass(
                self.points[num], receiver_required=False, use_chipper=True),
            'clear',
            required=True)

        self.add_subbehavior(
            skills.move.Move(self.offense_points[1 - num]),
            'keep moving',
            required=False)
