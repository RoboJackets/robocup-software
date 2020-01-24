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

# A basic play for the offensive scramble situation
# One robot will attempt to capture the ball
# Another robot will be sent behind the capturing one to a "dropback point"
# Another robot will be sent across the field from the capturing one
#


class BasicOffensiveScramble(standard_play.StandardPlay):

    _situationList = [
        situational_play_selection.SituationalPlaySelector.Situation.OFFENSIVE_SCRAMBLE
    ] # yapf: disable

    def __init__(self):
        super().__init__(continuous=False)

        self.add_transition(behavior.Behavior.State.start,
                            BasicOffensiveScramble.State.running, lambda: True,
                            'Immediately')

        self.add_transition(
            BasicOffensiveScramble.State.running,
            behavior.Behavior.State.completed, lambda: self.
            subbehavior_with_name('Capture ball').is_done_running(),
            'Captured')

    @classmethod
    def score(cls):
        return 10

    #Gets the point that the robot that will behind the capturing one will be
    def get_dropback_point(self):
        ball_pos = main.ball().pos
        dropback_point = robocup.Point(ball_pos.x,
                                       ball_pos.y - constants.Field.Length / 6)
        return dropback_point

    #Get point across from capturing robot another robot should be
    def get_across_point(self):
        ball_pos = main.ball().pos
        sign = 1
        mirror_x = 0 - main.ball().pos.x
        if (ball_pos.x <= constants.Field.Width / 4 and
                ball_pos.x >= -constants.Field.Width /
                4):  #Check if the ball is near the center line of the field
            if (ball_pos.x >= 0):
                sign = -1
            #If the ball is near the center then the across point should be a set distance across the ball position
            across_point = robocup.Point(
                ball_pos.x + (sign * constants.Field.Width / 2), ball_pos.y)
        else:
            #If the ball is not near the center, the across point is the mirror of the ball position across the y axis
            across_point = robocup.Point(mirror_x, ball_pos.y)
        return across_point

    def on_enter_running(self):
        self.add_subbehavior(
            skills.capture.Capture(), 'Capture ball', required=True)

        across_point = self.get_across_point()
        dropback_point = self.get_dropback_point()
        self.add_subbehavior(
            skills.move.Move(dropback_point),
            'move to dropback point',
            required=False)
        self.add_subbehavior(
            skills.move.Move(across_point),
            'move to across point',
            required=False)
