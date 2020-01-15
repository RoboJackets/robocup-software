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

class BasicOffensiveScramble(standard_play.StandardPlay):

    _situationList = [
        situational_play_selection.SituationalPlaySelector.Situation.OFFENSIVE_SCRAMBLE
    ] # yapf: disable

    class State(enum.Enum):
        get_ball = 1, 'Start the play'

    def __init__(self):
        super().__init__(continuous = False)

        for s in BasicOffensiveScramble.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            BasicOffensiveScramble.State.get_ball, lambda: True, 'Immidiatley')

        self.add_transition(BasicOffensiveScramble.State.get_ball,
            behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('Capture ball').state == behavior.
            Behavior.State.completed or self.subbehavior_with_name('Capture ball').state == behavior.
            Behavior.State.failed, 'Captured')

    @classmethod
    def score(cls):
        return 10

    def get_dropback_point(self):
        ball_pos = main.ball().pos
        dropback_point = robocup.Point(ball_pos.x, ball_pos.y - constants.Field.Length/6)
        return dropback_point

    def get_across_point(self):
        ball_pos = main.ball().pos
        sign = 1
        if (ball_pos.x >= 0):
            sign = -1
        across_point = robocup.Point(ball_pos.x + (sign * constants.Field.Width/5), ball_pos.y)
        return across_point

    def on_enter_get_ball(self):
        self.remove_all_subbehaviors()
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

    def on_exit_get_ball(self):
        self.remove_all_subbehaviors()

