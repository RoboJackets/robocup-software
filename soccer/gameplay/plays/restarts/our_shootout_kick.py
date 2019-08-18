import play
import behavior
import robocup
import main
import tactics.penalty
import planning_priority
from enum import Enum
import skills.move
import skills.line_kick
import constants


# one robot kicks the ball, the others just line up and wait
class OurShootoutKick(play.Play):
    class State(Enum):
        #setup
        starting = 0
        #kick
        shooting = 1

    def __init__(self):
        super().__init__(continuous=True)

        for s in OurShootoutKick.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            OurShootoutKick.State.starting, lambda: True,
                            'immediately')

        # begin to kick as soon as given the ready signal
        self.add_transition(
            OurShootoutKick.State.starting,
            OurShootoutKick.State.shooting, lambda: main.game_state(
            ).is_ready_state(), 'shoot when given the ready signal')

    def on_enter_starting(self):
        self.add_subbehavior(
            skills.move.Move(self.calculate_starting_pos()),
            'starting',
            required=False,
            priority=5)

    def execute_starting(self):
        self.subbehavior_with_name(
            "starting").pos = self.calculate_starting_pos()

    def on_exit_starting(self):
        self.remove_all_subbehaviors()

    def execute_shooting(self):
        kicker = skills.line_kick.LineKick()  #skills.pivot_kick.PivotKick()
        #aim for goal segmant
        win_eval = robocup.WindowEvaluator(main.context())

        kicker.target = constants.Field.TheirGoalSegment  # TODO Use win eval
        kicker.aim_params['desperate_timeout'] = 8
        #kick
        if (not self.has_subbehavior_with_name('kicker') or
                self.subbehavior_with_name('kicker').is_done_running()):
            self.remove_all_subbehaviors()
            self.add_subbehavior(kicker, 'kicker', required=False, priority=5)

    def on_exit_shooting(self):
        self.remove_all_subbehaviors()

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0.5 if gs.is_penalty_shootout() and gs.is_our_penalty(
        ) else float("inf")

    @classmethod
    def is_restart(cls):
        return False

    @classmethod
    def handles_goalie(cls):
        return True

    def calculate_starting_pos(self):
        # find the direction from the enemy goal to the ball
        behind_ball = (
            main.ball().pos - constants.Field.TheirGoalSegment.center())
        # normalize the vector
        behind_ball = behind_ball.normalized()
        behind_ball = behind_ball * 0.5 + main.ball().pos
        return behind_ball
