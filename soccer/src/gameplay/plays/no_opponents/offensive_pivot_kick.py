import play
import behavior
import robocup
import skills.line_kick
import main
import constants
import enum
import role_assignment


class OffensivePivotKick(play.Play):
    def __init__(self):
        super().__init__(continuous=False)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.has_subbehavior_with_name('kicker') and self.subbehavior_with_name('kicker').is_done_running(),
            "kicker finished")

    def on_enter_running(self):
        kicker = skills.pivot_kick.PivotKick()
        kicker.target = constants.Field.TheirGoalSegment

        kicker.aim_params = {'error_threshold': .01,
                             'desperate_timeout': 10,
                             'max_steady_ang_vel': 4}
        self.add_subbehavior(kicker, 'kicker', required=True, priority=100)

    def on_exit_running(self):
        self.remove_subbehavior('kicker')

    @classmethod
    def score(cls):
        gs = main.game_state()
        #Currently has lower priority than basic_122. Maybe add a check to see if we have all our robots?
        return 15 if gs.is_playing() else float("inf")

    @classmethod
    def handles_goalie(self):
        return False
