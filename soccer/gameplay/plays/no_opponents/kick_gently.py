import play
import behavior
import skills.move
import skills.pivot_kick
import constants
import robocup
import main
import enum
import timeout_behavior
import tactics.coordinated_pass
import evaluation.touchpass_positioning


class KickGently(play.Play):

    Threshold = 1
    Kick_Power = .1

    class State(enum.Enum):
        passing = 1
        dumb = 2
        kicking = 3
    #Used for most restarts with no enemy team, passes the ball to the center then scores
    def __init__(self, indirect=None):

        #center of their half
        self.target = constants.Field.CenterPoint
        self.target.y *= 1.5

        super().__init__(continuous=True)

        for state in KickGently.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            KickGently.State.passing, lambda: True,
                            'immediately')

        self.add_transition(
            KickGently.State.passing, KickGently.State.dumb,
            lambda: abs(main.ball().pos.x) > KickGently.Threshold and self.subbehavior_with_name('passer').state == behavior.Behavior.State.completed,
            'not centered')

        self.add_transition(KickGently.State.dumb, KickGently.State.passing,
                            lambda: True, 'immediately')

        self.add_transition(
            KickGently.State.passing, KickGently.State.kicking,
            lambda: abs(main.ball().pos.x) <= KickGently.Threshold,
            'ball is centered')

        self.add_transition(
            KickGently.State.kicking, KickGently.State.kicking,
            lambda: abs(main.ball().pos.x) <= KickGently.Threshold and self.subbehavior_with_name('kicker').state == behavior.Behavior.State.completed,
            'ball is centered')

        self.add_transition(
            KickGently.State.kicking, KickGently.State.passing,
            lambda: abs(main.ball().pos.x) > KickGently.Threshold,
            'ball is not centered')

    def on_enter_passing(self):
        passer = skills.pivot_kick.PivotKick()
        passer.target = self.target
        passer.kick_power = KickGently.Kick_Power
        self.add_subbehavior(passer, 'passer', required=True, priority=100)

    def on_exit_passing(self):
        self.remove_all_subbehaviors()

    def on_enter_kicking(self):
        kicker = skills.pivot_kick.PivotKick()
        kicker.target = constants.Field.TheirGoalSegment
        #threshold needs some tuning
        kicker.aim_params = {'error_threshold': .05,
                             'desperate_timeout': float("inf"),
                             'max_steady_ang_vel': 4}
        self.add_subbehavior(kicker, 'kicker', required=True, priority=100)

    def on_exit_kicking(self):
        self.remove_all_subbehaviors()

    def execute_running(self):
        main.debug_drawer().draw_circle(self.target, constants.Robot.Radius,
                                        constants.Colors.Green, "target")

    @classmethod
    def score(cls):
        gs = main.game_state()
        #This play does not work well against functional opponent robots, maybe add
        #"and len(main.system_state().their_robots)==0" or some kind of check for nonmoving opponents?
        return 1 if gs.is_playing() else float("inf")

    @classmethod
    def handles_goalie(cls):
        return True
