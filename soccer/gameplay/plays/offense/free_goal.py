import play
import behavior
import robocup
import skills.line_kick
import tactics.defense
import main
import constants
import enum
import role_assignment


class FreeGoal(play.Play):

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
        kicker=skills.pivot_kick.PivotKick()
        kicker.target=constants.Field.TheirGoalSegment
        #main.ball().pos - constants.Field.TheirGoalSegment.center().y
        #if (abs(main.ball().pos.y - constants.Field.TheirGoalSegment.center().y) < 1):
        #    kicker.target = constants.Field.TheirGoalSegment.center() - robocup.Point(0,1.5)

        kicker.aim_params={'error_threshold':.005,'desperate_timeout': 20,'max_steady_ang_vel':4}
        self.add_subbehavior(kicker,'kicker',required=True,priority=100)

    def on_exit_running(self):
        self.remove_subbehavior('kicker')



    @classmethod
    def score(cls):
        gs = main.game_state()
        # readd and len(main.system_state().their_robots)==0
        return 2 if gs.is_playing() else float("inf")

    @classmethod
    def handles_goalie(self):
        return False
