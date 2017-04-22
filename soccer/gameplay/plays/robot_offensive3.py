import play
import behavior
import skills
import tactics
import main
import robocup
import evaluation
import constants


class RobotOffensive3(play.Play):

    def __init__(self):
        super().__init__(continuous=False)  # FIXME: continuous?


        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")
        self.add_transition(behavior.Behavior.State.running, behavior.Behavior.State.completed, lambda: self.subbehavior_with_name('striker').state==behavior.Behavior.State.completed,"kick complete")

    
    def on_enter_running(self):
        goalie=tactics.positions.goalie.Goalie()
        goalie.shell_id = main.root_play().goalie_id
        self.add_subbehavior(goalie,'goalie',required=True)

        striker=skills.pivot_kick.PivotKick()
        striker.target = constants.Field.TheirGoalSegment
        striker.aim_params['error_threshold'] = 0.15
        striker.aim_params['max_steady_ang_vel'] = 7
        striker.aim_params['min_steady_duration'] = 0.1
        striker.aim_params['desperate_timeout'] = 2.5
        self.add_subbehavior(striker,'striker',required=True)
        #TODO: add a third robot as support that is actually helpful


    def on_exit_running(self):
        self.remove_subbehavior('goalie')
        self.remove_subbehavior('striker')


    @classmethod
    def score(cls):
        return 10 if main.game_state().is_playing() else float("inf")

    @classmethod
    def handles_goalie(cls):
        return True
