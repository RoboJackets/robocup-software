import robocup
import play
import behavior
import constants
import main
import skills.move
import skills.capture
import enum
import evaluation
import tactics.coordinated_pass

class TwoSideAttack(play.Play):
    # Try to pass to the better target
    # Soccer/gameplay/evaluation/shot.py
    # Tell where passing from and where to pass to
    # Estimate of which shot is better

    class State(enum.Enum):
        setup = 1
        passing = 2
        kicking = 3;

    def __init__(self):
        super().__init__(continuous=False)

        # Setup
            # Move A and move B, capture in setup
        # Passing
            # Pick best target, add coordinated pass subbehavior
        # Kicking
            # Pivot kick (by default attacks enemy goal)

        self.add_state(TwoSideAttack.State.setup,
            behavior.Behavior.State.running)
        self.add_state(TwoSideAttack.State.passing,
            behavior.Behavior.State.running)
        self.add_state(TwoSideAttack.State.kicking,
            behavior.Behavior.State.running)

        # Add transitions
        self.add_transition(behavior.Behavior.State.start,
            TwoSideAttack.State.setup,
            lambda: True,
            'immediately')
        self.add_transition(TwoSideAttack.State.setup,
            TwoSideAttack.State.passing,
            lambda: self.all_subbehaviors_completed(),
            'all subbehaviors completed')
        self.add_transition(TwoSideAttack.State.passing,
            TwoSideAttack.State.kicking,
            lambda: self.all_subbehaviors_completed(),
            'all subbehaviors completed')

        self.robot_points = [
            robocup.Point(-constants.Field.Width / 4.0, 3 * constants.Field.Length / 4.0),
            robocup.Point(constants.Field.Width / 4.0, 3 * constants.Field.Length / 4.0)
        ]

    @classmethod
    def score(cls):
        if main.game_state().is_playing():
            if main.ball().valid:
                if main.ball().pos.y < constants.Field.Length / 2:
                    return 5
                return 15
            return float("inf")
        return float("inf")


    def all_subbehaviors_completed(self):
        return all([bhvr.is_done_running() for bhvr in self.all_subbehaviors()])


    def on_enter_setup(self):
        # Add subbehaviors based on information
        capture = skills.capture.Capture()
        self.add_subbehavior(skills.move.Move(self.robot_points[0]), 'moveA', required=True)
        self.add_subbehavior(skills.move.Move(self.robot_points[1]), 'moveB', required=True)
        self.add_subbehavior(capture, 'capture', required=True)


    def on_exit_setup(self):
        to_exclude_0 = self.subbehavior_with_name('moveA')
        to_exclude_1 = self.subbehavior_with_name('moveB')
        capture = self.subbehavior_with_name('capture')
        self.to_exclude = [to_exclude_0.robot, to_exclude_1.robot, capture.robot]
        self.remove_all_subbehaviors()


    def on_enter_passing(self):
        # Do shot evaluation here
        win_eval = robocup.WindowEvaluator(main.system_state())
        for r in self.to_exclude:
            win_eval.add_excluded_robot(r)
        _, best = win_eval.eval_pt_to_opp_goal(self.robot_points[0])
        # TODO to_exclude is a hack
        rob_0_chance = best.shot_success * evaluation.passing.eval_pass(self.robot_points[0], self.to_exclude[2].pos)

        _, best = win_eval.eval_pt_to_opp_goal(self.robot_points[1])
        rob_1_chance = best.shot_success * evaluation.passing.eval_pass(self.robot_points[1], self.to_exclude[2].pos)

        if rob_0_chance > rob_1_chance:
            robot_pos = 0
        else:
            robot_pos = 1

        self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.robot_points[robot_pos]), 'pass')


    def on_exit_passing(self):
        self.remove_all_subbehaviors()


    def on_enter_kicking(self):
        kick = skills.pivot_kick.PivotKick()
        kick.target = constants.Field.TheirGoalSegment
        kick.aim_params['desperate_timeout'] = 3
        self.add_subbehavior(kick, 'kick', required=False)


    def on_exit_kicking(self):
        self.remove_all_subbehaviors()
