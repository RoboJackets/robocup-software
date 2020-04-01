import robocup
import standard_play
import behavior
import constants
import main
import skills.move
import skills.capture
import enum
import evaluation
import tactics.coordinated_pass
import tactics.defense


class TwoSideAttack(standard_play.StandardPlay):
    # Try to pass to the better target
    # Soccer/gameplay/evaluation/shot.py
    # Tell where passing from and where to pass to
    # Estimate of which shot is better

    class State(enum.Enum):
        ## Move A and move B, capture in setup
        setup = 1
        ## Pick best target, add coordinated pass subbehavior
        passing = 2
        ## Pivot kick (by default attacks enemy goal)
        kicking = 3

    def __init__(self):
        super().__init__(continuous=False)

        self.add_state(TwoSideAttack.State.setup,
                       behavior.Behavior.State.running)
        self.add_state(TwoSideAttack.State.passing,
                       behavior.Behavior.State.running)
        self.add_state(TwoSideAttack.State.kicking,
                       behavior.Behavior.State.running)

        # Add transitions
        self.add_transition(behavior.Behavior.State.start,
                            TwoSideAttack.State.setup, lambda: True,
                            'immediately')
        self.add_transition(
            TwoSideAttack.State.setup, TwoSideAttack.State.passing,
            lambda: self.subbehavior_with_name('capture').is_done_running(),
            'all subbehaviors completed')

        self.add_transition(
            TwoSideAttack.State.passing, TwoSideAttack.State.kicking,
            lambda: (not self.has_subbehavior_with_name('pass') or
                     self.subbehavior_with_name('pass').state == behavior.Behavior.State.completed),
            'Pass completed')

        self.add_transition(
            TwoSideAttack.State.kicking, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('kick').state == behavior.Behavior.State.completed,
            'Kick completed')

        self.robot_points = [
            robocup.Point(-constants.Field.Width / 4.0,
                          3 * constants.Field.Length /
                          4.0), robocup.Point(constants.Field.Width / 4.0,
                                              3 * constants.Field.Length / 4.0)
        ]
        self.passRobot1 = None
        self.passRobot2 = None
        self.captureRobot = None

    @classmethod
    def score(cls):
        if main.game_state().is_playing():
            return 9
        return float("inf")

    def on_enter_setup(self):
        # Add subbehaviors based on information
        capture = skills.capture.Capture()
        self.add_subbehavior(
            skills.move.Move(self.robot_points[0]),
            'moveA',
            required=True)
        self.add_subbehavior(
            skills.move.Move(self.robot_points[1]),
            'moveB',
            required=True)
        self.add_subbehavior(capture, 'capture', required=True)

    def on_exit_setup(self):
        self.passRobot1 = self.subbehavior_with_name('moveA').robot
        self.passRobot2 = self.subbehavior_with_name('moveB').robot
        self.captureRobot = self.subbehavior_with_name('capture').robot
        self.to_exclude = [self.passRobot1, self.passRobot2, self.captureRobot]
        self.remove_all_subbehaviors()

    def on_enter_passing(self):
        # Do shot evaluation here
        win_eval = robocup.WindowEvaluator(main.context())
        for r in self.to_exclude:
            win_eval.add_excluded_robot(r)

        _, rob_1_best_shot = win_eval.eval_pt_to_opp_goal(self.passRobot1.pos)
        _, rob_1_best_pass = win_eval.eval_pt_to_pt(self.captureRobot.pos,
                                                    self.passRobot1.pos, 0.3)
        rob_1_chance = 0
        if (rob_1_best_shot and rob_1_best_pass):
            rob_1_chance = rob_1_best_pass.shot_success * rob_1_best_shot.shot_success

        _, rob_2_best_shot = win_eval.eval_pt_to_opp_goal(self.passRobot2.pos)
        _, rob_2_best_pass = win_eval.eval_pt_to_pt(self.captureRobot.pos,
                                                    self.passRobot2.pos, 0.3)
        rob_2_chance = 0
        if (rob_2_best_shot and rob_2_best_pass):
            rob_2_chance = rob_2_best_pass.shot_success * rob_2_best_shot.shot_success

        _, direct_shot = win_eval.eval_pt_to_opp_goal(self.captureRobot.pos)

        direct_success = 0
        if direct_shot:
            if (self.captureRobot.pos.y < 4):
                direct_success = direct_shot.shot_success * 0.7
            else:
                direct_success = direct_shot.shot_success

            if (direct_shot and direct_success > rob_1_chance and
                    direct_success > rob_2_chance):
                return

        if rob_1_chance > rob_2_chance:
            self.add_subbehavior(
                tactics.coordinated_pass.CoordinatedPass(self.passRobot1.pos),
                'pass')
        else:
            self.add_subbehavior(
                tactics.coordinated_pass.CoordinatedPass(self.passRobot2.pos),
                'pass')

    def on_exit_passing(self):
        self.remove_all_subbehaviors()

    def on_enter_kicking(self):
        kick = skills.pivot_kick.PivotKick()
        kick.target = constants.Field.TheirGoalSegment
        kick.aim_params['desperate_timeout'] = 3
        self.add_subbehavior(kick, 'kick', required=False)

    def on_exit_kicking(self):
        self.remove_all_subbehaviors()
