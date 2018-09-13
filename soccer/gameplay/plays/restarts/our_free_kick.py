import standard_play
import behavior
import skills.move
import skills.pivot_kick
import constants
import robocup
import math
import main
import tactics.coordinated_pass
import evaluation.passing_positioning


class OurFreeKick(standard_play.StandardPlay):

    Running = False
    BumpKickPower = 0.01
    FullKickPower = 1
    MaxShootingAngle = 80
    # Untested as of now
    MaxChipRange = 3
    MinChipRange = 0.3

    def __init__(self, indirect=None):
        super().__init__(continuous=True)

        # If we are indirect we don't want to shoot directly into the goal
        gs = main.game_state()

        if (main.ball().pos.y > constants.Field.Length / 2):
            self.indirect = gs.is_indirect()
        else:
            self.indirect = False

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        # FIXME: this could also be a PivotKick
        kicker = skills.line_kick.LineKick()
        # kicker.use_chipper = True
        kicker.min_chip_range = OurFreeKick.MinChipRange
        kicker.max_chip_range = OurFreeKick.MaxChipRange

        gap = evaluation.shooting.find_gap(
            max_shooting_angle=OurFreeKick.MaxShootingAngle)

        kicker.target = gap

        shooting_line = robocup.Line(main.ball().pos, gap)

        # If we are at their goal, shoot full power
        if shooting_line.segment_intersection(constants.Field.TheirGoalSegment) is not None:
            kicker.kick_power = self.FullKickPower
        # If we are aiming in the forward direction and not at one of the "endzones", shoot full power
        elif (shooting_line.line_intersection(constants.Field.FieldBorders[0]) or
              shooting_line.line_intersection(constants.Field.FieldBorders[2])) and \
              gap.y - main.ball().pos.y > 0:
            kicker.kick_power = self.FullKickPower
        # If we are probably aiming down the field, slowly kick so we dont carpet
        else:
            kicker.kick_power = self.BumpKickPower

        # Try passing if we are doing an indirect kick
        if self.indirect:
            receive_pt, receive_value = evaluation.passing_positioning.eval_best_receive_point(main.ball().pos)

            # Check for valid target pass position
            if receive_value != 0:
                pass_behavior = tactics.coordinated_pass.CoordinatedPass(
                    receive_pt,
                    None,
                    (kicker, lambda x: True),
                    receiver_required=False,
                    kicker_required=False,
                    prekick_timeout=9)
                # We don't need to manage this anymore
                self.add_subbehavior(pass_behavior, 'kicker')
            else:
                self.add_subbehavior(kicker, 'kicker', required=False, priority=5)
        else:
            self.add_subbehavior(kicker, 'kicker', required=False, priority=5)

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('kicker').is_done_running() and self.subbehavior_with_name('kicker').state != tactics.coordinated_pass.CoordinatedPass.State.timeout,
            'kicker completes')

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if OurFreeKick.Running or (
            gs.is_ready_state() and gs.is_our_free_kick()) else float("inf")

    def execute_running(self):
        if self.indirect \
           and self.subbehavior_with_name('kicker').state == tactics.coordinated_pass.CoordinatedPass.State.timeout:
            self.indirect = False
            self.remove_subbehavior('kicker')
            kicker = skills.line_kick.LineKick()
            kicker.target = constants.Field.TheirGoalSegment
            self.add_subbehavior(kicker, 'kicker', required=False, priority=5)

        if self.indirect:
            passState = self.subbehavior_with_name('kicker').state
            OurFreeKick.Running = passState == tactics.coordinated_pass.CoordinatedPass.State.receiving or \
                                  passState == tactics.coordinated_pass.CoordinatedPass.State.kicking

    def on_enter_running(self):
        OurFreeKick.Running = False

    def on_exit_running(self):
        OurFreeKick.Running = False

    @classmethod
    def is_restart(cls):
        return True
