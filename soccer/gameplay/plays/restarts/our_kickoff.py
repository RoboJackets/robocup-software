import play
import behavior
import robocup
import skills.line_kick
import tactics.defense
import main
import constants
import enum
import role_assignment


class OurKickoff(play.Play):

    KickPower = 0.5
    ChipPower = 1.0

    class State(enum.Enum):
        setup = 1
        kick = 2

    def __init__(self):
        super().__init__(continuous=False)

        self._kicker_shell_id = None

        for state in OurKickoff.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            OurKickoff.State.setup, lambda: True,
                            'immediately')

        self.add_transition(OurKickoff.State.setup, OurKickoff.State.kick,
                            lambda: not main.game_state().is_setup_state(),
                            "referee leaves setup")

        self.add_transition(
            OurKickoff.State.kick, behavior.Behavior.State.completed,
            lambda: self.has_subbehavior_with_name('kicker') and self.subbehavior_with_name('kicker').is_done_running(),
            "kicker finished")

        # TODO: verify that these values are right - I'm fuzzy on my matrix multiplication...
        idle_positions = [
            robocup.Point(0.7, constants.Field.Length / 2.0 - 0.2),
            robocup.Point(-0.7, constants.Field.Length / 2.0 - 0.2)
        ]
        self.centers = []
        for i, pos_i in enumerate(idle_positions):
            center_i = skills.move.Move(pos_i)
            self.add_subbehavior(center_i,
                                 'center' + str(i),
                                 required=False,
                                 priority=4 - i)
            self.centers.append(center_i)

        self.add_subbehavior(tactics.defense.Defense(),
                             'defense',
                             required=False)

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_our_kickoff() else float("inf")

    @classmethod
    def is_restart(cls):
        return True

    @classmethod
    def handles_goalie(cls):
        return True

    def on_enter_setup(self):
        mover = skills.move.Move(robocup.Point(0, constants.Field.Length / 2.0
                                               - 0.30))
        self.add_subbehavior(mover, 'move', required=False, priority=5)

    def execute_setup(self):
        for center in self.centers:
            if center.robot is not None:
                center.robot.face(main.ball().pos)

    def on_enter_kick(self):
        if self.subbehavior_with_name('move').robot is not None:
            self._kicker_shell_id = self.subbehavior_with_name(
                'move').robot.shell_id()
        self.remove_subbehavior('move')
        kicker = skills.line_kick.LineKick()
        kicker.target = constants.Field.TheirGoalSegment
        # kicker.use_chipper = True
        kicker.kick_power = OurKickoff.KickPower
        kicker.chip_power = OurKickoff.ChipPower
        self.add_subbehavior(kicker, 'kicker', required=True, priority=5)

    def execute_kick(self):
        # all centers should face the ball
        for center in self.centers:
            if center.robot is not None:
                center.robot.face(main.ball().pos)

    def role_requirements(self):
        reqs = super().role_requirements()
        if 'move' in reqs:
            for r in role_assignment.iterate_role_requirements_tree_leaves(
                    reqs['move']):
                r.chipper_preference_weight = role_assignment.PreferChipper
        if 'kicker' in reqs:
            for r in role_assignment.iterate_role_requirements_tree_leaves(
                    reqs['kicker']):
                r.previous_shell_id = self._kicker_shell_id
        return reqs
