import single_robot_composite_behavior
import behavior
import main
import role_assignment
import enum
import robocup
import skills
import constants
import skills.pivot_kick


## This class is currently not used for anything
# This was ported from our old C++ gameplay system and served a purpose then, but is unused now
class Penalty(single_robot_composite_behavior.SingleRobotCompositeBehavior):
    class State(enum.Enum):
        waiting = 1
        setup = 2
        ready = 3

    def __init__(self):
        super().__init__(continuous=False)

        for substate in Penalty.State:
            self.add_state(substate, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Penalty.State.waiting, lambda: True, 'immediately')

        self.add_transition(Penalty.State.waiting, Penalty.State.setup,
                            lambda: main.game_state().is_setup_state(),
                            'ref says time to setup')

        for state in [Penalty.State.waiting, Penalty.State.setup]:
            self.add_transition(state, Penalty.State.ready,
                                lambda: main.game_state().is_ready_state(),
                                'ref says ready')

        self.add_transition(
            Penalty.State.ready, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('kick').state == behavior.Behavior.State.completed,
            'done kicking')

    def execute_waiting(self):
        self.robot.face(main.ball().pos)

    def execute_setup(self):
        penalty_mark = robocup.Point(
            0, constants.Field.Length - constants.Field.PenaltyDist)
        backoff = 0.5
        if main.ball().pos.near_point(penalty_mark, 0.5):
            self.robot.move_to(main.ball().pos + (main.ball(
            ).pos - robocup.Point(0, constants.Field.Length).normalized()) *
                               backoff)
        else:
            self.robot.move_to(penalty_mark - robocup.Point(0, backoff))

        self.robot.face(main.ball().pos)

    def on_enter_ready(self):
        kick = skills.pivot_kick.PivotKick()
        self.add_subbehavior(kick, 'kick', required=True, priority=100)

    def on_execute_ready(self):
        self.robot.is_penalty_kicker = True

    def on_exit_ready(self):
        self.remove_subbehavior('kick')

    ## prefer to get assigned robot closest to ball
    def role_requirements(self):
        reqs = super().role_requirements()
        if isinstance(reqs, role_assignment.RoleRequirements):
            reqs.destination_shape = main.ball().pos

        return reqs
