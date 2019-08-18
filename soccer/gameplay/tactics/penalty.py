import single_robot_composite_behavior
import behavior
import main
import role_assignment
import enum
import robocup
import skills
import constants
import planning_priority
import skills.line_kick

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
        backoff = 0.5

        self.robot.disable_avoid_ball()

        self.robot.move_to(main.ball().pos - robocup.Point(0, backoff))
        # FIXME this is old code to stick to the penalty area if possible.
        # penalty_mark = robocup.Point(
        #     0, constants.Field.Length - constants.Field.PenaltyDist)
        # Now we just track the ball. Find out if this is a good idea.
        # if main.ball().pos.near_point(penalty_mark, 0.5):
        #     self.robot.move_to(main.ball().pos - robocup.Point(0, backoff))
        # else:
        #     self.robot.move_to(penalty_mark - robocup.Point(0, backoff))

        self.robot.face(main.ball().pos)

    def on_enter_ready(self):
        kick = skills.line_kick.LineKick()
        self.add_subbehavior(kick,
                             'kick',
                             required=True,
                             priority=planning_priority.PENALTY_KICKER)

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
