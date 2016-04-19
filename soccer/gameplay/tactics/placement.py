import single_robot_composite_behavior
import behavior
import main
import role_assignment
import enum
import robocup
import skills
import constants
import skills.pivot_kick
import skills.capture
import skills.move


class Placement(single_robot_composite_behavior.SingleRobotCompositeBehavior):
    class State(enum.Enum):
        capturing = 1  #waiting
        placing = 2  #setup
        standby = 3  #ready

    def __init__(self):
        super().__init__(continuous=False)

        for substate in Placement.State:
            self.add_state(substate, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Placement.State.capturing, lambda: True,
                            'immediately')

        self.add_transition(
            Placement.State.capturing, Placement.State.placing,
            lambda: self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed,
            'done capturing')

        self.add_transition(
            Placement.State.placing, Placement.State.standby,
            lambda: self.subbehavior_with_name('placing').state == behavior.Behavior.State.completed,
            'done placing')

        self.add_transition(
            Placement.State.standby, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('standby').state == behavior.Behavior.State.completed,
            'finished standby')

    def on_enter_capturing(self):
        capture = skills.capture.Capture()
        self.add_subbehavior(capture, 'capture', required=True, priority=100)

    def on_exit_capturing(self):
        self.remove_subbehavior('capture')

    def on_enter_placing(self):
        place = skills.move.Move(0, 0)
        self.add_subbehavior(place, 'place', required=True, priority=100)

    def on_exit_placing(self):
        self.remove_subbehavior('place')

        direction = constants.Field.CenterPoint - main.ball().pos
        self.robot.move_to(main.ball().pos + direction.normalized())

    def on_enter_standby(self):
        standby = skills.move.Move(0, 0)
        self.add_subbehavior(standby, 'standby', required=True, priority=100)

    def on_exit_standby(self):
        self.remove_subbehavior('standby')

    ## prefer to get assigned robot closest to ball
    def role_requirements(self):
        reqs = super().role_requirements()
        if isinstance(reqs, role_assignment.RoleRequirements):
            reqs.destination_shape = main.ball().pos

        return reqs
