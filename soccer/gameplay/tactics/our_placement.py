import single_robot_composite_behavior
import behavior
import main
import constants
import role_assignment
import enum
import robocup
import skills
import constants
import skills.pivot_kick
import skills.capture
import skills.move
import skills.move_direct


class OurPlacement(single_robot_composite_behavior.SingleRobotCompositeBehavior):
    class State(enum.Enum):
        capturing = 1  #waiting
        placing = 2  #setup
        standby = 3  #ready

    def __init__(self):
        super().__init__(continuous=False)

        for substate in OurPlacement.State:
            self.add_state(substate, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            OurPlacement.State.capturing, lambda: True,
                            'immediately')

        self.add_transition(
            OurPlacement.State.capturing, OurPlacement.State.placing,
            lambda: self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed,
            'done capturing')

        self.add_transition(
            OurPlacement.State.placing, OurPlacement.State.standby,
            lambda: self.subbehavior_with_name('place').state == behavior.Behavior.State.completed,
            'done placing')

        self.add_transition(
            OurPlacement.State.standby, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('standby').state == behavior.Behavior.State.completed,
            'finished standby')

    def on_enter_capturing(self):
        print("CAPTURING")
        capture = skills.capture.Capture()
        self.add_subbehavior(capture, 'capture', required=True, priority=100)

    def on_execute_capturing(self):
        main.system_state().draw_circle(main.game_state().get_ball_placement_point(),5,constants.Colors.White,"Place")

    def on_exit_capturing(self):
        self.remove_subbehavior('capture')

    def on_enter_placing(self):
        print("PLACING")
        place = skills.move_direct.MoveDirect(main.game_state().get_ball_placement_point())
        print("SET UP PLACE")
        self.add_subbehavior(place, 'place', required=True, priority=100)
        print("ADDED SUBBEHAVIOR")

    def on_execute_placing(self):
        print("EXECUTING PLACING")
        main.system_state().draw_circle(main.game_state().get_ball_placement_point(),5,constants.Colors.White,"Place")

    def on_exit_placing(self):
        print("DONE PLACING")
        self.remove_subbehavior('place')

        direction = constants.Field.CenterPoint - main.ball().pos
        self.robot.move_to(main.ball().pos + direction.normalized())

    def on_enter_standby(self):
        print("STANDBY  ---------------------------------------")
        standby = skills.move.Move(10, 10)
        self.add_subbehavior(standby, 'standby', required=True, priority=100)

    def on_execute_standby(self):
        main.system_state().draw_circle(main.game_state().get_ball_placement_point(),5,constants.Colors.White,"Place")

    def on_exit_standby(self):
        self.remove_subbehavior('standby')

    ## prefer to get assigned robot closest to ball
    """
    def role_requirements(self):
        reqs = super().role_requirements()
        if isinstance(reqs, role_assignment.RoleRequirements):
            reqs.destination_shape = main.ball().pos

        return reqs
"""
