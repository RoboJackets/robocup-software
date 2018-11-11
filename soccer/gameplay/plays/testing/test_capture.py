import play
import behavior
import skills.move
import skills.capture
import enum
import robocup
import role_assignment


# this test repeatedly runs the capture behavior
class TestCapture(play.Play):
    # True makes the robot rotate around the ball after capturing
    RotateOnCapture = False

    class State(enum.Enum):
        setup = 1
        capturing = 2
        rotating = 3

    def __init__(self):
        super().__init__(continuous=True)

        self.shell_id = None
        self.dribbler_power = 128
        self.face_target = robocup.Point(0,0)
        self.max_turn_count = 2
        self.turn_count = self.max_turn_count

        for state in TestCapture.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(
            behavior.Behavior.State.start, TestCapture.State.setup,
            lambda: True, 'immediately')

        self.add_transition(
            TestCapture.State.setup, TestCapture.State.capturing,
            lambda: self.subbehavior_with_name('move').state == behavior.Behavior.State.completed,
            'capturing ball')

        self.add_transition(
            TestCapture.State.capturing, TestCapture.State.rotating,
            lambda: self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed and TestCapture.RotateOnCapture == True,
            'rotating to check capture')

        self.add_transition(
            TestCapture.State.rotating, TestCapture.State.capturing,
            lambda: self.subbehavior_with_name('aim').state == behavior.Behavior.State.failed,
            'got away from ball, recapturing')

        self.add_transition(
            TestCapture.State.capturing, TestCapture.State.setup,
            lambda: self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed and TestCapture.RotateOnCapture == False,
            'successful capture returning to setup')

        self.add_transition(
            TestCapture.State.rotating, TestCapture.State.setup,
            lambda: self.turn_count < 0,
            'successful capture returning to setup')

    def on_enter_capturing(self):
        self.add_subbehavior(skills.capture.Capture(),
                             'capture',
                             required=True)

    def on_exit_capturing(self):
        self.remove_subbehavior('capture')

    def on_enter_setup(self):
        self.turn_count = self.max_turn_count
        m = skills.move.Move()
        m.pos = robocup.Point(0, 1.5)
        self.add_subbehavior(m, 'move', required=True)

    def on_exit_setup(self):
        self.remove_subbehavior('move')

    def on_enter_rotating(self):
        aim = skills.aim.Aim()
        aim.target_point = self.face_target
        aim.dribbler_power = self.dribbler_power
        self.add_subbehavior(aim, 'aim', required=True)

    def execute_rotating(self):
        aim = self.subbehavior_with_name('aim')
        if self.turn_count >= 0:
            if aim.state == skills.aim.Aim.State.aimed:
                aim.target_point.rotate(aim.robot.pos, 180.0)
                self.turn_count -= 1

    def on_exit_rotating(self):
        self.remove_subbehavior('aim')

    def execute_running(self):
        for bhvr in self.all_subbehaviors():
            if bhvr.robot != None:
                self.shell_id = bhvr.robot.shell_id()

    def role_requirements(self):
        reqs = super().role_requirements()
        if self.shell_id != None:
            for req in role_assignment.iterate_role_requirements_tree_leaves(reqs):
                req.required_shell_id = self.shell_id
        return reqs
