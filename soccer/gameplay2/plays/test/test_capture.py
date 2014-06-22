import play
import behavior
import enum

# this test repeatedly runs the capture behavior
# it gets SetupDistFromBall away from the ball, then runs the capture behavior.  This repeats forever
class TestCapture(play.Play):

    class State(enum.Enum):
        setup = 1
        capturing = 2

    def __init__(self):
        super().__init__(continuous=True)

        self._robot = None
        self._robot_id = None
        self._sub_behavior = None

        self.add_state(TestCapture.State.setup, behavior.Behavior.State.running)
        self.add_state(TestCapture.State.capturing, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            TestCapture.State.setup,
            lambda: True,
            'immediately')

        self.add_transition(TestCapture.State.setup,
            TestCapture.State.capturing,
            lambda: self.sub_behavior.behavior_state() == behavior.Behavior.State.completed,
            'robot away from ball')

        self.add_transition(TestCapture.State.capturing,
            TestCapture.State.setup,
            lambda: self.sub_behavior.behavior_state() == behavior.Behavior.State.completed,
            'successful capture')


    def on_enter_capturing(self):
        self.sub_behavior = skills.capture.Capture()
        self.sub_behavior.robot = self.robot
    def on_exit_capturing(self):
        self.sub_behavior = None


    def on_enter_setup(self):
        self.sub_behavior = skills.move.Move()
        self.sub_behavior.pos = robocup.Point(0, 0)
        self.sub_behavior.robot = self.robot
    def on_exit_setup(self):
        self.sub_behavior = None


    def execute_running(self):
        self.sub_behavior.run()


    @property
    def robot_id(self):
        return self._robot_id
    @robot_id.setter
    def robot_id(self, value):
        self._robot_id = value
    

    @property
    def robot(self):
        return self._robot
    @robot.setter
    def robot(self, value):
        self._robot = value
        if self.sub_behavior != None:
            self.sub_behavior.robot = value
    

    @play.Play.robots.setter
    def robots(self, robots):
        self._robots = robots

        if robots == None:
            self.robot_id = None

        # use our previous bot if possible
        if self.robot_id != None:
            previous_bot_arr = [bot for bot in robots if bot.shell_id == self.robot_id]
            if len(previous_bot_arr) == 1:
                self.robot = previous_bot_arr[0]
            else:
                self.robot_id = None

        if self.robot_id == None:
            if len(robots) > 0:
                self.robot = robots[0]
                self.robot_id = self.robot.shell_id


    @property
    def sub_behavior(self):
        return self._sub_behavior
    @sub_behavior.setter
    def sub_behavior(self, value):
        self._capture_behavior = value
    

