import play
import behavior
import single_robot_behavior
import robocup
import constants


class Jitterer(single_robot_behavior.SingleRobotBehavior):

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')


    def on_exit_start(self):
        self.iter_count = 0


    def execute_running(self):
        self.iter_count += 1

        vel = robocup.Point(0, 1)
        vel.rotate(math.pi / 4.0) # 45 degrees

        if self.iter_count % 2 == 0:
            vel.rotate(math.pi / 2.0)

        self.robot.set_world_vel(vel)


class TestJitteryMotion(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')

        j = Jitterer()
        self.add_subbehavior(j, 'jitterer', required=False)
