import play
import behavior
import single_robot_behavior
import robocup
import constants
import math

# this play tests the effect of sending motion commands to the bot that jitter
#
#     ^
#    /
#   / vel cmd 1
#  /
#
# ^
# \
#  \  vel cmd 2
#   \
#
# we constantly alter between the above two vel commands to see if the robot goes straight and how fast it goes


class Jitterer(single_robot_behavior.SingleRobotBehavior):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

    def on_exit_start(self):
        self.iter_count = 0

    def execute_running(self):
        self.iter_count += 1

        if True:
            speed = .4

            if self.iter_count % 2 == 0:
                vel = robocup.Point.direction(math.pi / 4.0) * speed
            else:
                vel = robocup.Point.direction(math.pi * 3.0 / 4.0) * speed
        else:
            vel = robocup(0, speed)

        self.robot.set_world_vel(vel)


class TestJitteryMotion(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        j = Jitterer()
        self.add_subbehavior(j, 'jitterer', required=False)
