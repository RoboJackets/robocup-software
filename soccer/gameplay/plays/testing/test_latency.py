import main
import tools
import behavior
import behavior_sequence
import constants
import math
import time
import enum
import play
import skills.latency
import robocup
import role_assignment

# A shameless rip from repeated lineup
class TestLatency(play.Play):
    class State(enum.Enum):
        left = 0
        right = 1
        pause = 2

    PAUSE = 2.0
    BUFFER = .7

    def __init__(self):
        super().__init__(continuous=True)

        self.shell_id = None

        self.side_start = time.time()

        behaviors = [
            skills.latency.Latency(self.generate_point(-TestLatency.BUFFER)),
            tools.sleep.SleepBehavior(TestLatency.PAUSE),
            skills.latency.Latency(self.generate_point(TestLatency.BUFFER)),
            tools.sleep.SleepBehavior(TestLatency.PAUSE),
        ]
        b = behavior_sequence.BehaviorSequence(
            continuous=True, repeat=True, behaviors=behaviors)
        self.add_subbehavior(b, 'line up behavior')

    # x_multiplier is a 1 or -1 to indicate which side of the field to be on
    # 1 is right, -1 is left
    def generate_point(self, x_multiplier):
        x = (constants.Field.Width / 2 - constants.Robot.Radius * 2
             ) * x_multiplier
        point = robocup.Point(x, 1.2)
        return point

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
