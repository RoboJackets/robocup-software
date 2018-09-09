import single_robot_composite_behavior
import main
import behavior
import math
import time
import enum
import play
import skills
import robocup

class Latency(single_robot_composite_behavior.SingleRobotCompositeBehavior):
    class State(enum.Enum):
        move = 0

    def __init__(self, targetPos=robocup.Point(0, 0)):
        super().__init__(continuous=False)

        for state in Latency.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(
            behavior.Behavior.State.start, Latency.State.move,
            lambda: True,
            'moving')

        self.add_transition(
            Latency.State.move, behavior.Behavior.State.completed,
            lambda: self.latencySet == True and self.subbehavior_with_name('move').state == behavior.Behavior.State.completed,
            'done')

        self._targetPos = targetPos
        self.latencySet = False
        self.latency = 0.0
        self.startingPos = robocup.Point(0, 0)

    def on_enter_move(self):
        self.add_subbehavior(skills.move.Move(self._targetPos), 'move', required=True)
        self.latency = time.time()
        self.startingPos = self.robot.pos

    def execute_running(self):
        if self.robot != None:
            if (self.robot.pos - self.startingPos).mag() > 0.001 and self.latencySet == False:
                self.latencySet = True
                self.latency = time.time() - self.latency

    def on_exit_move(self):
        self.remove_subbehavior('move')
        print("final" + str(self.latency))

    def role_requirements(self):
        reqs = super().role_requirements()
        return reqs
