import play
import single_robot_composite_behavior
import robocup
import constants
import behavior
import skills.face
import enum
import math
import time


class Facer(single_robot_composite_behavior.SingleRobotCompositeBehavior):

    class State(enum.Enum):
        rotating = 1
        pausing = 2


    def __init__(self):
        super().__init__(continuous=True)

        for state in Facer.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.angle = math.pi / 2.0
        self.point = robocup.Point(0, constants.Field.Length / 4.0)

        self.add_transition(behavior.Behavior.State.start,
            Facer.State.rotating,
            lambda: True,
            'immediately')
        self.add_transition(Facer.State.rotating,
            Facer.State.pausing,
            lambda: self.subbehavior_with_name('rotate').state == behavior.Behavior.State.completed,
            'rotated to appropriate angle')
        self.add_transition(Facer.State.pausing,
            Facer.State.rotating,
            lambda: self.done_pausing(),
            'done pausing')


    @property
    def angle(self):
        return self._angle
    @angle.setter
    def angle(self, value):
        self._angle = value
    

    # where the robot sits on the field as it rotates
    # Default: center of our half of the field
    @property
    def point(self):
        return self._point
    @point.setter
    def point(self, value):
        self._point = value


    def on_enter_rotating(self):
        self.add_subbehavior(skills.face.Face(self.point, self.angle), 'rotate')
    def on_exit_rotating(self):
        self.remove_subbehavior('rotate')
        self.angle += math.pi / 2

    def on_enter_pausing(self):
        self.add_subbehavior(skills.move.Move(self.point), 'hold')
        self.pause_start_time = time.time()
    def on_exit_pausing(self):
        self.remove_subbehavior('hold')

    def done_pausing(self):
        return (time.time() - self.pause_start_time) > 1.5



## This play rotates the bot 90 degrees, pauses, and repeats
# It's useful for tuning the angle PID controller
class TuneFacing(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        rotater = Facer()
        self.add_subbehavior(rotater, 'rotater', required=False)
