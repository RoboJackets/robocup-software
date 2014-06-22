import behavior
import skill
from enum import Enum


class Capture(skill.Skill):

    class State(Enum):
        course_approach = 1
        fine_approach = 2


    def __init__(self):
        super().__init__(continuous=False)

        self.add_state(Capture.State.course_approach, behavior.Behavior.State.running)
        self.add_state(Capture.State.fine_approach, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start, Capture.State.course_approach, lambda: True, 'immediately')
        self.add_transition(Capture.State.course_approach, Capture.State.fine_approach,
            lambda:
                True,
            'dist to ball < threshold')
        self.add_transition(Capture.State.fine_approach, behavior.Behavior.State.completed,
            lambda:
                True,
            'has ball')
