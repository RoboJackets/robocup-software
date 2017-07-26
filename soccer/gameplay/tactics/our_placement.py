import single_robot_composite_behavior
import behavior
import main
import constants
import role_assignment
import enum
import robocup
import skills
import skills.dribble
import skills.move
import time


class OurPlacement(single_robot_composite_behavior.SingleRobotCompositeBehavior
                   ):
    class State(enum.Enum):
        dribble = 1
        pause = 2
        avoid = 3

    def __init__(self):
        super().__init__(continuous=False)

        self.pause_time = 0

        for substate in OurPlacement.State:
            self.add_state(substate, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            OurPlacement.State.dribble, lambda: True,
                            'immediately')

        #place the ball in the target area
        self.add_transition(
            OurPlacement.State.dribble, OurPlacement.State.pause,
            lambda: self.subbehavior_with_name('dribble').state == behavior.Behavior.State.completed,
            'finished dribbling')

        self.add_transition(OurPlacement.State.pause, OurPlacement.State.avoid,
                            lambda: time.time() - self.pause_time >= 1,
                            'pause for 1 seconds')

        #if the ball comes out of the target area, put it back
        self.add_transition(
            OurPlacement.State.avoid, OurPlacement.State.dribble,
            lambda: (main.ball().pos - main.game_state().get_ball_placement_point()).mag() > 0.1,
            'ball moved out of target area')

        #this play stays perpetually in the avoid state until a different command is given

    def on_enter_dribble(self):
        dribble = skills.dribble.Dribble(main.game_state(
        ).get_ball_placement_point())
        self.add_subbehavior(dribble, 'dribble', required=True, priority=100)

    def execute_dribble(self):
        self.robot.is_ball_placer = True
        # self.robot.set_max_speed(1)

    def on_exit_dribble(self):
        self.robot.set_dribble_speed(0)
        self.remove_all_subbehaviors()

    def on_enter_pause(self):
        self.pause_time = time.time()

    def execute_pause(self):
        self.robot.disable_avoid_ball()

    def execute_avoid(self):
        self.robot.is_ball_placer = True
        #the avoid radius is 6cm larger than the area we need to stay out of plus the radius of the robot just for safety
        self.robot.face(main.ball().pos)
        self.robot.move_to(main.ball().pos + ((self.robot.pos - main.ball().pos
                                               ).normalized() * .65))

    ## prefer to get assigned robot closest to ball
    def role_requirements(self):
        reqs = super().role_requirements()
        if isinstance(reqs, role_assignment.RoleRequirements):
            reqs.destination_shape = main.ball().pos

        return reqs
