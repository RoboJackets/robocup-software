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


class OurPlacement(single_robot_composite_behavior.SingleRobotCompositeBehavior):
    class State(enum.Enum):
        dribble=1
        avoid=2

    def __init__(self):
        super().__init__(continuous=False)

        for substate in OurPlacement.State:
            self.add_state(substate, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            OurPlacement.State.dribble, lambda: True,
                            'immediately')
        #place the ball in the target area
        self.add_transition(OurPlacement.State.dribble, OurPlacement.State.avoid,
            lambda: self.subbehavior_with_name('dribble') == behavior.Behavior.State.completed,
            'finished dribbling')
        #if the ball comes out of the target area, put it back
        self.add_transition(
            OurPlacement.State.avoid, OurPlacement.State.dribble,
            lambda: (main.ball().pos - main.game_state().get_ball_placement_point()).mag() < 0.01,
            'ball moved out of target area')

        #this play stays perpetually in the avoid state until a different command is given 

    def on_enter_dribble(self):
        dribble=skills.dribble.Dribble(main.game_state().get_ball_placement_point(),0)
        self.add_subbehavior(dribble, 'dribble', required=True, priority=100)
    
    def on_exit_dribble(self):
        self.robot.set_dribble_speed(0)
        self.remove_subbehavior('dribble')

    def on_enter_avoid(self):
        avoid=skills.move.Move(robocup.point(10,10))
        self.add_subbehavior(avoid,'avoid', required=True, priority=100)

    ## prefer to get assigned robot closest to ball
    def role_requirements(self):
        reqs = super().role_requirements()
        if isinstance(reqs, role_assignment.RoleRequirements):
            reqs.destination_shape = main.ball().pos

        return reqs

#change moveplay to something else
