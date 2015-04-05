import play
import behavior
import skills.pivot_kick
import skills.move
import tactics.coordinated_pass
import tactics.behavior_sequence
import robocup
import constants
import main
from enum import Enum


## Continually runs a coordinated pass to opposite sides of the field
class OneTouchPass(play.Play):

    ReceiveXCoord = 1
    ReceiveYCoord = constants.Field.Length * 5.0 / 6.0

    class State(Enum):
        passing = 1
        shooting = 2   # Shooting into the goal

    def __init__(self):
        super().__init__(continuous=False)

        for state in OneTouchPass.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                OneTouchPass.State.passing,
                lambda: True,
                'immediately')

        self.add_transition(OneTouchPass.State.passing,
                OneTouchPass.State.shooting,
                lambda: self.subbehavior_with_name('pass').state == behavior.Behavior.State.completed,
                'preparing to shoot')

        self.add_transition(OneTouchPass.State.shooting,
                behavior.Behavior.State.completed,
                lambda: self.subbehavior_with_name('kick').state == behavior.Behavior.State.completed,
                'Shooter shot')

    def reset_receive_point(self):
        pass_bhvr = self.subbehavior_with_name('pass')
        pass_bhvr.receive_point = robocup.Point(OneTouchPass.ReceiveXCoord, OneTouchPass.ReceiveYCoord)

    def on_enter_passing(self):
        pass_bhvr = tactics.coordinated_pass.CoordinatedPass()
        self.add_subbehavior(pass_bhvr, 'pass')
        if pass_bhvr.receive_point == None:
            self.reset_receive_point()


    def on_enter_shooting(self):
        kick = skills.pivot_kick.PivotKick()
        self.add_subbehavior(kick, 'kick', required=True)
        kick.target = constants.Field.TheirGoalSegment
        kick.aim_params['desperate_timeout'] = 3

    def on_exit_passing(self):
        self.remove_subbehavior('pass')

    def on_exit_shooting(self):
        self.remove_subbehavior('kick')

