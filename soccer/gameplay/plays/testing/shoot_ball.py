import play
import tactics.line_up
import behavior_sequence
import tools.sleep
import robocup
import constants
import time
import enum
import skills
import behavior
import tactics.defense
import skills.pivot_kick
import main


class ShootBall(play.Play):

    class State(enum.Enum):
        start_run = 0
        running = 1
        shoot = 3
        complete = 4
        missed = 5

    # initialize constants, etc.
    def __init__(self):
        # not sure if we need this
        super().__init__(continuous=True)


        self.add_state(ShootBall.State.start_run, behavior.Behavior.State.start)

        self.add_state(ShootBall.State.running, behavior.Behavior.State.running)

        self.add_state(ShootBall.State.shoot, behavior.Behavior.State.running)

        #self.add_state(ShootBall.State.complete, behavior.Behavior.State.complete)

        #self.add_state(ShootBall.State.missed, behavior.Behavior.State.failed)

        ball_missed = (lambda: ( (self.get_ball_pos().x > 0.5 or (self.get_ball_pos().x < -0.5  )) or (self.get_ball_pos().y < 9)))

        self.add_transition(behavior.Behavior.State.start, self.State.start_run, lambda:True, 'immediately')

        self.add_transition(self.State.start_run, self.State.running, lambda:True, 'immediately')

        #will need to code lambda function for checking a shot can be taken before shooting
        self.add_transition(self.State.running, self.State.shoot, lambda:True, 'if_shot_can_be_taken')

        #self.add_transition(self.State.shoot, self.State.complete, lambda:True, 'made_shot')

        #self.add_transition(self.State.complete, self.State.missed, ball_missed, 'missed')

    def get_ball_pos(self):
        return main.ball().pos

    def on_enter_shoot(self):
        print("starting")
        #kick = self.subbehavior_with_name('kick')
        #b = tactics.defense.Defense()
        kick = skills.pivot_kick.PivotKick()

        kick.target = constants.Field.TheirGoalSegment
        kick.aim_params['desperate_timeout'] = 3
        self.add_subbehavior(kick, 'kick', required=False)

    def execute_shoot(self):
        print("shooting")
        kick = self.subbehavior_with_name('kick')

        #sublist = self.subbehavior_with_name('defense').all_subbehaviors()
        #roblist = []
        #for behavior in sublist:
        #    roblist.append(behavior.robot)

        #kick.shot_obstacle_ignoring_robots = roblist

        if kick.is_done_running():
            kick.restart()

#    def on_exit_running(self):
#        self.remove_subbehavior('defense')
#        self.remove_subbehavior('kick')
