import robocup
import constants
import play
import enum
import behavior
import main
import skills.move
import plays.testing.line_up
import time


class FollowBall(play.Play):
    class State(enum.Enum):
        # We only need one state, and we'll transition to itself when we want to update.
        ball_moved = 0
        nothing = 1

    def __init__(self):
        super().__init__(continuous=True)

        self.ball_pos = self.get_ball_pos()

        self.add_state(FollowBall.State.ball_moved,
                       behavior.Behavior.State.running)

        self.add_state(FollowBall.State.nothing,
                       behavior.Behavior.State.running)

        ball_has_moved = (lambda: (self.get_ball_pos().x != self.ball_pos.x) or
                          (self.get_ball_pos().y != self.ball_pos.y))
        #in_ball_radius = (lambda : False)
        print(main.our_robots()[1].shell_id)

        self.add_transition(behavior.Behavior.State.start,
                            self.State.nothing, lambda: True, 'immediately')

        self.add_transition(self.State.ball_moved,
                            self.State.nothing, lambda: True,
                            'ball staying still')

        self.add_transition(self.State.nothing, self.State.ball_moved,
                            ball_has_moved, 'ball moved')

    def get_ball_pos(self):
        return main.ball().pos

    def on_enter_nothing(self):
        print(self.ball_pos)
        print(self.get_ball_pos())

        self.remove_all_subbehaviors()
        print('Ball doing nothing')
        move_point = robocup.Point(self.ball_pos.x + 0.1,
                                   self.ball_pos.y - 0.1)
        self.add_subbehavior(skills.move.Move(move_point), 'Robot' + str(1))

    def execute_noting(self):
        print('Execute Nothing')
        print(self.ball_pos)
        print(self.get_ball_pos())

    def on_enter_ball_moved(self):
        print('On Ball moved')
        self.ball_pos = self.get_ball_pos()
        print(self.ball_pos)
        print(self.get_ball_pos())
