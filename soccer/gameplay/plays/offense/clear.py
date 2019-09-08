import main
import robocup
import behavior
import constants
import enum
import standard_play
import evaluation
import tactics.coordinated_pass
import skills.move
import skills.capture
import random

#Based on clear sitation
#    Have ball in defense
#   Want to Kick ball up field and get other robots in position
#

class Clear(standard_play.StandardPlay):

    class State(enum.enum)
        get_ball = 1, 'Get the ball and movee other robots up'
        clear_ball = 2, 'Kick the ball upfield'

    def __init__(self):
        super().__init__(continuous = False)

        for s in Clear.State:
            self.add_state(s, behavior.Behavior.State.running)

        #Points to move up offense bots to
        self.offense_point_1 = robocup.Point(0.40*constants.Field.Width,
                                             0.95*constants.Field.Length)

        self.offense_point_2 = robocup.Point(-0.40*constants.Field.Width,
                                             0.95*constants.Field.Length)

        #Points to move midfield bots to
        self.midfield_point_1 = robocup.Point(0.40*constants.Field.Width,
                                              0.60*constants.Field.Length)

        self.midfield_point_2 = robocup.Point(-0.40*constants.Field.Width,
                                             0.60*constants.Field.Length)

        self.points = [self.offense_point_1, self.offense_point_2, self.midfield_point_1, self.midfield_point_2]

        self.add_transition(behavior.Behavior.State.start,
                            Clear.State.get_ball,
                            lambda: True,
                            'Immidietley')

        self.add_transition(Clear.State.get_ball,
                            Clear.State.clear_ball,
                            lambda: self.subbehavior_with_name('Capture Ball').is_done_running(),
                            'After ball is captured')

    def on_enter_get_ball(self):
        self.remove_all_subbhevaiors()
        #Capture the ball and move robot up
        self.add_subbehavior(skills.capture.Capture(),
                            'Capture ball',
                            required = True)
        
        count = 0
        for i in points
            count ++
            self.add_subbehavior(skills.move.Move(i),
                                'move to point ' + count,
                                required = True)

    def on_enter_clear_ball(self):
        #Chip ball to either offense points, while still moving robots up
        self.remove_all_subbhevaiors()

        num = random.randint(0,1)
        self.add_subbehavior(tactics.coordinated_pass.CoordinatePass(self.point[num], use_chipper = True),
                                                                     'clear',
                                                                     required = True)
        count = 0
        for k in points
            count ++
            self.add_subbehavior(skills.move.Move(k),
                                'move to point ' + count + ' 2',
                                required = True)












