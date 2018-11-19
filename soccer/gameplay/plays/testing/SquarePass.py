import robocup
import play
import behavior
import skills.move
import skills.capture
import tactics.coordinated_pass
import constants
import main
import enum

## A demo play written during a teaching session to demonstrate play-writing
# Three robots form a triangle on the field and pass the ball A->B->C->A and so on.
class SquarePass(play.Play):
    class State(enum.Enum):
        ## 2 robots get on the corners of a triangle,
        # while a third fetches the ball
        setup = 1

        ## The robots continually pass to each other
        passing = 2

    def __init__(self):
        super().__init__(continuous=True)

        # register states - they're both substates of "running"
        print(robocup.Field_Dimensions.CurrentDimensions.__dict__)
        self.add_state(SquarePass.State.setup,
                       behavior.Behavior.State.running)
        self.add_state(SquarePass.State.passing,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            SquarePass.State.setup, lambda: True,
                            'immediately')
        self.add_transition(
            SquarePass.State.setup, SquarePass.State.passing, lambda: self.
            all_subbehaviors_completed(), 'all subbehaviors completed')

        self.square_points = [
            robocup.Point(constants.Field.Width / 8, constants.Field.Length / 2 ),
            robocup.Point(.9*constants.Field.Width / 2  , constants.Field.Length / 2),
            robocup.Point(constants.Field.Width / 8, constants.Field.Length / 4),
            robocup.Point(.9*constants.Field.Width / 2  , constants.Field.Length / 4),
        ]

    def on_enter_setup(self):
        closestPt = min(self.square_points,
                        key=lambda pt: pt.dist_to(main.ball().pos))

        otherPts = list(self.square_points)
        otherPts.remove(closestPt)

        self.add_subbehavior(skills.move.Move(otherPts[0]), 'move1')
        self.add_subbehavior(skills.move.Move(otherPts[1]), 'move2')
        self.add_subbehavior(skills.move.Move(otherPts[2]), 'move3')
        self.add_subbehavior(skills.capture.Capture(), 'capture')
        

    def on_exit_setup(self):
       
        self.remove_all_subbehaviors()

    def execute_passing(self):
        # If we had a pass in progress before and it finished, remove it
        
        if self.has_subbehaviors():
            if self.all_subbehaviors()[0].is_done_running():
                self.remove_all_subbehaviors()

        # if we're not currently passing, start a new pass
        if not self.has_subbehaviors():
            # pick pass from and to points
            kickFrom = min(self.square_points,
                           key=lambda pt: pt.dist_to(main.ball().pos))
            kickFromIdx = self.square_points.index(kickFrom)
            kickToIdx = (kickFromIdx + 1) % len(self.square_points)
            kickToPt = self.square_points[kickToIdx]

            # add the pass subbehavior
            self.add_subbehavior(
                tactics.coordinated_pass.CoordinatedPass(kickToPt), 'pass')

    def on_exit_passing(self):
        self.remove_all_subbehaviors()
