import robocup
import constants
import single_robot_composite_behavior
import enum
import behavior
import main
import skills.move
import subprocess

## Motivates, encourages, and directs the team.
class Coach(single_robot_composite_behavior.SingleRobotCompositeBehavior):

    MaxSpinAngle = 360
    SpinPerTick = 1

    OurScore = 0
    TheirScore = 0

    class State(enum.Enum):
        watching = 0
        celebrating = 1
        motivating = 2

    def __init__(self):
        super().__init__(continuous=True)
        self.spin_angle = 0

        self.add_state(Coach.State.watching, behavior.Behavior.State.running)
        self.add_state(Coach.State.celebrating, behavior.Behavior.State.running)
        self.add_state(Coach.State.motivating, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start, self.State.watching,
                            lambda: True, 'immediately')

        # Transitions into and out of celebration
        self.add_transition(self.State.watching, self.State.celebrating,
                            lambda: self.our_score_increased(), 'goal scored')
        self.add_transition(self.State.celebrating, self.State.watching,
                            lambda: self.spin_angle > Coach.MaxSpinAngle, 'done with celebration')

        self.add_transition(self.State.watching, self.State.motivating,
                            lambda: self.their_score_increased(), 'Let a goal through')
        self.add_transition(self.State.motivating, self.State.watching,
                            lambda: True, 'done talking')

    def our_score_increased(self):
        if (Coach.OurScore < main.game_state().our_score):
            return True
        else:
            Coach.OurScore = main.game_state().our_score
            return False
    def their_score_increased(self):
        if (Coach.TheirScore < main.game_state().their_score):
            return True
        else:
            Coach.TheirScore = main.game_state().their_score
            return False

    def on_enter_running(self):
        move_point = robocup.Point(-constants.Field.Width / 2 - constants.Robot.Radius * 2,
                                   constants.Field.Length / 3)

        move = skills.move.Move(move_point)
        self.add_subbehavior(move, 'coach')

    def on_exit_running(self):
        self.remove_all_subbehaviors()

    def execute_watching(self):
        move = self.subbehavior_with_name('coach')

        # Face ball
        if (main.ball().valid and move.robot is not None):
            move.robot.face(main.ball().pos)

            # Don't cross to the opponent's side of the field
            max_y = constants.Field.Length / 2 - constants.Robot.Radius * 2
            min_y = constants.Robot.Radius * 2
            move_y_pos = max(min(main.ball().pos.y, max_y), min_y)

            move.pos = robocup.Point(move.pos.x, move_y_pos)

    def on_enter_celebrating(self):
        Coach.OurScore = main.game_state().our_score

    def execute_celebrating(self):
        move = self.subbehavior_with_name('coach')
        # Face ball
        if (move.robot is not None):
            angle = move.robot.angle
            facing_point = robocup.Point.direction(angle) + move.robot.pos
            facing_point.rotate(move.robot.pos, Coach.SpinPerTick)
            self.spin_angle += Coach.SpinPerTick
            move.robot.face(facing_point)

    def on_enter_motivating(self):
        Coach.TheirScore = main.game_state().their_score

        print("\n\nListen up:")
        print(Coach.fortune_wrapper())

    @staticmethod
    ## Returns a fortune.
    def fortune_wrapper():
        try:
            # Prefer -o if available
            fortune = subprocess.check_output(["fortune", "-o"])
            fortune = fortune.decode("utf-8")
        except subprocess.CalledProcessError:
            try:
                # Fall back to any fortune
                fortune = subprocess.check_output(["fortune"])
                fortune = fortune.decode("utf-8")
            except subprocess.CalledProcessError:
                # We must be on Windows, give a sna... enlightening remark. (install fortune!)
                fortune = '''"There is no such thing as a typical user, and if there were, it wouldn't be you" - #usability'''
        return fortune
