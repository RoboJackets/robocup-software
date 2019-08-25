import behavior
import robocup
import main
import enum
import constants
import tactics.line_up
import tactics.our_placement
import play


# one robot places the ball, the others just line up and wait
class Placement(play.Play):
    class State(enum.Enum):
        placing = 1  # Normal
        reset = 2  # update information and go back into placing

    def __init__(self):
        super().__init__(continuous=True)

        for state in Placement.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Placement.State.placing, lambda: True,
                            'immediately')
        self.add_transition(Placement.State.placing, Placement.State.reset,
                            lambda: self.check_update(), 'command changed')
        self.add_transition(Placement.State.reset, Placement.State.placing,
                            lambda: True, 'immediately')
        if (main.game_state() is not None):
            self._pos = main.game_state().get_ball_placement_point()
            self._our_restart = main.game_state().is_our_restart()
        else:
            self._pos = None
            self._our_restart = None

    def create_lineup(self):
        xsize = constants.Field.Width / 2 - .5
        if self._pos.x > 0:
            xsize = -xsize

        return robocup.Segment(
            robocup.Point(xsize, 1), robocup.Point(xsize, 2.5))

    def check_update(self):
        #for some reason using != on two robocup points always returns false here
        return (self._pos - main.game_state().get_ball_placement_point()).mag(
        ) != 0 or self._our_restart != main.game_state().is_our_restart()

    def on_enter_reset(self):
        self._pos = main.game_state().get_ball_placement_point()
        self._our_restart = main.game_state().is_our_restart()

    def on_enter_placing(self):
        if self._our_restart:
            self.placer = tactics.our_placement.OurPlacement()
            self.add_subbehavior(self.placer,
                                 'placer',
                                 required=True,
                                 priority=90)

        line_up = tactics.line_up.LineUp(self.create_lineup())
        self.add_subbehavior(line_up, 'line_up', required=False, priority=80)

    def execute_placing(self):
        main.debug_drawer().draw_circle(self._pos, 0.1, constants.Colors.Green,
                                        "Place")
        main.debug_drawer().draw_circle(self._pos, 0.5, constants.Colors.Red,
                                        "Avoid")

    def on_exit_placing(self):
        self.remove_all_subbehaviors()

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_our_placement() else float("inf")

    @classmethod
    def is_restart(cls):
        return True

    @classmethod
    def handles_goalie(cls):
        return True
