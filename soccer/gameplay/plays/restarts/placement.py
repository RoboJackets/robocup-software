import behavior
import robocup
import main
import constants
import tactics.line_up
import tactics.our_placement
import standard_play


# one robot places the ball, the others just line up and wait
class Placement(standard_play.StandardPlay):
    def __init__(self):
        super().__init__(continuous=True)

        if self.has_subbehavior_with_name('defense'):
            self.remove_subbehavior('defense')

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        self._pos = main.game_state().get_ball_placement_point()
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

    def execute_running(self):
        #if the ball placement command changes, reset the behaviors accordingly
        if self.check_update():

            #print(self._our_restart!=main.game_state().is_our_restart())
            self._pos = main.game_state().get_ball_placement_point()
            self._our_restart = main.game_state().is_our_restart()

            self.remove_all_subbehaviors()
            if self._our_restart:
                self.placer = tactics.our_placement.OurPlacement()
                self.add_subbehavior(self.placer,
                                     'placer',
                                     required=True,
                                     priority=90)

            line_up = tactics.line_up.LineUp(self.create_lineup())
            self.add_subbehavior(line_up,
                                 'line_up',
                                 required=True,
                                 priority=80)

        main.system_state().draw_circle(self._pos, 0.1, constants.Colors.Green,
                                        "Place")
        main.system_state().draw_circle(self._pos, 0.5, constants.Colors.Red,
                                        "Avoid")

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_our_placement() else float("inf")

    @classmethod
    def is_restart(cls):
        return True
