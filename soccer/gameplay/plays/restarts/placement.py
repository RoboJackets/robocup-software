import standard_play
import behavior
import robocup
import main
import constants
import tactics.line_up
import tactics.our_placement


# one robot places the ball, the others just line up and wait
class Placement(standard_play.StandardPlay):
    def __init__(self):
        super().__init__(continuous=True)

        self.remove_subbehavior('defense')

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.placer.is_done_running(), 'when placer finishes.')

        self.placer = tactics.our_placement.OurPlacement()
        self.add_subbehavior(self.placer, 'placer', required=True, priority=90)

        line = robocup.Segment(robocup.Point(1.5, 1), robocup.Point(1.5, 2.5))
        line_up = tactics.line_up.LineUp(line)

    def execute_running(self):
        main.system_state().draw_circle(
            main.game_state().get_ball_placement_point(), 0.01,
            constants.Colors.Green, "Place")
        main.system_state().draw_circle(
            main.game_state().get_ball_placement_point(), 0.05,
            constants.Colors.Red, "Avoid")

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_our_placement() else float("inf")

    @classmethod
    def is_restart(cls):
        return True

    @classmethod
    def handlesgoalie(cls):
        return True
