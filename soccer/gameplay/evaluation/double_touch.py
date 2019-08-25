import constants
import evaluation.ball
import fsm
import main
import logging
import enum
import evaluation


## A state machine that tracks the double touch rule
#
# From the official rules (as of 2013):
#   "If, after the ball enters play other than due to a forced restart, the kicker touches the ball a second
#    time (without holding the ball) before it has touched another robot:
#      an indirect free kick is awarded to the opposing team, the kick to be taken from the place
#      where the infringement occurred (see Law 13)"
#
class DoubleTouchTracker(fsm.StateMachine):

    TOUCH_RADIUS = constants.Ball.Radius * 2.5

    class State(enum.Enum):
        start = 1
        restart_play_began = 2
        kicking = 3  # we go to this state once we have determined which of our bots is the kicker
        kicker_forbidden = 4  # the kicker has kicked or fumbled and is no longer allowed to touch it
        other_robot_touched = 5  # after another bot has touched the ball, the double touch rule is no longer applicable

    def __init__(self):
        super().__init__(start_state=DoubleTouchTracker.State.start)

        for state in DoubleTouchTracker.State:
            self.add_state(state)

        # FIXME: is it only restart plays?
        self.add_transition(
            DoubleTouchTracker.State.start,
            DoubleTouchTracker.State.restart_play_began,
            lambda: (main.root_play().play is not None and
                     main.root_play().play.__class__.is_restart() and
                     main.game_state().is_our_restart()),
            'we start running an offensive restart play')

        self.add_transition(
            DoubleTouchTracker.State.restart_play_began,
            DoubleTouchTracker.State.kicking,
            lambda: ((any(self._touching_ball(bot) for bot in main.our_robots())) or
                     main.game_state().is_playing()) and not main.game_state().is_placement(),
            'one of our bots has the ball or the ball was kicked')

        self.add_transition(DoubleTouchTracker.State.kicking,
                            DoubleTouchTracker.State.kicker_forbidden,
                            # The ball is no longer in restart, we have begun playing
                            (lambda: main.game_state().is_playing() or
                             # We aren't in a restart anymore
                             main.root_play().play is None or
                             not main.root_play().play.__class__.is_restart()),
                            'ball has moved and is now in play')

        self.add_transition(DoubleTouchTracker.State.kicker_forbidden,
                            DoubleTouchTracker.State.other_robot_touched,
                            lambda: self.other_robot_touching_ball(),
                            'another robot has touched the ball')

    def _touching_ball(self, bot):
        """A function for combining the ball sense and our own home
        grown cookies and ice cream."""
        return bot.has_ball() or self._bot_in_radius(bot)

    ## The shell id of the robot that isn't allowed to touch the ball
    # returns None if everyone is allowed to touch
    def forbidden_ball_toucher(self):
        return (self.kicker_shell_id
                if self.state == DoubleTouchTracker.State.kicker_forbidden
                else None)

    # returns True if a bot other than the kicker is touching the ball
    def other_robot_touching_ball(self):
        for bot in filter(lambda bot: bot.visible,
                          list(main.our_robots()) + list(main.their_robots())):
            if (bot.is_ours() and
                bot.has_ball() and
                    bot.shell_id() != self.kicker_shell_id):
                return True
            if ((bot.shell_id() != self.kicker_shell_id or
                 not bot.is_ours()) and
                    self._bot_in_radius(bot)):
                return True
        return False

    def _bot_in_radius(self, bot):
        "Check if a bot is within a touch radius"
        max_radius = constants.Robot.Radius + DoubleTouchTracker.TOUCH_RADIUS
        return (bot.visible and
                main.ball().valid and
                bot.pos.near_point(main.ball().pos, max_radius))

    # reset
    def on_enter_start(self):
        self.kicker_shell_id = None
        self.pre_ball_pos = None

    def on_enter_kicking(self):
        if main.ball().valid:
            self.pre_ball_pos = main.ball().pos

    # when we've identified that one of our bots has the ball,
    # record it's shell id
    def on_exit_restart_play_began(self):
        for bot in main.our_robots():
            if self._touching_ball(bot):
                self.kicker_shell_id = bot.shell_id()
                return
        if main.ball().valid:
            # pick our closest robot
            ball_pos = self.pre_ball_pos
            if not ball_pos:
                ball_pos = main.ball().pos
            bot = min(main.our_robots(),
                      key=lambda bot: bot.pos.dist_to(ball_pos))
            self.kicker_shell_id = bot.shell_id()

    def on_enter_kicker_forbidden(self):
        logging.info("Due to DoubleTouch rule, robot '" + str(
            self.kicker_shell_id) + "' can't touch the ball")

    def execute_kicker_forbidden(self):
        bot = None
        for b in main.our_robots():
            if b.shell_id() == self.kicker_shell_id:
                bot = b
                break
        if self.kicker_shell_id and bot:
            main.debug_drawer().draw_text("Blocking double touch!", bot.pos,
                                          constants.Colors.Red,
                                          "Double Touches")

# global double touch tracker
_tracker = DoubleTouchTracker()


def tracker():
    global _tracker
    return _tracker
