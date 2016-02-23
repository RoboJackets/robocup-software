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
            lambda: (main.root_play().play != None and main.root_play().play.__class__.is_restart() and main.game_state().is_our_restart()),
            'we start running an offensive restart play')

        self.add_transition(
            DoubleTouchTracker.State.restart_play_began,
            DoubleTouchTracker.State.kicking,
            lambda: any(bot.has_ball() for bot in main.our_robots()),
            'one of our bots has the ball')

        self.add_transition(DoubleTouchTracker.State.kicking,
                            DoubleTouchTracker.State.kicker_forbidden,
                            lambda: not self.kicker_has_possession(),
                            'kicker kicks or fumbles ball')

        self.add_transition(DoubleTouchTracker.State.kicker_forbidden,
                            DoubleTouchTracker.State.other_robot_touched,
                            lambda: self.other_robot_touching_ball(),
                            'another robot has touched the ball')

    def kicker_has_possession(self):
        if self.kicker_shell_id != None:
            for bot in main.our_robots():
                if bot.shell_id() == self.kicker_shell_id:
                    # we use two methods here because the ball-sensor output is often jittery
                    return bot.has_ball() or evaluation.ball.robot_has_ball(
                        bot)
        return False

    ## The shell id of the robot that isn't allowed to touch the ball
    # returns None if everyone is allowed to touch
    def forbidden_ball_toucher(self):
        return self.kicker_shell_id if self.state == DoubleTouchTracker.State.kicker_forbidden else None

    # returns True if a bot other than the kicker is touching the ball
    def other_robot_touching_ball(self):
        max_radius = constants.Robot.Radius + constants.Ball.Radius + 0.03
        for bot in list(main.our_robots()) + list(main.their_robots()):
            if bot.visible and (not bot.is_ours() or
                                not bot.shell_id() == self.kicker_shell_id):
                if bot.pos.near_point(main.ball().pos, max_radius):
                    return True

        return False

    # reset
    def on_enter_start(self):
        self.kicker_shell_id = None

    # when we've identified that one of our bots has the ball,
    # record it's shell id
    def on_exit_restart_play_began(self):
        for bot in main.our_robots():
            if bot.has_ball():
                self.kicker_shell_id = bot.shell_id()
                return

    def on_enter_kicker_forbidden(self):
        logging.info("Due to DoubleTouch rule, robot '" + str(
            self.kicker_shell_id) + "' can't touch the ball")

# global double touch tracker
_tracker = DoubleTouchTracker()


def tracker():
    global _tracker
    return _tracker
