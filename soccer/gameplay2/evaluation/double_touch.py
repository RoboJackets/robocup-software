import evaluation.ball


# This module implements our observance of the double-touch rule
#
# From the official rules (as of 2013):
#   "If, after the ball enters play other than due to a forced restart, the kicker touches the ball a second
#    time (without holding the ball) before it has touched another robot:
#      an indirect free kick is awarded to the opposing team, the kick to be taken from the place
#      where the infringement occurred (see Law 13)"
#


_forbidden_ball_toucher = None


def reset():
    global _forbidden_ball_toucher
    # TODO: reset other tracking state
    _forbidden_ball_toucher = None


# At each iteration, you must call `update()` to do tracking of the rule
def update():
    # for bot in main.our_robots():
    #     if bot.just_kicked():
    #         _forbidden_ball_toucher = bot.shell_id()

    global _forbidden_ball_toucher
    raise NotImplementedError()


# returns the shell id of the robot that is'nt allowed to touch the ball right now
# according to the double-touch rule
# TODO: explain the double-touch rule
def forbiden_ball_toucher():
    global _forbidden_ball_toucher
    return _forbidden_ball_toucher
