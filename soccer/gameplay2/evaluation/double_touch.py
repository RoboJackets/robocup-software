import evaluation.ball


# This module implements our observance of the double-touch rule
# At each iteration, you must call `update()` to do tracking of the rul


_forbidden_ball_toucher = None


def reset():
    global _forbidden_ball_toucher
    _forbidden_ball_toucher = None


def update():
    global _forbidden_ball_toucher
    raise NotImplementedError()


# returns the shell id of the robot that is'nt allowed to touch the ball right now
# according to the double-touch rule
# TODO: explain the double-touch rule
def forbiden_ball_toucher():
    global _forbidden_ball_toucher
    return _forbidden_ball_toucher
