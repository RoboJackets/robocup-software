import sheen.tactic as tactic
import sheen.tactic.pass_or_shoot as pass_or_shoot
import sheen.tactic.def_support as def_support

import sheen.play as play
import sheen.play.pure_play as pure_play


class Tactics(play.TacticsEnum):
    PASS_OR_SHOOT = play.TacticEntry(pass_or_shoot.PassOrShoot)
    DEF_SUPPORT = play.TacticEntry(def_support.DefSupport)


class OffensiveTriangle(pure_play.PurePlay):
    """ A basic offensive play. The three offensive robots form an attack triangle while
    the other two robots support the ball carrier.
    """

    def __init__(self, tactics_factory: tactic.Factory):
        super().__init__(Tactics(tactics_factory))
