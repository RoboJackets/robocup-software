import stp.play as play
import stp.play.pure_play as pure_play
import stp.tactic as tactic

import rj_gameplay.tactic.def_support as def_support
import rj_gameplay.tactic.pass_or_shoot as pass_or_shoot


class Tactics(play.TacticsEnum):
    PASS_OR_SHOOT = play.TacticEntry(pass_or_shoot.PassOrShoot)
    DEF_SUPPORT = play.TacticEntry(def_support.DefSupport)


class OffensiveTriangle(pure_play.PurePlay):
    """A basic offensive play. The three offensive robots form an attack triangle while
    the other two robots support the ball carrier.
    """

    def __init__(self, tactics_factory: tactic.Factory):
        super().__init__(Tactics(tactics_factory), ctx)
