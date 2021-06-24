"""This module contains all the plays used by the decision tree situation analyzer.
Ported from the old situation alaysis."""

import stp.situation


class NoSituation(stp.situation.ISituation):
    """No Situation."""

    ...


class PrepareKickoff(stp.situation.ISituation):
    """Setup for kickoff."""

    ...


class Kickoff(stp.situation.ISituation):
    """Kickoff."""

    ...


class DefendKickoff(stp.situation.ISituation):
    """Kickoff defense."""

    ...


class DefendRestartOffensive(stp.situation.ISituation):
    """Defending our opponents restart on their side of the field."""

    ...


class DefendRestartMidfield(stp.situation.ISituation):
    """Plays for defending our opponents restart on their side of the field."""

    ...


class DefendRestartDefensive(stp.situation.ISituation):
    """Plays for defending our opponents restart in the midfield."""

    ...


class DefendRestartOffensiveDirect(stp.situation.ISituation):
    """Defending our direct opponents restart on their side of the field."""

    ...


class DefendRestartMidfieldDirect(stp.situation.ISituation):
    """Plays for defending our opponents direct restart on their side of the field."""

    ...


class DefendRestartDefensiveDirect(stp.situation.ISituation):
    """Plays for defending our opponents direct restart in the midfield."""

    ...


class Clear(stp.situation.ISituation):
    """Plays for clearing the ball from our side of the field (should include defensive
    caution)."""

    ...


class DefendClear(stp.situation.ISituation):
    """Plays for defending the opponents clear, when the ball is on their side."""

    ...


class DefendGoal(stp.situation.ISituation):
    """Plays for defending our goal from opponents near it with the ball."""

    ...


class MidfieldClear(stp.situation.ISituation):
    """Plays for when we possess the ball in the midfield."""

    ...


class AttackGoal(stp.situation.ISituation):
    """Plays for attacking the opponents goal, when we have the ball near it."""

    ...


class OffensiveScramble(stp.situation.ISituation):
    """Plays for getting a loose ball when the ball is on the opponents half."""

    ...


class MidfieldScramble(stp.situation.ISituation):
    """Plays for getting a loose ball when the ball is on the opponents half."""

    ...


class DefensiveScramble(stp.situation.ISituation):
    """Plays for getting a loose ball when the ball is on our half."""

    ...


class SaveBall(stp.situation.ISituation):
    """Plays that will trigger when the ball is headed out of the field with no
    obstructions."""

    ...


class SaveShot(stp.situation.ISituation):
    """Plays that will trigger when the ball is headed directly at our goal."""

    ...


class OffensivePileup(stp.situation.ISituation):
    """Plays to handle a pile up on their side of the field."""

    ...


class MidfieldPileup(stp.situation.ISituation):
    """Plays to handle a pile up in the midfield."""

    ...


class DefensivePileup(stp.situation.ISituation):
    """Plays to handle a pile up on our side of the field."""

    ...


class MidfieldDefendClear(stp.situation.ISituation):
    """Plays to defend a clear when the ball is in the midfield."""

    ...


class PrepareShootout(stp.situation.ISituation):
    """Plays for making shootout shots."""

    ...


class Shootout(stp.situation.ISituation):
    """Plays for making shootout shots."""

    ...


class DefendShootout(stp.situation.ISituation):
    """Plays for defending shootout shots."""

    ...


class PreparePenalty(stp.situation.ISituation):
    """Set up for penalty shots."""

    ...


class Penalty(stp.situation.ISituation):
    """Plays for making penalty shots."""

    ...


class PrepareDefendPenalty(stp.situation.ISituation):
    """Prepare to defend the penalty. Our goalie must be on the goal box line."""

    ...


class DefendPenalty(stp.situation.ISituation):
    """Plays for defending penalty shots."""

    ...


class OffensiveKick(stp.situation.ISituation):
    """Plays for indirect kicks on their side."""

    ...


class DefensiveKick(stp.situation.ISituation):
    """Plays for indirect kicks on our side."""

    ...


class MidfieldKick(stp.situation.ISituation):
    """Plays for indirect kicks in the midfield."""

    ...


class OffensiveKickDirect(stp.situation.ISituation):
    """Plays for direct kicks on their side."""

    ...


class DefensiveKickDirect(stp.situation.ISituation):
    """Plays for direct kicks on our side."""

    ...


class MidfieldKickDirect(stp.situation.ISituation):
    """Plays for direct kicks in the midfield."""

    ...


class GoalieClear(stp.situation.ISituation):
    """Plays for clearing the ball when our goalie possesses the ball."""

    ...


class Stop(stp.situation.ISituation):
    """Plays for dealing with the stop state."""

    ...
