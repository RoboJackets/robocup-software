"""This module contains all the plays used by the decision tree situation analyzer.
Ported from the old situation alaysis."""

from stp.situation import ISituation


class NoSituation(ISituation):
    """No Situation."""

    ...


class PrepareKickoff(ISituation):
    """Setup for kickoff."""

    ...


class Kickoff(ISituation):
    """Kickoff."""

    ...


class DefendKickoff(ISituation):
    """Kickoff defense."""

    ...


class DefendRestartOffensive(ISituation):
    """Defending our opponents restart on their side of the field."""

    ...


class DefendRestartMidfield(ISituation):
    """Plays for defending our opponents restart on their side of the field."""

    ...


class DefendRestartDefensive(ISituation):
    """Plays for defending our opponents restart in the midfield."""

    ...


class DefendRestartOffensiveDirect(ISituation):
    """Defending our direct opponents restart on their side of the field."""

    ...


class DefendRestartMidfieldDirect(ISituation):
    """Plays for defending our opponents direct restart on their side of the field."""

    ...


class DefendRestartDefensiveDirect(ISituation):
    """Plays for defending our opponents direct restart in the midfield."""

    ...


class Clear(ISituation):
    """Plays for clearing the ball from our side of the field (should include defensive
    caution)."""

    ...


class DefendClear(ISituation):
    """Plays for defending the opponents clear, when the ball is on their side."""

    ...


class DefendGoal(ISituation):
    """Plays for defending our goal from opponents near it with the ball."""

    ...


class MidfieldClear(ISituation):
    """Plays for when we possess the ball in the midfield."""

    ...


class AttackGoal(ISituation):
    """Plays for attacking the opponents goal, when we have the ball near it."""

    ...


class OffensiveScramble(ISituation):
    """Plays for getting a loose ball when the ball is on the opponents half."""

    ...


class MidfieldScramble(ISituation):
    """Plays for getting a loose ball when the ball is on the opponents half."""

    ...


class DefensiveScramble(ISituation):
    """Plays for getting a loose ball when the ball is on our half."""

    ...


class SaveBall(ISituation):
    """Plays that will trigger when the ball is headed out of the field with no
    obstructions."""

    ...


class SaveShot(ISituation):
    """Plays that will trigger when the ball is headed directly at our goal."""

    ...


class OffensivePileup(ISituation):
    """Plays to handle a pile up on their side of the field."""

    ...


class MidfieldPileup(ISituation):
    """Plays to handle a pile up in the midfield."""

    ...


class DefensivePileup(ISituation):
    """Plays to handle a pile up on our side of the field."""

    ...


class MidfieldDefendClear(ISituation):
    """Plays to defend a clear when the ball is in the midfield."""

    ...


class PrepareShootout(ISituation):
    """Plays for making shootout shots."""

    ...


class Shootout(ISituation):
    """Plays for making shootout shots."""

    ...


class PrepareDefendShootout(ISituation):
    """Plays for defending shootout shots."""

    ...


class DefendShootout(ISituation):
    """Plays for defending shootout shots."""

    ...


class PreparePenalty(ISituation):
    """Set up for penalty shots."""

    ...


class Penalty(ISituation):
    """Plays for making penalty shots."""

    ...


class PrepareDefendPenalty(ISituation):
    """Prepare to defend the penalty. Our goalie must be on the goal box line."""

    ...


class DefendPenalty(ISituation):
    """Plays for defending penalty shots."""

    ...


class OffensiveKick(ISituation):
    """Plays for indirect kicks on their side."""

    ...


class DefensiveKick(ISituation):
    """Plays for indirect kicks on our side."""

    ...


class MidfieldKick(ISituation):
    """Plays for indirect kicks in the midfield."""

    ...


class OffensiveKickDirect(ISituation):
    """Plays for direct kicks on their side."""

    ...


class DefensiveKickDirect(ISituation):
    """Plays for direct kicks on our side."""

    ...


class MidfieldKickDirect(ISituation):
    """Plays for direct kicks in the midfield."""

    ...


class GoalieClear(ISituation):
    """Plays for clearing the ball when our goalie possesses the ball."""

    ...


class Stop(ISituation):
    """Plays for dealing with the stop state."""

    ...
