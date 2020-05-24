from enum import Enum


class Situation(Enum):
    NONE = 0  # This situation should never be encountered during gameplay
    KICKOFF = 1  # Plays that can perform our kickoff
    DEFEND_RESTART_OFFENSIVE = 2  # Plays for defending our opponents restart on their side of the field
    DEFEND_RESTART_MIDFIELD = 3  # Plays for defending our opponents restart in the midfield
    DEFEND_RESTART_DEFENSIVE = 4  # Plays for defending our opponents restart on our side of the field
    CLEAR = 5  # play for clearing the ball from our side of the field (should include defensive caution)
    DEFEND_CLEAR = 6  # Plays for defending the opponents clear, when the ball is on their side.
    DEFEND_GOAL = 7  # Plays for defending our goal from opponents near it with the ball
    MIDFIELD_CLEAR = 8  # Plays for when we possess the ball in the midfield
    ATTACK_GOAL = 9  # Plays for attacking the opponents goal, when we have the ball near it
    OFFENSIVE_SCRAMBLE = 10  # Plays for getting a loose ball when the ball is on the opponents half
    MIDFIELD_SCRAMBLE = 11  # Plays for getting a loose ball when the ball is in the midfield
    DEFENSIVE_SCRAMBLE = 12  # Plays for getting a loose ball when the ball is on our half
    SAVE_BALL = 13  # Plays that will trigger when the ball is headed out of the field with no obstuctions
    SAVE_SHOT = 14  # Plays that will trigger when the ball is headed directly at our goal
    OFFENSIVE_PILEUP = 15  # Plays to handle a pile up on their side of the field
    MIDFIELD_PILEUP = 16  # Plays to handle a pile up in the midfield
    DEFENSIVE_PILEUP = 17  # Plays to handle a pile up on our side of the field
    MIDFIELD_DEFEND_CLEAR = 18  # Plays to defend a clear when the ball is in the midfield
    SHOOTOUT = 19  # Plays for making shootout shots
    DEFEND_SHOOTOUT = 20  # Plays for defending shootout shots
    PENALTY = 21  # Plays for making penalty shots
    DEFEND_PENALTY = 22  # Plays for defending penalty shots
    OFFENSIVE_KICK = 23  # Plays for direct and indirect kicks on their side
    DEFENSIVE_KICK = 24  # Plays for direct and indirect kicks on our side
    MIDFIELD_KICK = 25  # Plays for direct and indirect kicks in the midfield
    GOALIE_CLEAR = 26  # Plays for clearing the ball when our goalie possesses the ball
