#pragma once

#include <string>

namespace RefereeModuleEnums {
// These are the "coarse" stages of the game.
enum Stage {
    // The first half is about to start.
    // A kickoff is called within this stage.
    // This stage ends with the NORMAL_START.
    NORMAL_FIRST_HALF_PRE = 0,
    // The first half of the normal game, before half time.
    NORMAL_FIRST_HALF = 1,
    // Half time between first and second halves.
    NORMAL_HALF_TIME = 2,
    // The second half is about to start.
    // A kickoff is called within this stage.
    // This stage ends with the NORMAL_START.
    NORMAL_SECOND_HALF_PRE = 3,
    // The second half of the normal game, after half time.
    NORMAL_SECOND_HALF = 4,
    // The break before extra time.
    EXTRA_TIME_BREAK = 5,
    // The first half of extra time is about to start.
    // A kickoff is called within this stage.
    // This stage ends with the NORMAL_START.
    EXTRA_FIRST_HALF_PRE = 6,
    // The first half of extra time.
    EXTRA_FIRST_HALF = 7,
    // Half time between first and second extra halves.
    EXTRA_HALF_TIME = 8,
    // The second half of extra time is about to start.
    // A kickoff is called within this stage.
    // This stage ends with the NORMAL_START.
    EXTRA_SECOND_HALF_PRE = 9,
    // The second half of extra time.
    EXTRA_SECOND_HALF = 10,
    // The break before penalty shootout.
    PENALTY_SHOOTOUT_BREAK = 11,
    // The penalty shootout.
    PENALTY_SHOOTOUT = 12,
    // The game is over.
    POST_GAME = 13
};

std::string stringFromStage(Stage s);

// These are the "fine" states of play on the field.
enum Command {
    // All robots should completely stop moving.
    HALT = 0,
    // Robots must keep 50 cm from the ball.
    STOP = 1,
    // A prepared kickoff or penalty may now be taken.
    NORMAL_START = 2,
    // The ball is dropped and free for either team.
    FORCE_START = 3,
    // The yellow team may move into kickoff position.
    PREPARE_KICKOFF_YELLOW = 4,
    // The blue team may move into kickoff position.
    PREPARE_KICKOFF_BLUE = 5,
    // The yellow team may move into penalty position.
    PREPARE_PENALTY_YELLOW = 6,
    // The blue team may move into penalty position.
    PREPARE_PENALTY_BLUE = 7,
    // The yellow team may take a direct free kick.
    DIRECT_FREE_YELLOW = 8,
    // The blue team may take a direct free kick.
    DIRECT_FREE_BLUE = 9,
    // The yellow team may take an indirect free kick.
    INDIRECT_FREE_YELLOW = 10,
    // The blue team may take an indirect free kick.
    INDIRECT_FREE_BLUE = 11,
    // The yellow team is currently in a timeout.
    TIMEOUT_YELLOW = 12,
    // The blue team is currently in a timeout.
    TIMEOUT_BLUE = 13,
    // The yellow team just scored a goal.
    // For information only.
    // For rules compliance, teams must treat as STOP.
    GOAL_YELLOW = 14,
    // The blue team just scored a goal.
    GOAL_BLUE = 15,
    // Equivalent to STOP, but the yellow team must pick up the ball and
    // drop it in the Designated Position.
    BALL_PLACEMENT_YELLOW = 16,
    // Equivalent to STOP, but the blue team must pick up the ball and drop
    // it in the Designated Position.
    BALL_PLACEMENT_BLUE = 17
};

std::string stringFromCommand(Command c);
}  // namespace RefereeModuleEnums
