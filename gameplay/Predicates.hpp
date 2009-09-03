#pragma once

#include "Predicate.hpp"

namespace Gameplay
{
    namespace Predicates
    {
        /** Who has control of the ball */
        extern Predicate offense;
        /** Free ball is when neither team has control, and ball speed is low */
        extern Predicate free_ball;

        /** Field position of the ball */
        extern Predicate home_field;
        extern Predicate mid_field;
        extern Predicate opp_field;

        // This is used to add a constant bias to PREFER
        extern Predicate always;

        // Available robots.
        // Each of these is true if we have at least the corresponding number of robots plus a goalie.
        extern Predicate have2, have3, have4;

        /// states ///
        /** 500 mm ball zone */
        extern Predicate stopped;
        /** setup, cannot touch ball, can approach */
        extern Predicate setup;
        /** waiting for opponent to start */
        extern Predicate ready;
        /** free running */
        extern Predicate playing;

        // Setup or ready for any restart
        extern Predicate restart;

        // Restarts
        extern Predicate kickoff;
        extern Predicate penalty;
        extern Predicate direct;
        extern Predicate indirect;
        extern Predicate freekick;

        /** true if we are responsible for the start action */
        extern Predicate our_restart;

        extern Predicate winning;
        extern Predicate losing;
    }
}
