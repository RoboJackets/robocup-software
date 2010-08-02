/**
 * @file OptimizedOffense.hpp
 * @brief The base play for the optimization-based offensive passing play
 * @author Alex Cunningham
 */

#pragma once

#include "../Play.hpp"

namespace Gameplay
{
	namespace Plays
	{
		/**
		 * This play is a top-level play for handling offense with
		 * optimization-based play design.  It's basic goal is to handle the
		 * high-level decision-making as to what offensive play to execute, and
		 * then parameterize and execute lower level plays.
		 *
		 * Modes of play:
		 *  1. One robot makes direct shot
		 *  2. Pass, direct shot (two robots)
		 *  3. Pass, Pass, direct shot (three robots)
		 *  4. Cyclical passing if no shot is available after a pass-pass-shot
		 *
		 *  All optmized plays have a basic structure:
		 *  1. Determine that the play is viable
		 *  2. Initialize a first guess at the plan
		 *  3. Run an iterative optimization routine until plan is good, or out of time
		 *  4. Store plan for execution
		 *
		 * The resulting plans are all spline-based robot trajectories, with timing
		 * for ball kicks.
		 */
		class OptimizedOffense: public Play
		{
			public:
				OptimizedOffense(GameplayModule *gameplay);

				/**
				 * Runs whenever the game is running and the
				 * opponents do not have the ball
				 */
				virtual bool applicable(const std::set<Robot *> &robots);

				/** Called every frame */
				virtual bool run();

			protected:



				bool planReady_; /// true if there is a calculated plan
		};
	}
}

