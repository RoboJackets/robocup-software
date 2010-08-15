/*
 * DemoPassPlay.hpp
 *
 *  We perform three basic steps to implement a pass:
 *    1.) Generate high-level analytic plan(s)
 *    2.) Optimize high-level plan(s), considering constraints and opponents
 *    3.) Underlying planner - low-level behaviors
 *          Actually executes plan
 *
 *  This is currently just a skeleton play that drives the underlying OptmizedPassing
 *  behavior.
 *
 *  Created on: Nov 29, 2009
 *      Author: Philip Rogers
 *      Author:
 */

#pragma once

#include <gameplay/Play.hpp>
#include <gameplay/behaviors/OptimizedPassing.hpp>

using namespace std;

namespace Gameplay{
	namespace Plays{
		class DemoPassPlay: public Play{
			public:
				DemoPassPlay(GameplayModule *gameplay);

				virtual bool run();

			protected:
				Behaviors::OptimizedPassing passPlanner_;
		};
	}
}
