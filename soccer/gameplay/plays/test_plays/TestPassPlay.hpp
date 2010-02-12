/*
 * TestPassPlay.hpp
 *
 *  We perform three basic steps to implement a pass:
 *    1.) Generate high-level analytic plan(s)
 *    2.) Optimize high-level plan(s), considering constraints and opponents
 *    3.) Underlying planner - low-level behaviors
 *          Actually executes plan
 *
 *  Currently, this is under development and should not be used.
 *
 *  Created on: Nov 29, 2009
 *      Author: Philip Rogers
 *      Author:
 */

#pragma once

#include <gameplay/Play.hpp>
#include <gameplay/behaviors/Intercept.hpp>
#include <gameplay/behaviors/Kick.hpp>
#include <boost/foreach.hpp>
#include <gameplay/optimizer/PassOptimizer.hpp>
#include <gameplay/optimizer/PassConfig.hpp>
#include <gameplay/optimizer/AnalyticPassPlanner.hpp>

using namespace std;

//typedef boost::ptr_vector<PassConfig> PassConfigVector;

namespace Gameplay{
	namespace Plays{
		class TestPassPlay: public Play{
			public:
				TestPassPlay(GameplayModule *gameplay);

				virtual bool applicable(){return true;}

				virtual void assign(std::set<Robot *> &available);

				virtual bool run();

			protected:
				void initializePlan();

				// passing system state
				enum State{Initializing,Optimizing,Executing,Done};
				State _passState;

				AnalyticPassPlanner::PassConfigVector initialPlans;
				PassConfig bestPassConfig;
				Behaviors::Kick kicker;
				Behaviors::Intercept interceptor;
				int passIndex;
				double playTime;
				bool newPassState;

				/// Pass Planning engine
				AnalyticPassPlanner analyticPlanner_;

				/// Optimization Engine
				Optimization::PassOptimizer optimizer_;

				/// storage of robot pointers
				std::set<Robot *> full_available_;
		};
	}
}
