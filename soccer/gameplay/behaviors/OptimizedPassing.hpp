#pragma once

#include "../Behavior.hpp"
#include <gameplay/behaviors/Intercept.hpp>
#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/optimizer/PassOptimizer.hpp>
#include <gameplay/optimizer/PassConfig.hpp>
#include <gameplay/optimizer/AnalyticPassPlanner.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		class OptimizedPassing: public Behavior
		{

			public:
				OptimizedPassing(GameplayModule *gameplay,
						double time_margin = 5.5,
						double robotsuccess_margin = 0.05,
						double ballsuccess_margin = 0.2);

				virtual void assign(std::set<Robot *> &available);

				virtual bool run();

				virtual bool done() { return _passState == Done; }

			protected:
				bool initializePlan();

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

				/// margins
				double time_margin_;         /// seconds a play can deviate from plan before abort
				double robotsuccess_margin_; /// if robot within this region, a move is complete
				double ballsuccess_margin_;  /// if robot within this region, a move is complete

				/// Pass Planning engine
				AnalyticPassPlanner analyticPlanner_;

				/// Optimization Engine
				Optimization::PassOptimizer optimizer_;
		};
	}
}
