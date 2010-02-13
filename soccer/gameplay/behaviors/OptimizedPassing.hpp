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
				OptimizedPassing(GameplayModule *gameplay);

				virtual void assign(std::set<Robot *> &available);

				virtual bool run();

				virtual bool done() { return _passState == Done; }

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
		};
	}
}
