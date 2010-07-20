#pragma once

#include <gameplay/Behavior.hpp>
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
						double ballsuccess_margin = 0.2,
						bool enableOptimization = true);

				virtual bool assign(std::set<Robot *> &available);

				virtual bool run();

				virtual bool done() { return _passState == Done; }

				// enable/disable optimization
				void enableOptimization(bool opt) { enableOptimization_ = opt; }
				bool enableOptimization() const { return enableOptimization_; }

				// set margins
				inline void setMargins(double time, double robotsuccess, double ballsuccess) {
					time_margin_ = time;
					robotsuccess_margin_ = robotsuccess;
					ballsuccess_margin_ = ballsuccess;
				}

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

				// velocity scaling close to the ball
				float _ballHandlingScale;
				float _ballHandlingRange;

				int _ballControlFrames; // number of frames the ball must be in roller before leaving intercept
				int _ballControlCounter;

				/// Pass Planning engine
				AnalyticPassPlanner analyticPlanner_;

				/// enable flag for optimization
				bool enableOptimization_;

				/// Optimization Engine
				Optimization::PassOptimizer optimizer_;
		};
	}
}
