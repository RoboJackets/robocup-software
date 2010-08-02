/**
 *  Play to test the PassConfig Optimizer, by generating a PassConfig
 *  and then optimizing it using the system state.
 *
 *  Note that this play does not execute the play, only runs the optimizer
 *  to create a new play.
 */

#pragma once

#include <vector>
#include <gameplay/Play.hpp>
#include <gameplay/optimizer/AnalyticPassPlanner.hpp>
#include <gameplay/optimizer/PassConfig.hpp>
#include <gameplay/optimizer/PassOptimizer.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class DemoPassConfigOptimize: public Play
		{
			public:
				DemoPassConfigOptimize(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();

			protected:
				/** The initial config - created at startup */
				PassConfig config_;

				/** The optimized config - created on optimization step */
				PassConfig opt_config_;

				/** state to handle initialization and execution */
				typedef enum {
					INIT,
					SHOW,
					DONE
				} DemoState;
				DemoState testState_;

				/** initial plan generator */
				AnalyticPassPlanner analyticPlanner_;

				/** optimizer engine */
				Optimization::PassOptimizer optimizer_;

		};
	}
}
