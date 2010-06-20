#pragma once

#include "../Behavior.hpp"
#include "../Window.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class GoalDefender : public Behavior
		{
			public:
				GoalDefender(GameplayModule *gameplay);

				virtual bool run();
				virtual bool done();
				virtual bool assign(std::set<Robot *> &available);

				virtual float score(Robot* robot);

			protected:
				//Window evaluator for checking passes/shots
				Gameplay::WindowEvaluator* _winEval;
		};
	}
}
