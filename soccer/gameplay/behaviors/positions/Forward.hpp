#pragma once

#include "../../Behavior.hpp"

#include <gameplay/behaviors/Kick.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		class Forward : public Behavior
		{
			public:
				Forward(GameplayModule * gameplay);

				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();

				bool isIntercept();

				//the other forward
				Forward *teammate;
				


			protected:
				virtual float score(Robot* robot);
				
				//Kick/pass behavior
				Gameplay::Behaviors::Kick _kick;
		};
	}
}
