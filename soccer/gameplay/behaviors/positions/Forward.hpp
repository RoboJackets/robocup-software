#pragma once

#include "../../Behavior.hpp"
#include "../Kick.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Kick;

		class Forward : public Behavior
		{
			public:
				Forward(GameplayModule * gameplay, Role * role);
				~Forward();

				virtual void run();
				virtual void start();
				virtual float score(Robot* robot);

				bool isIntercept();

			protected:
				//Kick/pass behavior
				Gameplay::Behaviors::Kick * _kick;

				//the other forward
				Forward * _teammate;

		};
	}
}
