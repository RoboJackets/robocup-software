#pragma once

#include "../Behavior.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class ZoneOffense: public Behavior
		{
			public:
				ZoneOffense(GameplayModule *gameplay);
				
				/** note: can use either two or three robots */
				virtual bool assign(std::set<Robot *> &available);

				virtual bool run();

				// loose quadrants
				typedef enum {
					LEFT_FORWARD,
					RIGHT_FORWARD,
					LEFT_HOME,
					RIGHT_HOME
				} Quadrant;

				// splitting field into nine sections
				typedef enum {
					LEFT,
					CENTER,
					RIGHT
				} LRZone;
				typedef enum {
					HOMEFIELD,
					MIDFIELD,
					OPPFIELD
				} SOZone;

			protected:

				Robot *_leftAttack, *_rightAttack, *_midfielder;
		};
	}
}
