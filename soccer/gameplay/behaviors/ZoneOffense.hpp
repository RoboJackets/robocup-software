#pragma once

#include "../Behavior.hpp"
#include "Kick.hpp"
#include "Mark.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class ZoneOffense: public Behavior
		{
			public:
				ZoneOffense(GameplayModule *gameplay);
				
				virtual ~ZoneOffense();

				/** note: can use either two or three robots */
				virtual bool assign(std::set<Robot *> &available);

				virtual bool run();

				/** zones - track which robot is handling the ball */
				typedef enum {
					LEFT,
					RIGHT,
					MIDFIELD,
					NONE
				} Zone;

				void activeZone(Zone zone) { _activeZone = zone; }
				Zone activeZone() const { return _activeZone; }

			protected:

				Robot *_leftAttack, *_rightAttack, *_midfielder;

				// sub-behaviors to simplify system
				Kick * _kicker;
				Mark *_markLeft, *_markRight, *_markMidfield;

				Zone _activeZone; /// zone which is actively trying to get the ball

				// Real zones - smaller, "NoMid" versions don't include midfielders
				Geometry2d::Rect _midfieldZone,
							     _leftZone, _rightZone,
							     _leftNoMidZone, _rightNoMidZone;

				// enlarged zones - used for hysteresis
				Geometry2d::Rect _midfieldBigZone,
								 _leftBigZone, _rightBigZone,
								 _leftBigNoMidZone, _rightBigNoMidZone;
		};
	}
}
