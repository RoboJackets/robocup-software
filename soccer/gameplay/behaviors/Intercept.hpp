#pragma once

#include "../Behavior.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Intercept: public Behavior
		{
			public:
				Intercept(GameplayModule *gameplay, float dist = 0.25);

				typedef enum
				{
					ApproachFar,
					ApproachBall,
					Done
				} State;

				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();

				Geometry2d::Point target;
				
				//set/get
				void farDist(float dist) { _farDist = dist; }
				float farDist() { return _farDist; }
// 				State state() { return _state; }

			protected:
				virtual float score(Robot * robot);

				typedef enum {
					LEFT,
					RIGHT,
					UNSET
				} DriveSide;
				DriveSide _driveSide;

				State _state;
				
				float _farDist; /// Distance threshold between near and far intercept

				int _ballControlFrames; // number of frames the ball must be in roller before leaving intercept
				int _ballControlCounter;

		};
	}
}
