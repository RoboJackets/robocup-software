#pragma once

#include "../Behavior.hpp"

#include "Intercept.hpp"
#include <iostream>

namespace Gameplay
{
	class Window;
	class WindowEvaluator;

	namespace Behaviors
	{
		/**
		 * Kick allows for general ball firing action
		 *  - Direct kicking
		 *  - Chip Kicking
		 *
		 * Can do either full power shots or reduced power for passes
		 */
		class Kick: public Behavior
		{
			public:

				//TODO: include others, like yank, bunting, etc.
				typedef enum {
					KICK,
					CHIP
				} KickType;

				typedef enum {
					PIVOT,
					ONETOUCH
				} AimType;

				typedef enum {
					ROBOT,
					GOAL,
					SEGMENT
				} TargetType;

				Kick(GameplayModule *gameplay);
				~Kick();

				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();

				bool isIntercept()
				{
					return _state == Intercept;
				}
				bool isDone()
				{
					return _state == Done;
				}

				bool automatic;

				enum State
				{
					Intercept,
					Aim,
					Shoot,
					OneTouchAim,
					Done
				};
				State getState() const { return _state; }

				/** returns if successful due to check for chipper */
				bool kickType(KickType mode);
				KickType kickType() const { return _kickType; }

				void aimType(AimType mode) { _aimType = mode; }
				AimType aimType() const { return _aimType; }

				void targetType(TargetType mode) { _targetType = mode; }
				TargetType targetType() const { return _targetType; }

				/// set target functions
				void setTarget(); /// sets to goal
				void setTarget(const Geometry2d::Segment& seg); /// shoot to arbitrary segment (clearing)
				void setTarget(Robot * r); /// pass to robot

				/** Restarts the kick play - keep going after ball */
				void restart() {_state = Intercept;}

				/**
				 * velocity bounding close to the ball
				 * Set scale to 1.0 for no thresholding
				 */
				void setVScale(float scale, float range) {
					_ballHandlingScale = scale;
					_ballHandlingRange = range;
				}

			protected:
				virtual float score(Robot* robot);

				WindowEvaluator *_win;
				Gameplay::Behaviors::Intercept* _intercept;

				State _state;
				float _lastMargin;
				Robot * _targetRobot;
				Geometry2d::Segment _target;

				typedef enum {
					LEFT,
					RIGHT,
					UNSET
				} DriveSide;

				AimType _aimType;
				KickType _kickType;
				TargetType _targetType;
				DriveSide _driveSide;

				//we lock in a pivot point
				Geometry2d::Point _pivot;

				// The robot's position when it entered the Shoot state
				Geometry2d::Point _shootStart;

				Geometry2d::Point _shootMove;
				Geometry2d::Point _shootBallStart;

				// velocity scaling close to the ball
				float _ballHandlingScale;
				float _ballHandlingRange;

				// control values for one touch bezier movement
				std::vector<Geometry2d::Point> _controls;

				// Kick evaluation via obstacles
				Geometry2d::Segment evaluatePass(); /// finds a pass segment
				Geometry2d::Segment evaluateShot(); /// finds a shot segment
				Geometry2d::Segment evaluateSegment();  /// finds a clear segment on an arbitrary segment

				// determining how hard to kick
				int calcKickStrength(const Geometry2d::Point& targetCenter);

				// checks whether other robots intersect the shot properly
				bool checkRobotIntersections(const Geometry2d::Segment& shotLine);

				// Individual state functions - these should execute a state's
				// activities, and return what the next state should be
				State intercept(const Geometry2d::Point& targetCenter);
				State aim(const Geometry2d::Point& targetCenter, bool canKick);
				State shoot(const Geometry2d::Point& targetCenter, int kickStrength);

				// additional state for one-touch aiming
				State oneTouchApproach();
		};
	}
}
