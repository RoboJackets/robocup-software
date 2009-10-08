#pragma once

#include <Constants.hpp>
#include <LogFrame.hpp>

#include <stdint.h>
#include <string>
#include <list>

namespace Gameplay
{
	class Role;
	class Opponent;
	class GameplayModule;
	class Behavior;
	
	// Information about one robot on our team.
	class Robot
	{
		public:
			Robot(GameplayModule *gameplay, int id, bool self);

			Packet::LogFrame::Robot *state() const;

			bool visible() const;
			bool assigned() const;

			// If this is one of our robots, this is the role it is assigned (if any).
			Role *role() const
			{
				return _role;
			}
			
			void role(Role *role);

			// If this is an opponent robot, this is the opponent identity it is assigned.
			Opponent *opponent() const
			{
				return _opponent;
			}
			
			void opponent(Opponent *opp);

			Behavior *behavior() const;

			bool goalie() const
			{
				return _goalie;
			}
			
			void goalie(bool flag);

			int id() const
			{
				return _id;
			}
			
			const std::string &name() const
			{
				return _name;
			}
			void name(const std::string &name)
			{
				_name = name;
			}
			
			void free(bool f)
			{
				_free = f;
			}
			bool free() const
			{
				return _free;
			}
			
			const Geometry2d::Point &pos() const
			{
				return state()->pos;
			}
			
			const Geometry2d::Point &vel() const
			{
				return state()->vel;
			}
			
			const float &angle() const
			{
				return state()->angle;
			}
			
			void move(Geometry2d::Point pt)
			{
				state()->cmd.goalPosition = pt;
			}
			void spin(Packet::LogFrame::MotionCmd::SpinType dir)
			{
				state()->cmd.spin = dir;
			}
			
			bool haveBall() const
			{
				return state()->haveBall;
			}
			
			void dribble(int8_t speed)
			{
				state()->radioTx.roller = speed;
			}
			
			void pivot(Geometry2d::Point ctr,
			        Packet::LogFrame::MotionCmd::PivotType dir)
			{
				state()->cmd.pivotPoint = ctr;
				state()->cmd.pivot = dir;
			}
			
			void face(Geometry2d::Point pt, bool continuous = false)
			{
				state()->cmd.goalOrientation = pt;
				state()->cmd.face
				        = continuous ? Packet::LogFrame::MotionCmd::Endpoint
				                : Packet::LogFrame::MotionCmd::Continuous;
			}
			
			void kick(uint8_t strength)
			{
				willKick = true;
				state()->radioTx.kick = strength;
			}
			
			void pivot(Geometry2d::Point center, bool cw)
			{
				state()->cmd.pivotPoint = center;
				state()->cmd.pivot = cw ? Packet::LogFrame::MotionCmd::CW
				        : Packet::LogFrame::MotionCmd::CCW;
			}
			
			bool self() const
			{
				return _self;
			}
						
			// True if this robot intends to kick the ball.
			// This is reset when this robot's role changes.
			// This allows the robot to get close to the ball during a restart.
			bool willKick;

			// True if this robot should avoid the ball by 500mm.
			// Used during our restart for robots that aren't going to kick
			// (not strictly necessary).
			bool avoidBall;

			// True if this robot intends to get close to an opponent
			// (e.g. for stealing).
			// This reduces the size of the opponent's obstacle.
			// These are reset when this robot's role changes.
			bool approachOpponent[Constants::Robots_Per_Team];

		protected:
			GameplayModule *_gameplay;

			int _id;
			bool _self;
			std::string _name;

			Role *_role;
			Opponent *_opponent;
			bool _goalie;

			/** if true, this robot is not controlled by another robot/behavior */
			bool _free;
	};
}
