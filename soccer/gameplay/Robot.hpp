#pragma once

#include <Constants.hpp>
#include <LogFrame.hpp>
#include <MotionCmd.hpp>

#include <stdint.h>
#include <string>
#include <list>

namespace Gameplay
{
	class Opponent;
	class GameplayModule;
	class Behavior;
	
	// This is largely a wrapper around Packet::LogFrame::Robot.
	// It provides convenience functions for setting motion commands and reading state.
	// It also tracks per-robot information that is internal to gameplay which does not need to be logged.
	class Robot
	{
		public:
			Robot(GameplayModule *gameplay, int id, bool self);

			Packet::LogFrame::Robot *packet() const
			{
				return _packet;
			}

			bool visible() const
			{
				return packet()->valid;
			}

			int id() const
			{
				return _id;
			}
			
			const Geometry2d::Point &pos() const
			{
				return packet()->pos;
			}
			
			const Geometry2d::Point &vel() const
			{
				return packet()->vel;
			}
			
			const float &angle() const
			{
				return packet()->angle;
			}
			
			// Move to a particular point using the RRT planner
			void move(Geometry2d::Point pt)
			{
				packet()->cmd.goalPosition = pt;

				// enable the RRT-based planner
				packet()->cmd.planner = Packet::MotionCmd::RRT;
			}
			
			/// Move along a path for waypoint-based control
			void moveExplicit(const std::vector<Geometry2d::Point>& path)
			{
				// set motion command to use the explicit path generation
				packet()->cmd.planner = Packet::MotionCmd::Explicit;

				// clear the path and set it to the correct one
				packet()->cmd.explicitPath.clear();
				packet()->cmd.explicitPath = path;
			}

			/// Move using direct velocity control
			void moveDirectVelocity(const Geometry2d::Point& trans, double ang)
			{
				//NOT IMPLEMENTED!
				packet()->cmd.planner = Packet::MotionCmd::DirectVelocity;
				packet()->cmd.direct_ang_vel = ang;
				packet()->cmd.direct_trans_vel = trans;
			}

			/**
			 * Move using timed-positions, so that each node has a target time
			 * Also, the command needs a start time, so that it can calculate deltas
			 * in seconds
			 */
			void moveTimePos(const std::vector<Packet::MotionCmd::PathNode>& timedPath, uint64_t start) {
				// set controller type
				packet()->cmd.planner = Packet::MotionCmd::TimePosition;

				// set path
				packet()->cmd.timePosPath.clear();
				packet()->cmd.timePosPath = timedPath;

				// set start time
				packet()->cmd.start_time = start;
			}

			void spin(Packet::MotionCmd::SpinType dir)
			{
				packet()->cmd.spin = dir;
			}
			
			bool haveBall() const
			{
				return packet()->haveBall;
			}
			
			void dribble(int8_t speed)
			{
				packet()->radioTx.roller = speed;
			}
			
			void pivot(Geometry2d::Point ctr, Packet::MotionCmd::PivotType dir)
			{
				packet()->cmd.pivotPoint = ctr;
				packet()->cmd.pivot = dir;
			}
			
			void face(Geometry2d::Point pt, bool continuous = false)
			{
				packet()->cmd.goalOrientation = pt;
				packet()->cmd.face = continuous ? Packet::MotionCmd::Endpoint : Packet::MotionCmd::Continuous;
			}
			
			void faceNone()
			{
				packet()->cmd.face = Packet::MotionCmd::None;
			}
			
			void kick(uint8_t strength)
			{
				willKick = true;
				packet()->radioTx.kick = strength;
			}
			
			void pivot(Geometry2d::Point center, bool cw)
			{
				packet()->cmd.pivotPoint = center;
				packet()->cmd.pivot = cw ? Packet::MotionCmd::CW : Packet::MotionCmd::CCW;
			}
			
			bool charged() const
			{
				return packet()->radioRx.charged;
			}
			
			bool self() const
			{
				return _self;
			}
			
			ObstacleGroup &obstacles() const
			{
				return packet()->obstacles;
			}
			
			void resetMotionCommand();
			
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
			Packet::LogFrame::Robot *_packet;
	};
}
