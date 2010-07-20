#pragma once

#include <Constants.hpp>
#include <framework/SystemState.hpp>

#include <stdint.h>
#include <string>
#include <list>
#include <vector>

namespace Gameplay
{
	class Opponent;
	class GameplayModule;
	class Behavior;

	// This is largely a wrapper around SystemState::Robot.
	// It provides convenience functions for setting motion commands and reading state.
	// It also tracks per-robot information that is internal to gameplay which does not need to be logged.
	class Robot
	{
		public:
			Robot(GameplayModule *gameplay, int id, bool self);

			SystemState::Robot *packet() const;

			// Status indicators
			bool self() const;    /// true if this is one of our robots
			bool visible() const; /// true if robot is valid - FIXME: needs better check
			int id() const;       /// index in the array - NOT THE SHELL!
			bool haveBall() const; /// true if we have the ball
			SystemState::Robot::Rev rev() const; /// the revision for the robot, use for capability checks
			bool hasChipper() const; /// true if robot can chip

			// kicker readiness checks
			bool charged() const; /// true if the kicker is ready
			float kickTimer() const; /// returns the time since the kicker was last charged, 0.0 if ready

			/**
			 * state variable updates - includes timers, etc.  Called each frame
			 */
			void update();

			// Geometry helper functions
			const Geometry2d::Point &pos() const;  /// Position
			const Geometry2d::Point &vel() const;  /// Velocity (vector)
			const float &angle() const;	  /// global orientation of the robot (radians)
			const Geometry2d::Segment kickerBar() const; /// segment for the location of the kicker
			Geometry2d::Point pointInRobotSpace(const Geometry2d::Point& pt) const; /// converts a point to the frame of reference of robot

			// simple checks to do geometry
			/** returns true if the position specified is behind the robot */
			bool behindBall(const Geometry2d::Point& ballPos) const;

			// Commands
			void setVScale(float scale = 1.0); /// scales the velocity
			void setWScale(float scale = 0.5); /// scales the angular velocity
			void resetMotionCommand();  /// resets all motion commands for the robot

			// Move to a particular point using the RRT planner
			void move(Geometry2d::Point pt, bool stopAtEnd=false);

			/**
			 * Move along a path for waypoint-based control
			 * If not set to stop at end, the planner will send the robot
			 * traveling in whatever direction it was moving in at the end of the path.
			 * This should only be used when there will be another command when
			 * the robot reaches the end of the path.
			 */
			void move(const std::vector<Geometry2d::Point>& path, bool stopAtEnd=true);

			/**
			 * Move via a bezier curve, designed to allow for faster movement
			 * The points specified are bezier control points, which define the
			 * path taken.  Note: longer paths are more computationally expensive.
			 *
			 * To enable control point modification to allow for avoidance of obstacles,
			 * set the enableAvoid flag to true, false otherwise.  The stop at end
			 * flag works like in other move commands
			 */
			void bezierMove(const std::vector<Geometry2d::Point>& controls,
					MotionCmd::OrientationType facing,
					MotionCmd::PathEndType endpoint=MotionCmd::StopAtEnd);

			/**
			 * Move using timed-positions, so that each node has a target time
			 * Also, the command needs a start time, so that it can calculate deltas
			 * in seconds
			 */
			void move(const std::vector<MotionCmd::PathNode>& timedPath, uint64_t start);

			/**
			 * Apply direct motion commands to the motors - use only for calibration
			 * Vector of speeds must have 4 elements - will pad with zeros otherwise
			 * Only use this function for calibration
			 */
			void directMotorCommands(const std::vector<int8_t>& speeds);

			/**
			 * Move using direct velocity control by specifying
			 * translational and angular velocity
			 */
			void directMotionCommands(const Geometry2d::Point& trans, double ang);

			/**
			 * Makes the robot spin in a specified direction
			 */
			void spin(MotionCmd::SpinType dir);

			/*
			 * Enable dribbler (note: can go both ways)
			 */
			void dribble(int8_t speed);

			/**
			 * Pivots around a given point in a particular direction
			 * Specify direction manually, or with bool
			 */
			void pivot(Geometry2d::Point ctr, MotionCmd::PivotType dir);
			void pivot(Geometry2d::Point center, bool cw);

			/**
			 * Face a point while remaining in place
			 */
			void face(Geometry2d::Point pt, bool continuous = false);

			/**
			 * Remove the facing command
			 */
			void faceNone();

			/**
			 * enable kick when ready at a given strength
			 */
			void kick(uint8_t strength);

			/**
			 * enable chip when ready at a given strength
			 */
			void chip(uint8_t strength);

			ObstacleGroup &obstacles() const;

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
			
			// True if this robot should not be used in plays (for mixed play)
			bool exclude;

			/**
			 * Convenience function for changing the approachOpponent flag given a robot key
			 */
			void approachOpp(Robot * opp, bool value);

			// External access functions for utility reasons

			/** adds the pose to the history in the state variable */
			void updatePoseHistory();

		protected:
			GameplayModule *_gameplay;

			int _id;
			bool _self;
			SystemState::Robot *_packet;
			uint64_t _lastChargedTime;
	};
}
