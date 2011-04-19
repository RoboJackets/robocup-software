#pragma once

#include <stdint.h>
#include <vector>
#include <boost/optional.hpp>
#include <boost/array.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <QColor>

#include <Constants.hpp>
#include <framework/SystemState.hpp>
#include <framework/MotionCmd.hpp>
#include <framework/Path.hpp>
#include <gameplay/planning/rrt.hpp>
#include <protobuf/RadioTx.pb.h>
#include <protobuf/RadioRx.pb.h>

class SystemState;
class RobotConfig;

namespace Packet
{
class DebugText;
class LogFrame_Robot;
};

namespace Gameplay
{
class GameplayModule;
class Behavior;
}

namespace Planning
{
class Dynamics;
namespace RRT
{
class Planner;
}
}

class Robot
{
public:
	Robot(unsigned int shell, bool self);

	unsigned int shell() const
	{
		return _shell;
	}

	bool self() const
	{
		return _self;
	}

	bool visible;
	Geometry2d::Point pos;
	Geometry2d::Point vel;
	float angle;
	float angleVel;

private:
	unsigned int _shell;
	bool _self;
};

class OurRobot: public Robot
{
public:
	typedef boost::optional<Geometry2d::Point> OptionalPoint;
	typedef boost::array<bool, 5> RobotMask;
	typedef enum {
		CONTINUOUS,  /// change facing continously
		CONSTANT,    /// specifies changing facing before moving, and keeps it constant throughout trajectory
		ENDPOINT		 /// only handle facing at end (fastest)
	} FacingType;

	typedef enum {
		RRT, 			/// moves to a point with the RRT planner
		BEZIER,   /// moves using the bezier planner - broken
		OVERRIDE  /// moves to a point without regard for obstacles
	} MoveType;

	typedef enum {
		KICK,					/// will maneuver to kick the ball
		AVOID_LARGE,  /// will avoid the ball with large (0.5 meter) radius - needed for rules
		AVOID_SMALL,  /// will avoid ball with small radius - needed for maneuvering
	} BallAvoid;

	boost::shared_ptr<RobotConfig> config;

	OurRobot(int shell, SystemState *state);

	void addText(const QString &text, const QColor &color = Qt::white);

	bool haveBall() const; /// true if we have the ball

	//            bool sensorConfidence() const;

	bool hasChipper() const; /// true if robot can chip

	// kicker readiness checks
	bool charged() const; /// true if the kicker is ready
	float kickTimer() const; /// returns the time since the kicker was last charged, 0.0 if ready

	/**
	 * state variable updates - includes timers, etc.  Called each frame
	 */
	//FIXME - Remove
	void update();

	const Geometry2d::Segment kickerBar() const; /// segment for the location of the kicker
	Geometry2d::Point pointInRobotSpace(const Geometry2d::Point& pt) const; /// converts a point to the frame of reference of robot

	// simple checks to do geometry
	/** returns true if the position specified is behind the robot */
	//FIXME - Function name and comment don't match
	bool behindBall(const Geometry2d::Point& ballPos) const;

	// Commands

	/**
	 * generic move command - supplies all parameters, should be masked with other functions to allow for
	 * special cases (e.g., face())
	 *
	 * @param goal is the translational goal - if not specified, goal is current position
	 * @param goal_param specifies the movement type
	 * @param facing is the facing goal - if not specifed, no facing commanded
	 * @param facing_param specifies how to handle facing
	 */
	void move(const OptionalPoint& goal, const MoveType& goal_param, const OptionalPoint& facing, const FacingType& facing_param);

	/**
	 * Obstacle avoidance command - specifies how a robot will manage obstacles
	 * Per robot avoidance uses a mask, where true treats the robot as an obstacle
	 *
	 * @param ball sets the ball avoidance parameters
	 * @param opp_robots specifies handling of opponent robots with a mask
	 * @param self_robots specifies handling of self robots with a mask
	 * @param regions allows for explicit arbitrary obstacles
	 */
	void avoid(const BallAvoid& ball, const RobotMask& opp_robots, const RobotMask& self_robots, const ObstacleGroup& regions);

	void setVScale(float scale = 1.0); /// scales the velocity
	void setWScale(float scale = 0.5); /// scales the angular velocity
	void resetMotionCommand();  /// resets all motion commands for the robot

	/** Stop the robot */
	void stop();

	/**
	 * Move to a given point using the default RRT planner
	 */
	void move(Geometry2d::Point goal, bool stopAtEnd=false);

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
	 * Apply direct motion commands to the motors - use only for calibration
	 * Vector of speeds must have 4 elements - will pad with zeros otherwise
	 * Only use this function for calibration
	 */
	void directMotorCommands(const std::vector<int8_t>& speeds);

	/**
	 * Move using direct velocity control by specifying
	 * translational and angular velocity
	 */
	void directVelocityCommands(const Geometry2d::Point& trans, double ang);

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

	boost::ptr_vector<Packet::DebugText> robotText;

	ObstacleGroup obstacles;

	// True if this robot will treat opponents as obstacles
	// Set to false for defenders to avoid being herded
	bool avoidOpponents;

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
	bool approachOpponent[Num_Shells];

	// True if this robot should not be used in plays (for mixed play)
	bool exclude;

	/**
	 * Executes last motion command, retrieves the necessary set of
	 * obstacles, and performs planning
	 *
	 * @param global_obstacles is an obstacle group with field-related things
	 * @param goal area is the set of obstacles around the goal
	 * @param isGoalie is true if this robot is the goalie - controls flags for obstacles
	 */
	void execute(const ObstacleGroup& global_obstacles, const ObstacleGroup& goal_area, bool isGoalie);

	/**
	 * Convenience function for changing the approachOpponent flag given a robot key
	 */
	void approachOpp(Robot * opp, bool value);

	const std::vector<void *> &commandTrace() const
	{
	return _commandTrace;
	}

	MotionCmd cmd;
	bool hasBall;
	Geometry2d::Point cmd_vel;
	float cmd_w;
	Packet::RadioTx::Robot radioTx;
	Packet::RadioRx radioRx;

	//The confidence for this robot's ball sensor
	int sensorConfidence;

protected:
	// Stores a stack trace in _commandTrace
	void setCommandTrace();

	SystemState *_state;

	std::vector<void *> _commandTrace;

	uint64_t _lastChargedTime;

	/** Planning components - should be generalized */
	bool _planning_complete; /// set to false by move commands, set to true if motionCmd is ready

	Planning::Path _path;	/// latest path
	Planning::RRT::Planner *_planner;	/// single-robot RRT planner

	/** robot dynamics information */
	Planning::Dynamics *_dynamics;

	/** actually performs the conversion from path->point target */
	void executeMove(bool stopAtEnd);
};

class OpponentRobot: public Robot
{
public:
	OpponentRobot(unsigned int shell):
		Robot(shell, false)
	{
	}
};
