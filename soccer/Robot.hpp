#pragma once

#include <stdint.h>
#include <vector>
#include <boost/optional.hpp>
#include <boost/array.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <QColor>

#include <Constants.hpp>
#include <framework/MotionCmd.hpp>
#include <framework/Path.hpp>
#include <gameplay/planning/rrt.hpp>
#include <protobuf/RadioTx.pb.h>
#include <protobuf/RadioRx.pb.h>
#include "Processor.hpp"

class SystemState;
class RobotConfig;
class MotionControl;

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
	typedef boost::array<float,Num_Shells> RobotMask;
	typedef enum {
		NONE, 			 /// makes no attempt to adjust facing
		CONTINUOUS,  /// change facing continously
		CONSTANT,    /// specifies changing facing before moving, and keeps it constant throughout trajectory
		ENDPOINT		 /// only handle facing at end (fastest)
	} FacingType;

	typedef enum {
		RRT, 			/// moves to a point with the RRT planner
		OVERRIDE  /// moves to a point without regard for obstacles
	} MoveType;

	RobotConfig * config;

	OurRobot(int shell, SystemState *state);
	~OurRobot();

	void addText(const QString &text, const QColor &color = Qt::white);

	bool haveBall() const; /// true if we have the ball

	bool hasChipper() const; /// true if robot can chip

	// kicker readiness checks
	// TODO: replace with boost::timer
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

	/*
	 * Enable dribbler (note: can go both ways)
	 */
	void dribble(int8_t speed);

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

	// True if this robot will treat opponents as obstacles
	// Set to false for defenders to avoid being herded
	bool avoidOpponents() const;
	void avoidOpponents(bool enable);

	// True if this robot intends to kick the ball.
	// This is reset when this robot's role changes.
	// This allows the robot to get close to the ball during a restart.
	// if disabled, creates a small obstacle for the ball
	bool willKick() const;
	void willKick(bool enable);

	// Add a custom radius avoidance of the ball
	// creates an obstacle around the ball
	// if radius is 0 or less, it disables avoidance
	void disableAvoidBall();
	void avoidBall(float radius);
	float avoidBall() const;

	/**
	 * Adds an obstacle to the local set of obstacles for avoidance
	 * Cleared after every frame
	 */
	void localObstacles(const ObstaclePtr& obs) { _local_obstacles.add(obs); }
	void localObstacles(const ObstacleGroup& obs) { _local_obstacles.add(obs); }
	const ObstacleGroup& localObstacles() const { return _local_obstacles; }
	void clearLocalObstacles() { _local_obstacles.clear(); }

	// opponent approach interface

	void approachAllOpponents(bool enable = true);
	void avoidAllOpponents(bool enable = true);

	/** checks if opponents are avoided at all */
	bool avoidOpponent(unsigned shell_id) const;

	/** @return true if we are able to approach opponents */
	bool approachOpponent(unsigned shell_id) const;

	/** returns the avoidance radius */
	float avoidOpponentRadius(unsigned shell_id) const;

	/** enable/disable for opponent avoidance */
	void avoidOpponent(unsigned shell_id, bool enable_avoid);

	/** enable/disable approach of opponents - diable uses larger avoidance radius */
	void approachOpponent(unsigned shell_id, bool enable_approach);

	void avoidOpponentRadius(unsigned shell_id, float radius);

	// self approach interface

	/** determines whether a robot will avoid another robot when it plans - use for priority */

	void avoidAllTeammates(bool enable = true);
	void avoidTeammate(unsigned shell_id, bool enable = true);
	void avoidTeammateRadius(unsigned shell_id, float radius);
	bool avoidTeammate(unsigned shell_id) const;
	float avoidTeammateRadius(unsigned shell_id) const;

	// True if this robot should not be used in plays (for mixed play)
	bool exclude;

	// gameplay interface - interface for delayed update/planning

	/**
	 * Executes last motion command, retrieves the necessary set of
	 * obstacles, and performs planning
	 *
	 * Needs a set of global obstacles to use - assuming field regions and goal
	 */
	void execute(const ObstacleGroup& global_obstacles);

	const std::vector<void *> &commandTrace() const
	{
	return _commandTrace;
	}

	/** motion command - sent to point/wheel controllers, is valid when _planning_complete is true */
	MotionCmd cmd;

	bool hasBall;

	/** velocity specification for direct velocity control */
	Geometry2d::Point cmd_vel;
	float cmd_w;

	/** radio packets */
	Packet::RadioTx::Robot radioTx;
	Packet::RadioRx radioRx;

	//The confidence for this robot's ball sensor
	int sensorConfidence;

	void newRevision(bool is_2011) { _newRevision = is_2011; }
	bool newRevision() const { return _newRevision; }

	MotionControl *motionControl() const
	{
		return _motionControl;
	}
	
	SystemState *state() const
	{
		return _state;
	}

protected:
	// Stores a stack trace in _commandTrace
	void setCommandTrace();

	MotionControl *_motionControl;
	
	SystemState *_state;

	std::vector<void *> _commandTrace;

	uint64_t _lastChargedTime; // TODO: make this a boost pointer to avoid update() function

	bool _newRevision; /// true if this a 2011 robot, false otherwise

	/** Planning components for delayed planning */
	MoveType _planner_type;  /// movement class - set during move
	boost::optional<Geometry2d::Point> _delayed_goal;   /// goal from move command

	// obstacle management
	ObstacleGroup _local_obstacles; /// set of obstacles added by plays
	RobotMask _self_avoid_mask, _opp_avoid_mask;  /// masks for obstacle avoidance
	float _ball_avoid; /// radius of obstacle

	Planning::Path _path;	/// latest path
	Planning::RRT::Planner *_planner;	/// single-robot RRT planner

	// planning functions

	/**
	 * Creates a set of obstacles from a given robot team mask,
	 * where mask values < 0 create no obstacle, and larger values
	 * create an obstacle of a given radius
	 *
	 * NOTE: mask must not be set for this robot
	 *
	 * @param robots is the set of robots to use to create a mask - either self or opp from _state
	 */
	template<class ROBOT>
	ObstacleGroup createRobotObstacles(const std::vector<ROBOT*>& robots, const RobotMask& mask) const {
		ObstacleGroup result;
		for (size_t i=0; i<RobotMask::size(); ++i)
			if (mask[i] > 0 && robots[i] && robots[i]->visible)
				result.add(ObstaclePtr(new CircleObstacle(robots[i]->pos, mask[i])));
		return result;
	}

	/**
	 * Creates an obstacle for the ball if necessary
	 */
	ObstaclePtr createBallObstacle() const;

	/**
	 * Given a path, finds the first local goal through mixing to create
	 * a point target for the PointController.  Finds the closest point
	 * on the path, and mixes from there
	 *
	 * @param pose is the current robot pos
	 * @param path is the path
	 * @param obstacles are a set of obstacles to use
	 */
	Geometry2d::Point findGoalOnPath(const Geometry2d::Point& pos, const Planning::Path& path,
			const ObstacleGroup& obstacles = ObstacleGroup());

	/** executes RRT planning through a set of obstacles */
	Planning::Path rrtReplan(const Geometry2d::Point& goal, const ObstacleGroup& obstacles);

	// rendering

	//FIXME - This doesn't need to be in this class.  Put it in SystemState, along with other drawing?
	void drawPath(const Planning::Path& path, const QColor &color = Qt::black, const QString &layer = "Motion");
};

class OpponentRobot: public Robot
{
public:
	OpponentRobot(unsigned int shell):
		Robot(shell, false)
	{
	}
};
