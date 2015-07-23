#pragma once

#include <stdint.h>
#include <vector>
#include <boost/optional.hpp>
#include <boost/array.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/circular_buffer.hpp>
#include <QColor>
#include <Eigen/Geometry>
#include <Constants.hpp>
#include <Utils.hpp>

#include <planning/CompositePath.hpp>
#include <planning/InterpolatedPath.hpp>
#include <planning/RRTPlanner.hpp>
#include "planning/MotionConstraints.hpp"
#include "planning/MotionCommand.hpp"

#include <protobuf/RadioTx.pb.h>
#include <protobuf/RadioRx.pb.h>


class SystemState;
class RobotConfig;
class RobotStatus;
class MotionControl;
class RobotFilter;

namespace Packet
{
	class DebugText;
	class LogFrame_Robot;
};

namespace Gameplay
{
	class GameplayModule;
}

namespace Planning
{
	class RRTPlanner;
}

/**
 * @brief Contains robot motion state data
 * @details This class contains data that comes from the vision system
 * including position data and which camera this robot was seen by and
 * what time it was last seen.
 */
class RobotPose
{
public:
	RobotPose():
		visible(false),
		angle(0),
		angleVel(0),
		time(0),
		visionFrame(0)
	{
		// normalize angle so it's always positive
		while (angle < 0) angle += 2.0 * M_PI;
	}

	bool visible;
	Geometry2d::Point pos;
	Geometry2d::Point vel;
	float angle;	///	angle in radians.  0 radians means the robot is aimed along the x-axis
	float angleVel;	///	angular velocity in radians/sec

	// Time at which this estimate is valid
	Time time;
	int visionFrame;
};

class Robot: public RobotPose
{
public:
	Robot(unsigned int shell, bool self);
	~Robot();

	/**
	 * ID number for the robot.  This is the number that the dot pattern on the
	 * top of the robot represents
	 */
	unsigned int shell() const
	{
		return _shell;
	}

	/**
	 * Check whether or not this robot is on our team
	 */
	bool self() const
	{
		return _self;
	}

	/**
	 * Get the robot's position filter.
	 */
	RobotFilter *filter() const
	{
		return _filter;
	}

	bool operator==(const Robot &other) {
		return shell() == other.shell() && self() == other.self();
	}

	bool equals(const Robot &other) {
		return *this == other;
	}

	std::string to_string() const {
		return 	std::string("<Robot ") +
						(self() ? "us[" : "them[") +
						std::to_string(shell()) +
						"], pos=" +
						pos.toString() +
						">";
	}

	friend std::ostream& operator<< (std::ostream& stream, const Robot& robot) {
		stream << robot.to_string();
		return stream;
	}

	friend std::ostream& operator<< (std::ostream& stream, const Robot* robot) {
		stream << robot->to_string();
		return stream;
	}

private:
	unsigned int _shell;
	bool _self;
	RobotFilter *_filter;
};


/**
 * @brief A robot on our team
 * @details This extends Robot and provides methods for interacting with our robots.
 * A few things this class is responsible for:
 * - specifying target position, velocity, angle, etc
 * - kicking and chipping the ball
 * - keeping track of which hardware revision this bot is
 * - avoidance of the ball and other robots (this info is fed to the path planner)
 * - playing the GT fight song
 */
class OurRobot: public Robot {
public:
	typedef boost::array<float,Num_Shells> RobotMask;

	RobotConfig *config;
	RobotStatus *status;

	/**
	 * @brief Construct a new OurRobot
	 * @param shell The robot ID
	 * @param state A pointer to the global system state object
	 */
	OurRobot(int shell, SystemState *state);
	~OurRobot();

	void addStatusText();

	void addText(const QString &text, const QColor &color = Qt::white, const QString &layerPrefix = "RobotText");

	// kicker readiness checks
	bool charged() const; /// true if the kicker is ready
	float kickTimer() const; /// returns the time since the kicker was last charged, 0.0 if ready

	const Geometry2d::Segment kickerBar() const; /// segment for the location of the kicker
	Geometry2d::Point pointInRobotSpace(const Geometry2d::Point& pt) const; /// converts a point to the frame of reference of robot

	// simple checks to do geometry
	/** returns true if the position specified is behind the robot */
	//FIXME - Function name and comment don't match
	bool behindBall(const Geometry2d::Point& ballPos) const;

	// Gets the robot quaternion.  Returns false (and does not change q) if not available.
	boost::optional<Eigen::Quaternionf> quaternion() const;

	// Commands

	const MotionConstraints &motionConstraints() const {
		return _motionConstraints;
	}

	MotionConstraints &motionConstraints() {
		return _motionConstraints;
	}

	/**
	 * Returns a temporary observing pointer to the path of the robot.
	 * This is only currently supported for legacy reasons.
	 * Saving the pointer may lead to seg faults as it may be deleted by the Robot who owns it.
	 */
	const Planning::Path* path() const {
		return _path.get();
	}

	///	clears old radioTx stuff, resets robot debug text, and clears local obstacles
	void resetForNextIteration();

	///	clears all fields in the robot's MotionConstraints object, causing the robot to stop
	void resetMotionConstraints();

	/** Stop the robot */
	void stop();

	/**
	 * @brief Move to a given point using the default RRT planner
	 * @param endSpeed - the speed we should be going when we reach the end of the path
	 */
	void move(const Geometry2d::Point &goal, Geometry2d::Point endVelocity = Geometry2d::Point());


	/**
	 * @brief Move to a given point bypassing the RRT Path planner. This will plan a direct path ignoring 
	 *			all obstacles and the starting velocity	
	 * @param endSpeed - the speed we should be going when we reach the end of the path
	 */
	void moveDirect(const Geometry2d::Point &goal, float endSpeed = 0);

	Time pathStartTime() const {
		return _pathStartTime;
	}

	/**
	 * The number of consecutive times since now that we've set our path to something new.  This
	 * causes issues in Motion Control because the path start time is constantly reset, so we track
	 * it here and compensate for it in MotionControl.
	 * See _pathChangeHistory for more info
	 */
	int consecutivePathChangeCount() const;

	/**
	 * Sets the angleVelocity in the robot's MotionConstraints
	 */
	void angleVelocity(const float angleVelocity);

	/**
	 * Sets the worldVelocity in the robot's MotionConstraints
	 */
	void worldVelocity(const Geometry2d::Point &targetWorldVel);

	/**
	 * Face a point while remaining in place
	 */
	void face(const Geometry2d::Point &pt);

	/**
	 * Remove the facing command
	 */
	void faceNone();

	/**
	 * The robot pivots around it's mouth toward the given target
	 */
	void pivot(const Geometry2d::Point &pivotTarget);



	/*
	 * Enable dribbler (0 to 127)
	 */
	void dribble(uint8_t speed);


	/**
	 * KICKING/CHIPPING
	 *
	 * When we call kick() or chip(), it doesn't happen immediately.
	 * It primes the kicker or chipper to kick at the designated power the next time
	 * the bot senses that it has the ball.  Once this happens, we record the time
	 * of the actual kick.
	 */


	/**
	 * enable kick when ready at a given percentage of the currently set max kick power.
	 * @param strength a value between 0 and 1
	 */
	void kick(float strength);

	/**
	 * enable kick when ready at a given strength (0-255)
	 */
	void kickLevel(uint8_t strength);

	/**
	 * enable chip when ready at a given percentage of the currently set max chip power.
	 * @param strength a value between 0 and 1
	 */
	void chip(float strength);

	/**
	 * enable chip when ready at a given strength (0-255)
	 */
	void chipLevel(uint8_t strength);

	/**
	 * @brief Undoes any calls to kick() or chip().
	 */
	void unkick();

	Time lastKickTime() const;

	//	checks if the bot has kicked/chipped very recently.
	bool justKicked() {
		return timestamp() - lastKickTime() < 0.25 * SecsToTimestamp;
	}


	/**
	 * Gets a string representing the series of commands called on the robot this iteration.
	 * Contains face(), move(), etc - used to display in the BehaviorTree tab in soccer
	 */
	std::string getCmdText() const;

	/**
	 * ignore ball sense and kick immediately
	 */
	void kickImmediately(bool im);

	boost::ptr_vector<Packet::DebugText> robotText;

	// True if this robot will treat opponents as obstacles
	// Set to false for defenders to avoid being herded
	bool avoidOpponents() const;
	void avoidOpponents(bool enable);

	// Add a custom radius avoidance of the ball
	// creates an obstacle around the ball
	// if radius is 0 or less, it disables avoidance
	void disableAvoidBall();
	void avoidBallRadius(float radius);
	float avoidBallRadius() const;
	void resetAvoidBall();	//	sets avoid ball radius to Ball_Avoid_Small

	/**
	 * Adds an obstacle to the local set of obstacles for avoidance
	 * Cleared after every frame
	 */
	void localObstacles(const std::shared_ptr<Geometry2d::Shape>& obs) { _local_obstacles.add(obs); }
	const Geometry2d::CompositeShape& localObstacles() const { return _local_obstacles; }
	void clearLocalObstacles() { _local_obstacles.clear(); }



	// opponent approach interface

	void resetAvoidRobotRadii();

	void approachAllOpponents(bool enable = true);
	void avoidAllOpponents(bool enable = true);

	/** checks if opponents are avoided at all */
	bool avoidOpponent(unsigned shell_id) const;

	/** @return true if we are able to approach the given opponent */
	bool approachOpponent(unsigned shell_id) const;

	/** returns the avoidance radius */
	float avoidOpponentRadius(unsigned shell_id) const;

	/** returns the avoidance radius */
	void avoidAllOpponentRadius(float radius);

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

	/**
	 * Sets the avoid radius of all teammates to @radius for this robot.
	 * This is useful to easily keep our teammates from bumping the ball carrier.
	 */
	void shieldFromTeammates(float radius);

	// gameplay interface - interface for delayed update/planning

	/**
	 * Replans the path if needed.
	 * Sets some parameters on the path.
	 */
	void replanIfNeeded(const Geometry2d::CompositeShape& global_obstacles);


	/** status evaluations for choosing robots in behaviors - combines multiple checks */
	bool chipper_available() const;
	bool kicker_available() const;
	bool dribbler_available() const;
	bool driving_available(bool require_all = true) const; // checks for motor faults - allows one wheel failure if require_all = false

	// lower level status checks
	bool hasBall() const;
	bool ballSenseWorks() const;
	bool kickerWorks() const;
	float kickerVoltage() const;
	Packet::HardwareVersion hardwareVersion() const;

	/** radio packets */
	Packet::RadioTx::Robot radioTx;

	Packet::RadioRx &radioRx() {
		return _radioRx;
	}
	const Packet::RadioRx &radioRx() const {
		return _radioRx;
	}

	const Planning::MotionCommand motionCommand() const {
		return _motionCommand;
	}

	MotionControl *motionControl() const
	{
		return _motionControl;
	}

	SystemState *state() const
	{
		return _state;
	}

	/**
	 * @param age Time (in microseconds) that defines non-fresh
	 */
	bool rxIsFresh(Time age = 500000) const;

	/**
	 * @brief Starts the robot playing the fight song
	 */
	void sing()
	{
		addText("GO TECH!", QColor(255,0,255), "Sing");
		radioTx.set_sing(true);
	}

	bool isPenaltyKicker = false;

	static void createConfiguration(Configuration *cfg);

	double distanceToChipLanding(int chipPower);
	uint8_t chipPowerForDistance(double distance);

protected:
	MotionControl *_motionControl;

	SystemState *_state;

	// obstacle management
	Geometry2d::CompositeShape _local_obstacles; /// set of obstacles added by plays
	RobotMask _self_avoid_mask, _opp_avoid_mask;  /// masks for obstacle avoidance
	float _avoidBallRadius; /// radius of ball obstacle

	Planning::MotionCommand _motionCommand;
	Planning::MotionCommand::CommandType _lastCommandType;
	MotionConstraints _motionConstraints;

	std::shared_ptr<Planning::PathPlanner> _planner;	/// single-robot RRT planner

	void setPath(std::unique_ptr<Planning::Path> path);

	std::unique_ptr<Planning::Path> _path;	/// latest path



	Time _pathStartTime;

	///	whenever the constraints for the robot path are changed, this is set to true to trigger a replan
	bool _pathInvalidated;

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
	Geometry2d::CompositeShape createRobotObstacles(const std::vector<ROBOT*>& robots, const RobotMask& mask) const {
		Geometry2d::CompositeShape result;
		for (size_t i=0; i<RobotMask::size(); ++i)
			if (mask[i] > 0 && robots[i] && robots[i]->visible)
				result.add(std::shared_ptr<Geometry2d::Shape>(new Geometry2d::Circle(robots[i]->pos, mask[i])));
		return result;
	}


	/**
	 * Only adds obstacles within the checkRadius of the passed in position
	 * Creates a set of obstacles from a given robot team mask,
	 * where mask values < 0 create no obstacle, and larger values
	 * create an obstacle of a given radius
	 *
	 * NOTE: mask must not be set for this robot
	 *
	 * @param robots is the set of robots to use to create a mask - either self or opp from _state
	 */
	template<class ROBOT>
	Geometry2d::CompositeShape createRobotObstacles(const std::vector<ROBOT*>& robots, const RobotMask& mask,
				 Geometry2d::Point currentPosition, float checkRadius) const {
		Geometry2d::CompositeShape result;
		for (size_t i=0; i<RobotMask::size(); ++i)
			if (mask[i] > 0 && robots[i] && robots[i]->visible) {
				if (currentPosition.distTo(robots[i]->pos)<=checkRadius) {
					result.add(std::shared_ptr<Geometry2d::Shape>(new Geometry2d::Circle(robots[i]->pos, mask[i])));
				}
			}
		return result;
	}

	/**
	 * Creates an obstacle for the ball if necessary
	 */
	std::shared_ptr<Geometry2d::Shape> createBallObstacle() const;

protected:
	friend class Processor;

	///	The processor mutates RadioRx in place and calls this afterwards to let it know that it changed
	void radioRxUpdated();


protected:
	friend class MotionControl;

	/**
	 * There are a couple cases where the robot's path gets updated very often (almost every iteration):
	 * * the current path is blocked by an obstacle
	 * * the move() target keeps changing
	 *
	 * This causes the _pathStartTime to constantly be reset and motion control looks about zero seconds
	 * into the planned path and sends the robot a velocity command that's really really tiny, causing
	 * it to barely move at all.
	 *
	 * Our solution to this is to track the last N path changes in a circular buffer so we can tell if
	 * we're hitting this scenario.  If so, we can compensate, by having motion control look further into
	 * the path when commanding the robot.
	 */
	boost::circular_buffer<bool> _pathChangeHistory;	//	tracks whether or not we got a new path for the last N iterations

	///	the size of _pathChangeHistory
	static const int PathChangeHistoryBufferSize = 10;

	bool _didSetPathThisIteration;


private:
	void _kick(uint8_t strength);
	void _chip(uint8_t strength);
	void _unkick();

	uint32_t _lastKickerStatus;
	Time _lastKickTime;
	Time _lastChargedTime;

	Packet::RadioRx _radioRx;

	/**
	 * We build a string of commands such as face(), move(), etc at each iteration
	 * Then display this in the BehaviorTree tab in soccer
	 */
	//	note: originally this was not a pointer, but I got weird errors about a deleted copy constructor...
	std::stringstream *_cmdText;

	void _clearCmdText();


	///	default values for avoid radii
	static ConfigDouble *_selfAvoidRadius;
	static ConfigDouble *_oppAvoidRadius;
	static ConfigDouble *_oppGoalieAvoidRadius;
	static ConfigDouble *_goalChangeThreshold;
	static ConfigDouble *_replanTimeout;
};

/**
 * @brief A robot that is not on our team
 * @details This is a subclass of Robot, but really doesn't provide
 * any extra functionality.
 */
class OpponentRobot: public Robot {
public:
	OpponentRobot(unsigned int shell): Robot(shell, false) {}
};
