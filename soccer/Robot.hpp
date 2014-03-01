#pragma once

#include <stdint.h>
#include <vector>
#include <boost/optional.hpp>
#include <boost/array.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <QColor>
#include <Eigen/Geometry>
#include <Constants.hpp>
#include <planning/Path.hpp>
#include <planning/RRTPlanner.hpp>
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
	class Behavior;
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
	}
	
	bool visible;
	Geometry2d::Point pos;
	Geometry2d::Point vel;
	float angle;
	float angleVel;
	
	// Time at which this estimate is valid
	uint64_t time;
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

private:
	unsigned int _shell;
	bool _self;
	RobotFilter *_filter;
};


/**
 * This class contains the motion constraints that the high-level logic sets for a robot.
 * For position: set EITHER @motionTarget OR @targetWorldVel.
 * For angle: set EITHER @targetAngleVel OR @faceTarget.
 */
struct MotionConstraints {

	/**
	 * Position
	 */

	///	A point on the field that the robot should use path-planning to get to
	boost::optional<Geometry2d::Point> targetPos;
	
	/// Set the velocity in world coordinates directly (circumvents path planning)
	boost::optional<Geometry2d::Point> targetWorldVel;

	/**
	 * Angle
	 */
	
	/// Angular velocity in rad/s counterclockwise
	boost::optional<float> targetAngleVel;

	///	A global point on the field that the robot should face towards
	boost::optional<Geometry2d::Point> faceTarget;
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

	// Gets the robot quaternion.  Returns false (and does not change q) if not available.
	boost::optional<Eigen::Quaternionf> quaternion() const;
	
	// Commands

	//	FIXME: document
	void setMotionConstraints(const MotionConstraints &constraints);

	///	clears all fields in the robot's MotionConstraints object, causing the robot to stop
	void resetMotionConstraints();

	/** Stop the robot */
	void stop();

	/**
	 * @brief Move to a given point using the default RRT planner
	 * @param stopAtEnd UNUSED
	 */
	void move(const Geometry2d::Point &goal, bool stopAtEnd = false);

	void worldVelocity(const Geometry2d::Point &targetWorldVel);

	/*
	 * Enable dribbler (0 to 127)
	 */
	void dribble(int8_t speed);

	/**
	 * Face a point while remaining in place
	 */
	void face(const Geometry2d::Point &pt);

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

	/**
	 * ignore ball sense and kick immediately
	 */
	void immediate(bool im);

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
	void localObstacles(const ObstaclePtr& obs) { _local_obstacles.add(obs); }
	void localObstacles(const ObstacleGroup& obs) { _local_obstacles.add(obs); }
	const ObstacleGroup& localObstacles() const { return _local_obstacles; }
	void clearLocalObstacles() { _local_obstacles.clear(); }

	// opponent approach interface

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

	// True if this robot should not be used in plays (for mixed play)
	bool exclude;

	// gameplay interface - interface for delayed update/planning

	/**
	 * Executes last motion command, retrieves the necessary set of
	 * obstacles, and performs planning
	 *
	 * Needs a set of global obstacles to use - assuming field regions and goal
	 */
	//	FIXME: rewrite comment to describe new behavior
	void replanIfNeeded(const ObstacleGroup& global_obstacles);

	const std::vector<void *> &commandTrace() const
	{
		return _commandTrace;
	}

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

	uint64_t lastKickTime() const;

	/** radio packets */
	Packet::RadioTx::Robot radioTx;

	void setRadioRx(Packet::RadioRx rx);

	Packet::RadioRx &radioRx() {
		return _radioRx;
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
	bool rxIsFresh(uint64_t age = 500000) const;

	/**
	 * @brief Starts the robot playing the fight song
	 */
	void sing()
	{
		addText("GO TECH!", QColor(255,0,255), "Sing");
		radioTx.set_sing(true);
	}

	/**
	 * @brief Undoes any calls to kick() or chip().
	 */
	void unkick()
	{
		kick(0);
		chip(0);
		radioTx.set_use_chipper(false);
	}

protected:
	MotionControl *_motionControl;
	
	SystemState *_state;

	std::vector<void *> _commandTrace;

	// obstacle management
	ObstacleGroup _local_obstacles; /// set of obstacles added by plays
	RobotMask _self_avoid_mask, _opp_avoid_mask;  /// masks for obstacle avoidance
	float _avoidBallRadius; /// radius of ball obstacle

	MotionConstraints _motionConstraints;

	Planning::RRTPlanner *_planner;	/// single-robot RRT planner
	boost::optional<Planning::Path> _path;	/// latest path
	uint64_t _pathStartTime;
	
	boost::optional<Planning::AnglePath> _anglePath;
	uint64_t _anglePathStartTime;


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


private:
	uint32_t _lastKickerStatus;
	uint64_t _lastKickTime;
	uint64_t _lastChargedTime;

	Packet::RadioRx _radioRx;
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
