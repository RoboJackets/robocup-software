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
#include <planning/rrt.hpp>
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
 * @brief Specifies a location that a robot should attempt to get to
 */
class MotionTarget
{
public:
	MotionTarget()
	{
		pathLength = 0;
	}
	
	///	The point on the field that the robot should get to
	Geometry2d::Point pos;

	/**
	 * @brief Length of path in meters
	 * @details This is set by the path planner after it plans where the robot will go.
	 */
	float pathLength;
};

/**
 * @brief Specifies which direction a robot should face
 * @details A face target consists of a 2d point on the field that a robot should
 *          face towards.
 */
class FaceTarget {
public:

	/**
	 * The point on the field that the robot should attempt to face towards
	 */	
	Geometry2d::Point pos;
};

class MotionCommand
{
	public:
		MotionCommand()
		{
			vScale = 1.0;
			wScale = 1.0;
		}

		//FIXME - Remove pathLength.  Store a path in MotionCommand.  What about facing?
		
		// Any of these optionals may be set before motion control runs.
		// Motion control will fill in the blanks.  This allows bypassing parts of motion control.
		
		boost::optional<MotionTarget> target;
		boost::optional<FaceTarget> face;
		
		float vScale;
		float wScale;
		
		/// Velocity in world coordinates
		boost::optional<Geometry2d::Point> worldVel;
		
		/// Velocity in body coordinates (the front of the robot is +X)
		boost::optional<Geometry2d::Point> bodyVel;
		
		/// Angular velocity in rad/s counterclockwise
		boost::optional<float> angularVelocity;
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
class OurRobot: public Robot
{
public:
	typedef boost::array<float,Num_Shells> RobotMask;

	RobotConfig *config;
	RobotStatus *status;

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

	void setVScale(float scale = 1.0); /// scales the velocity
	void setWScale(float scale = 0.5); /// scales the angular velocity
	void resetMotionCommand();  /// resets all motion commands for the robot

	/** Stop the robot */
	void stop();

	/**
	 * @brief Move to a given point using the default RRT planner
	 */
	void move(Geometry2d::Point goal);

	/**
	 * @brief Move along a path for waypoint-based control
	 */
	void move(const std::vector<Geometry2d::Point>& path);

	/**
	 * Pivot around a point at a fixed radius and direction (CCW or CW),
	 * specified by the magnitude of the angular velocity.
	 * Used primarily for aiming around a ball.  Note that this will
	 * not handle obstacle avoidance.
	 */
	void pivot(double w, double radius);
	
	void pivot(double w, const Geometry2d::Point& center)
	{
		pivot(w, (pos - center).mag());
	}

	/**
	 * Move using direct velocity control by specifying
	 * translational and angular velocity.
	 * 
	 * All velocities are in m/s
	 */
	void bodyVelocity(const Geometry2d::Point& v);
	void worldVelocity(const Geometry2d::Point& v);
	void angularVelocity(double w);

	/*
	 * Enable dribbler (0 to 127)
	 */
	void dribble(int8_t speed);

	/**
	 * Face a point while remaining in place
	 */
	void face(Geometry2d::Point pt);

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
	void execute(const ObstacleGroup& global_obstacles);

	const std::vector<void *> &commandTrace() const
	{
		return _commandTrace;
	}

	/** motion command - sent to point/wheel controllers, is valid when _planning_complete is true */
	MotionCommand cmd;

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

	/** velocity specification for direct velocity control */
	Geometry2d::Point cmd_vel;
	float cmd_w;

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

	uint64_t _lastChargedTime; // TODO: make this a boost pointer to avoid update() function

	/** Planning components for delayed planning */
	bool _usesPathPlanning;
	boost::optional<Geometry2d::Point> _delayed_goal;   /// goal from move command

	// obstacle management
	ObstacleGroup _local_obstacles; /// set of obstacles added by plays
	RobotMask _self_avoid_mask, _opp_avoid_mask;  /// masks for obstacle avoidance
	float _avoidBallRadius; /// radius of obstacle

	Planning::Path _path;	/// latest path
	Planning::RRTPlanner *_planner;	/// single-robot RRT planner

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


private:
	uint32_t _lastKickerStatus;
	uint64_t _lastKickTime;

	Packet::RadioRx _radioRx;
};

/**
 * @brief A robot that is not on our team
 * @details This is a subclass of Robot, but really doesn't provide
 * any extra functionality.
 */
class OpponentRobot: public Robot
{
public:
	OpponentRobot(unsigned int shell): Robot(shell, false) {}
};
