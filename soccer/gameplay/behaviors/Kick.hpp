#pragma once

#include <gameplay/Behavior.hpp>
#include <gameplay/evaluation/WindowEvaluator.hpp>
#include <gameplay/behaviors/PivotKick.hpp>
#include <gameplay/behaviors/LineKick.hpp>
#include <gameplay/tactics/LuckyKick.hpp>

namespace Gameplay
{
namespace Behaviors
{

/**
 * Kick behavior will attempt to kick the ball at the goal, or downfield,
 * incorporating switching between kick types and bump and trap mechanism
 *
 * Kicking is handled by subbehaviors, while shot selection is handled here
 */
class Kick: public SingleRobotBehavior
{
public:
	Kick(GameplayModule *gameplay);

	static void createConfiguration(Configuration *cfg);

	virtual bool run();

	inline bool done() const { return _done; }

	/** sets the state back to default state */
	void restart();

	/** default target to full segment for opponent's goal */
	void setTargetGoal();

	/*** set a specific segment */
	void setTarget(const Geometry2d::Segment &seg);

	/** get current target */
	Geometry2d::Segment target() const { return _target; }

	/** find best segment on target, @return true if one exists, and returns segment in result */
	bool findShot(const Geometry2d::Segment& segment, Geometry2d::Segment& result, bool chipper,
			float min_segment_length = 0.1) const;

	/** simple flags to allow for backup targets if target not feasible */
	// Default to off
	bool enableGoalLineShot;				/// will try to kick anywhere through goal line
	bool enableLeftDownfieldShot;   /// kick off left edge of far half field
	bool enableRightDownfieldShot;  /// kick off rigth edge of far half field

	bool enableKick; ///enables the ability to kick the ball
	bool kickReady; ///true if robot is ready to kick the ball;
	// overrides
	bool override_aim;  /// if true, will shoot regardless of obstacles

	/** simple flag to enable pushing of opponent robots if necessary */
	bool enablePushing;    /// if true, disables collision detection on opponents
	bool forceChip;        /// if true, will only chip

	/** flags to allow for kicking styles	 */
	bool use_line_kick;
	bool use_chipper;
	short dribbler_speed;

	/** flags to take opportunities **/
	bool take_open_kicks;

	// set the kick/chip power directly
	uint8_t kick_power;
	uint8_t chip_power;

	// calculate and set the chip power
	void calculateChipPower(double dist);

	// chipping parameters - should get calculated somehow
	double minChipRange;
	double maxChipRange;

	/** sets line kicking parameters */
	void setLineKickVelocityScale(double scale_speed, double scale_acc, double scale_w);

private:
	bool _done;

	// segment-specific kick
	Geometry2d::Segment _target;

	// Subbehaviors
	PivotKick _pivotKick;
	LineKick _lineKick;

	// parameters for chip model
	static ConfigDouble *_chip_min_range;
	static ConfigDouble *_chip_max_range;
	static ConfigInt *_chip_min_power;
	static ConfigInt *_chip_max_power;

	// Luck?
	LuckyKick lucky;
};

} // \namespace Behaviors
} // \namespace Gameplay
