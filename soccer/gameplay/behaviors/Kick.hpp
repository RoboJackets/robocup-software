#pragma once

#include <gameplay/Behavior.hpp>
#include <gameplay/Window.hpp>
#include <gameplay/behaviors/PivotKick.hpp>
#include <gameplay/behaviors/LineKick.hpp>

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

	virtual bool run();

	inline bool done() const { return _state == State_Done; }

	/** sets the state back to default state */
	void restart();

	/** default target to full segment for opponent's goal */
	void setTargetGoal();

	void setTarget(const Geometry2d::Segment &seg);

	/** find best segment on target, @return true if one exists, and returns segment in result */
	bool findShot(const Geometry2d::Segment& segment, Geometry2d::Segment& result,
			float min_segment_length = 0.1) const;
private:
	typedef enum
	{
		State_PivotKick,   /// default kicking mode with aiming
		State_LineKick,    /// faster kicking mode when opponents nearby
		State_Bump,   		 /// when close to another robot, bump ball away
		State_Trap,        /// when ball is moving quickly, go around behind the ball first
		State_Done				 /// after a successful shot
	} State;

	State _state;

	// segment-specific kick
	Geometry2d::Segment _target;

	// Subbehaviors
	PivotKick _pivotKick;
	LineKick _lineKick;

};

} // \namespace Behaviors
} // \namespace Gameplay
