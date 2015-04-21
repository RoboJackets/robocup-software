/*
 * OurCornerKickChipToGoalArea.hpp
 *
 *  Created on: Jun 26, 2013
 *      Author: matt
 */

#ifndef OURCORNERKICKCHIPTOGOALAREA_HPP_
#define OURCORNERKICKCHIPTOGOALAREA_HPP_

#include "../Play.hpp"

#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/behaviors/positions/Defender.hpp>
#include <gameplay/behaviors/Move.hpp>
#include <gameplay/PreventDoubleTouch.hpp>

namespace Gameplay {
namespace Plays {

class OurCornerKick_ChipToGoalArea: public Gameplay::Play {
public:
	OurCornerKick_ChipToGoalArea(GameplayModule* gameplay);
	static float score(GameplayModule* gameplay);
	virtual bool run();
	~OurCornerKick_ChipToGoalArea();

protected:
	// always takes a chipper
	Behaviors::Kick _kicker;
	Behaviors::Move _center1;
	Behaviors::Move _center2;
	Behaviors::Defender _defender1, _defender2;
	PreventDoubleTouch _pdt;

	Geometry2d::Segment _target;

	enum State {
		Setup,
		Kick,
		Receive
	};

	State _state;

	Geometry2d::Point ReceiveSetupPoint1;
	Geometry2d::Point ReceiveSetupPoint2;
};

} /* namespace Plays */
} /* namespace Gameplay */
#endif /* OURCORNERKICKCHIPTOGOALAREA_HPP_ */
