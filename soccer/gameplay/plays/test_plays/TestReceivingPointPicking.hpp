/*
 * TestReceivingPointPicking.hpp
 *
 *  Created on: Jun 19, 2013
 *      Author: Matthew Barulic
 */

#ifndef TestReceivingPointPicking_HPP_
#define TestReceivingPointPicking_HPP_

#include "../../Play.hpp"
#include "../../behaviors/Move.hpp"
#include "../../Window.hpp"

namespace Gameplay {
namespace Plays {

class TestReceivingPointPicking: public Gameplay::Play {
public:
	TestReceivingPointPicking(GameplayModule *gameplay);
	Geometry2d::Point FindReceivingPoint(SystemState* state, Robot* robot, Geometry2d::Point ballPos, Geometry2d::Segment receivingLine);
	virtual bool run();
	~TestReceivingPointPicking();

private:
	GameplayModule* _gameplay;
	OurRobot* _robot;
	Behaviors::Move _move;
};

} /* namespace Plays */
} /* namespace Gameplay */
#endif /* TestReceivingPointPicking_HPP_ */
