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
#include "../../evaluation/ReceivePointEvaluator.hpp"

namespace Gameplay {
namespace Plays {

class TestReceivingPointPicking: public Gameplay::Play {
public:
	TestReceivingPointPicking(GameplayModule *gameplay);
	virtual bool run();
	~TestReceivingPointPicking();

private:
	GameplayModule* _gameplay;
	Behaviors::Move _move;
};

} /* namespace Plays */
} /* namespace Gameplay */
#endif /* TestReceivingPointPicking_HPP_ */
