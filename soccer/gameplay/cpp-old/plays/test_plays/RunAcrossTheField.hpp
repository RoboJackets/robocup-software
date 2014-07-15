/*
 * RunAcrossTheField.hpp
 *
 *  Created on: Jun 24, 2013
 *      Author: matt
 */

#ifndef RUNACROSSTHEFIELD_HPP_
#define RUNACROSSTHEFIELD_HPP_

#include <gameplay/Play.hpp>
#include <gameplay/behaviors/Move.hpp>

namespace Gameplay {
namespace Plays {

class RunAcrossTheField: public Gameplay::Play {
public:
	RunAcrossTheField(GameplayModule* gameplay);
	virtual bool run();
	~RunAcrossTheField();

private:
	Behaviors::Move _move;
	bool _running;
	Geometry2d::Point A;
	Geometry2d::Point B;
	Geometry2d::Point T;
	bool _hasSetT;
};

} /* namespace Plays */
} /* namespace Gameplay */
#endif /* RUNACROSSTHEFIELD_HPP_ */
