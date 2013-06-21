/*
 * ReceivePointEvaluator.hpp
 *
 *  Created on: Jun 20, 2013
 *      Author: matt
 */

#ifndef RECEIVEPOINTEVALUATOR_HPP_
#define RECEIVEPOINTEVALUATOR_HPP_


#include "Window.hpp"
#include <Configuration.hpp>
#include "../framework/SystemState.hpp"

namespace Gameplay {

class ReceivePointEvaluator {
public:

	static void createConfiguration(Configuration* cfg);

	static Geometry2d::Point FindReceivingPoint(SystemState* state, Geometry2d::Point receiverPos, Geometry2d::Point ballPos, Geometry2d::Segment receivingLine);

private:
	static ConfigBool* _visualize;
};

} /* namespace Gameplay */
#endif /* RECEIVEPOINTEVALUATOR_HPP_ */
