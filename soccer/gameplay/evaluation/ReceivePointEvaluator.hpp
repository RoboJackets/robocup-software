/*
 * ReceivePointEvaluator.hpp
 *
 *  Created on: Jun 20, 2013
 *      Author: matt
 */

#ifndef RECEIVEPOINTEVALUATOR_HPP_
#define RECEIVEPOINTEVALUATOR_HPP_


#include "WindowEvaluator.hpp"
#include <Configuration.hpp>
#include "../../SystemState.hpp"
#include <Geometry2d/Segment.hpp>
#include "../../Robot.hpp"

namespace Gameplay {

class ReceivePointEvaluator {
public:

	static void createConfiguration(Configuration* cfg);

	static Geometry2d::Point FindReceivingPoint(SystemState* state, Geometry2d::Point receiverPos, Geometry2d::Point ballPos, Geometry2d::Segment receivingLine, float* out_GoalWindowWidth=0);


	static float ComputePassChannelWidth(SystemState *state, Geometry2d::Segment &segment, OurRobot *excludeBot1 = NULL, OurRobot *excludeBot2 = NULL);


private:
	static ConfigBool* _visualize;
};

} /* namespace Gameplay */
#endif /* RECEIVEPOINTEVALUATOR_HPP_ */
