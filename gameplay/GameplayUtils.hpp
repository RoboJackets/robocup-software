#pragma once

#include "../../../motion/planning/Tree.hpp"

#include <Geometry2d/Point.hpp>

namespace Gameplay
{
	class WindowEvaluator;
	
	float scorePosition(WindowEvaluator &win, Geometry2d::Point pos, Geometry2d::Point ball_pos);
	Geometry2d::Point bestPassInTree(WindowEvaluator &win, Motion::Tree &tree,
		Geometry2d::Point cur, Geometry2d::Point ball);
}
