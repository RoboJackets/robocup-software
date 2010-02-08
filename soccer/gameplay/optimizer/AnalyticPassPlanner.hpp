/*
 * AnalyticPassPlanner.hpp
 *
 *  Created on: Dec 9, 2009
 *      Author: Philip Rogers
 */

#ifndef ANALYTICPASSPLANNER_HPP_
#define ANALYTICPASSPLANNER_HPP_

#include <iostream>
#include <fstream>
#include <gameplay/Robot.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/foreach.hpp>
#include <PassState.hpp>
#include <PassConfig.hpp>

#include <motion/planning/rrt.hpp>
#include <framework/Path.hpp>

typedef boost::ptr_vector<PassConfig> PassConfigVector;

using namespace Geometry2d;
using namespace Gameplay;
using namespace std;

namespace AnalyticPassPlanner {
	void generateAllConfigs(const Point &ballPos, set<Robot *> &_robots, PassConfigVector &passConfigResult);
	void evaluateConfigs(set<Robot *> &_robots, Robot** _opponents, PassConfigVector &passConfigs);
};

#endif /* ANALYTICPASSPLANNER_HPP_ */
