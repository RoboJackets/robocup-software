/*
 * AnalyticPassPlanner.hpp
 *
 *  Created on: Dec 9, 2009
 *      Author: Philip Rogers
 */

#pragma once

#include <gameplay/Robot.hpp>
#include <gameplay/optimizer/PassConfig.hpp>
#include <gameplay/GameplayModule.hpp>

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/foreach.hpp>

class AnalyticPassPlanner {

public:
	typedef boost::ptr_vector<PassConfig> PassConfigVector;

	AnalyticPassPlanner(Gameplay::GameplayModule* gameplay) : _gameplay(gameplay) {}

	void generateAllConfigs(const Geometry2d::Point &ballPos,
							std::set<Gameplay::Robot *> &robots,
							PassConfigVector &passConfigResult);
	void evaluateConfigs(std::set<Gameplay::Robot *> &_robots,
						 Gameplay::Robot** _opponents,
						 PassConfigVector &passConfigs);

protected:
	Gameplay::GameplayModule * _gameplay;
};
