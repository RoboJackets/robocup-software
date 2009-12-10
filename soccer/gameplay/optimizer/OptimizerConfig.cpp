/*
 * OptimizerConfig.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: alexgc
 */

#include <iostream>
#include <gameplay/optimizer/OptimizerConfig.hpp>

using namespace Gameplay;
using namespace Optimization;
using namespace std;

OptimizerConfig::OptimizerConfig() {
	// TODO Auto-generated constructor stub

}

OptimizerConfig::~OptimizerConfig() {
	// TODO Auto-generated destructor stub
}

void OptimizerConfig::print(const std::string& name) const {
	cout << "OptimizerConfig: " << name << endl;
}

bool OptimizerConfig::equals(const OptimizerConfig& expected, double tol) const {
	return false;
}

