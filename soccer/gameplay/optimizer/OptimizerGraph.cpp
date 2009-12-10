/*
 * OptimizerGraph.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: alexgc
 */

#include <iostream>
#include <gameplay/optimizer/OptimizerGraph.hpp>

using namespace Gameplay;
using namespace Optimization;
using namespace std;

OptimizerGraph::OptimizerGraph() {
	// TODO Auto-generated constructor stub

}

OptimizerGraph::~OptimizerGraph() {
	// TODO Auto-generated destructor stub
}

void OptimizerGraph::print(const std::string& name) const {
	cout << "OptimizerGraph: " << name << endl;
}

bool OptimizerGraph::equals(const OptimizerGraph& expected, double tol) const {
	return false;
}

