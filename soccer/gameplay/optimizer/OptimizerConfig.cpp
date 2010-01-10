/*
 * OptimizerConfig.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: alexgc
 */

#include <iostream>
#include <sstream>
#include <gtsam/Vector.h>
#include <OptimizerConfig.hpp>
#include <OptimizerUtils.hpp>

using namespace Gameplay;
using namespace Optimization;
using namespace std;

OptimizerConfig::OptimizerConfig() {
	// TODO Auto-generated constructor stub
}

OptimizerConfig::~OptimizerConfig() {
	// TODO Auto-generated destructor stub
}

Geometry2d::Point OptimizerConfig::getInit(int idx) const {
	string key = genKey(idx, "init");
	return vec2pt(get(key));
}

Geometry2d::Point OptimizerConfig::getFinal(int idx) const {
	string key = genKey(idx, "final");
	return vec2pt(get(key));
}

double OptimizerConfig::getTime(int idx) const {
	string key = genKey(idx, "t");
	return get(key)(0);
}

void OptimizerConfig::initRobot(int idx, const Geometry2d::Point& init, const Geometry2d::Point& end) {
	insert(genKey(idx, "init"), pt2vec(init));
	insert(genKey(idx, "final"), pt2vec(end));
}

void OptimizerConfig::initTime(int idx, const PassState& s) {
	insert(genKey(idx, "t"), gtsam::Vector_(1, s.timeLeaveState));
}

void OptimizerConfig::print(const std::string& name) const {
	VectorConfig::print("OptimizerConfig - " + name);
}

bool OptimizerConfig::equals(const OptimizerConfig& expected, double tol) const {
	return false;
}

void check_size(const string& key, const Vector & vj, const Vector & dj) {
  if (dj.size()!=vj.size()) {
    cout << "For key \"" << key << "\"" << endl;
    cout << "vj.size = " << vj.size() << endl;
    cout << "dj.size = " << dj.size() << endl;
    throw(std::invalid_argument("OptimizerConfig::+ mismatched dimensions"));
  }
}

OptimizerConfig OptimizerConfig::exmap(const VectorConfig & delta) const
{
	OptimizerConfig newConfig;
	for (const_iterator it = values.begin(); it!=values.end(); it++) {
		string j = it->first;
		const Vector &vj = it->second;
		if (delta.contains(j)) {
			const Vector& dj = delta[j];
			check_size(j,vj,dj);
			newConfig.insert(j, vj + dj);
		} else {
			newConfig.insert(j, vj);
		}
	}
	return newConfig;
}

