/*
 * PassConfig.hpp
 *
 *  Created on: Dec 4, 2009
 *      Author: Philip Rogers
 */

#ifndef PASSCONFIG_HPP_
#define PASSCONFIG_HPP_

#include <iostream>
#include <fstream>
#include <vector>
#include "PassState.hpp"
#include <gameplay/Robot.hpp>

using std::ostream;
using std::endl;

typedef std::vector<PassState> PassStateVector;

class PassConfig {
public:
	PassConfig();
	PassConfig(const PassConfig& c); /// copy constructor
	~PassConfig();

	void addPassState(const PassState& passState);
	PassState getPassState(int idx) const;
	int length() const;
	void setWeight(double w);
	friend ostream& operator<<(ostream& out, const PassConfig &config);
	friend bool operator<(const PassConfig& lhs, const PassConfig& rhs);

	PassStateVector passStateVector;
	double weight;
};

#endif /* PASSCONFIG_HPP_ */
