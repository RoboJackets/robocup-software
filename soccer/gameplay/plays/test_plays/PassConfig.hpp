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
#include "PassState.hpp"
#include <boost/ptr_container/ptr_vector.hpp>
#include <gameplay/Robot.hpp>

using std::ostream;
using std::endl;

typedef boost::ptr_vector<PassState> PassStateVector;

class PassConfig {
public:
	PassConfig();
	~PassConfig();

	void addPassState(PassState* passState);
	PassState* getPassState(int idx);
	int length();
	void setWeight(double w);
	friend ostream& operator<<(ostream& out, const PassConfig &config);
	friend bool operator<(const PassConfig& lhs, const PassConfig& rhs);

	PassStateVector passStateVector;
	double weight;
};

#endif /* PASSCONFIG_HPP_ */
