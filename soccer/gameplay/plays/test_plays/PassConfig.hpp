/*
 * PassConfig.hpp
 *
 *  Created on: Dec 4, 2009
 *      Author: Philip Rogers
 */

#ifndef PASSCONFIG_HPP_
#define PASSCONFIG_HPP_

#include "PassState.hpp"
#include <boost/ptr_container/ptr_vector.hpp>

typedef boost::ptr_vector<PassState> PassStateVector;

class PassConfig {
public:
	PassConfig();
	virtual ~PassConfig();

	void addPassState(PassState* passState);
	PassState* getPassState(int idx);
	int length();
	void setWeight(double w);

	PassStateVector passStateVector;
	double weight;
};

#endif /* PASSCONFIG_HPP_ */
