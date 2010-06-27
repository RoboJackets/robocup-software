/*
 * PassConfig.hpp
 *
 *  Created on: Dec 4, 2009
 *      Author: Philip Rogers
 */

#ifndef PASSCONFIG_HPP_
#define PASSCONFIG_HPP_

#include <iostream>
#include <vector>
#include <framework/SystemState.hpp>
#include "PassState.hpp"
#include <gameplay/Robot.hpp>

class PassConfig {
	typedef std::vector<PassState> PassStateVector;
public:
	PassConfig();
	PassConfig(const PassConfig& c); /// copy constructor
	~PassConfig();

	void addPassState(const PassState& passState);
	PassState getPassState(int idx) const;
	int length() const;
	void setWeight(double w);
	friend std::ostream& operator<<(std::ostream& out, const PassConfig &config);
	friend bool operator<(const PassConfig& lhs, const PassConfig& rhs);

	/**
	 * Drawing function with color parameters
	 * Ints are in RGB order
	 * supply a Log state to perform drawing
	 */
	void drawConfig(SystemState* state, int r, int g, int b) const;


	PassStateVector passStateVector;
	double weight;
};

#endif /* PASSCONFIG_HPP_ */
