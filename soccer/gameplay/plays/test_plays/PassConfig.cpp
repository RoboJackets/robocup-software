/*
 * PassConfig.cpp
 *
 *  Created on: Dec 4, 2009
 *      Author: Philip Rogers
 */

#include <gameplay/plays/test_plays/PassConfig.hpp>

PassConfig::PassConfig() : weight(0.0) {}

PassConfig::~PassConfig() {}


void PassConfig::addPassState(PassState* passState){
	passStateVector.push_back(passState);
}

PassState* PassConfig::getPassState(int idx){
	assert(idx < (int)passStateVector.size()); // pass state index must exist
	return &passStateVector.at(idx);
}

int PassConfig::length(){
	return passStateVector.size();
}

void PassConfig::setWeight(double w){
	weight = w;
}
