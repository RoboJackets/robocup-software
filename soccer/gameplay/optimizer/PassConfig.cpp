/*
 * PassConfig.cpp
 *
 *  Created on: Dec 4, 2009
 *      Author: Philip Rogers
 */

#include "PassConfig.hpp"

PassConfig::PassConfig() : weight(0.0) { }

PassConfig::PassConfig(const PassConfig& c)
: weight(c.weight){
	// copy over the states
	for (int i=0; i<c.length(); ++i)
		passStateVector.push_back(c.getPassState(i));
}

PassConfig::~PassConfig() {
	passStateVector.clear();
}

void PassConfig::addPassState(const PassState& passState){
	passStateVector.push_back(passState);
}

PassState PassConfig::getPassState(int idx) const {
	assert(idx < (int)passStateVector.size()); // pass state index must exist
	return passStateVector.at(idx);
}

int PassConfig::length() const{
	return passStateVector.size();
}

void PassConfig::setWeight(double w){
	weight = w;
}

ostream& operator<<(ostream& out, const PassConfig &config){
	out << "config weight(" << config.weight << "):" << endl;
	for(int i=0; i<(int)config.passStateVector.size(); i++){
		out << "\t state(" << i << "):" << config.passStateVector.at(i) << endl;
	}
	return out;
}

bool operator<(const PassConfig& lhs, const PassConfig& rhs){
	return (lhs.weight < rhs.weight);
}
