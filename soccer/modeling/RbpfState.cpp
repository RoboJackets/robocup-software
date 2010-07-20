/*
 * RbpfState.cpp
 *
 *  See: RbpfState.hpp for additional information
 *
 *  Author: Philip Rogers, Nov 10th 2009
 */

#include <modeling/RbpfState.hpp>

// initializes X (state), P (state covariance),
// the model index, the weight, and n (size of state)
// note that X (n x 1) and P (n x n) must have the correct sizes.
RbpfState::RbpfState(Vector _X, Matrix _P, int _modelIdx, double _w) : X(_X), P(_P), modelIdx(_modelIdx), weight(_w){
	assert(X.size() == P.size1()); // P must be of size (n x n)
	assert(X.size() == P.size2()); // P must be of size (n x n)
	n = X.size();                  // size of Kalman Filter state
}

RbpfState::~RbpfState() {}

// Copies one state into another. Note: this is not a copy constructor.
void RbpfState::copy(const RbpfState &state){
	n = state.n;
	X.assign(state.X);
	P.assign(state.P);
	modelIdx = state.modelIdx;
	weight = state.weight;
}

// overloaded operator for printing a RbpfState
// displays the X (state), P (state covariance), and the weight
std::ostream& operator<<(std::ostream& out, const RbpfState &state){
	// display X
	out << "X:" << std::endl;
	for(int x=0; x<state.n; x++)
		out << "\t" << state.X(x) << std::endl;
	// display P
	out << std::endl << "P:" << std::endl;
	for(int x=0; x<state.n; x++){
		out << "\t";
		for(int y=0; y<state.n; y++)
			out << state.P(x,y) << ", ";
		out << std::endl;
	}
	// display weight
	out << "weight:" << state.weight << std::endl;
	return out;
}
