/*
 * RbpfState.cpp
 *
 *  See: RbpfState.hpp for additional information
 *
 *  Author: Philip Rogers, Nov 10th 2009
 */

#include <RbpfState.hpp>

// Constructor: RbpfState(_X, _P, _model, _w)
//   initializes X (state), P (state covariance),
//   the model index, the weight, and n (size of state)
//   note that X (n x 1) and P (n x n) must have the correct sizes.
RbpfState::RbpfState(Vector _X, Matrix _P, int _modelIdx, double _w) : X(_X), P(_P), modelIdx(_modelIdx), weight(_w){
	assert(X.size() == P.size1()); // P must be of size (n x n)
	assert(X.size() == P.size2()); // P must be of size (n x n)
	n = X.size();                  // size of Kalman Filter state
}

RbpfState::~RbpfState() {}

// Function copy(RbpfState &state)
//   Copies one state into another. Note: this is not a copy constructor.
void RbpfState::copy(const RbpfState &state){
	n = state.n;
	X.assign(state.X);
	P.assign(state.P);
	modelIdx = state.modelIdx;
	weight = state.weight;
}

// Function: opterator<<
//   overloaded operator for printing a RbpfState
//   displays the X (state), P (state covariance), and the weight
ostream& operator<<(ostream& out, const RbpfState &state){
	// display X
	out << "X:" << endl;
	for(int x=0; x<state.n; x++)
		out << "\t" << state.X(x) << endl;
	// display P
	out << endl << "P:" << endl;
	for(int x=0; x<state.n; x++){
		out << "\t";
		for(int y=0; y<state.n; y++)
			out << state.P(x,y) << ", ";
		out << endl;
	}
	// display weight
	out << "weight:" << state.weight << endl;
	return out;
}
