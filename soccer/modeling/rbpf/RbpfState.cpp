/**
 * RbpfState.cpp
 *
 *  See: RbpfState.hpp for additional information
 *
 *  Author: Philip Rogers, Nov 10th 2009
 */

#include <iostream>
#include "RbpfState.hpp"

using namespace rbpf;
using namespace std;

const bool verbose = false;

// overloaded operator for printing a RbpfState
// displays the X (state), P (state covariance), and the weight
std::ostream& operator<<(std::ostream& out, const RbpfState &state){
	out << "X: \n" << state.X.transpose() << std::endl;
	out << "P: \n" << state.P << std::endl;
	out << "weight: " << state.weight << std::endl;
	out << "modelIdx: " << state.modelIdx << std::endl;
	return out;
}

RbpfState::RbpfState(const rbpf::VectorNd& _X, const rbpf::MatrixNNd& _P, int _modelIdx, double _w):
	X(_X), P(_P), modelIdx(_modelIdx), weight(_w)
{
	if (verbose) {
		cout << "Creating RbpfState - modelIdx: " << modelIdx << " weight: " << weight << "\n";
		cout << "X: \n" << X << "\n";
		cout << "P: \n" << P << endl;
	}
}
