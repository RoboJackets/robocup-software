/*
 * RbpfState.cpp
 *
 *  See: RbpfState.hpp for additional information
 *
 *  Author: Philip Rogers, Nov 10th 2009
 */

#include <modeling/RbpfState.hpp>

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
