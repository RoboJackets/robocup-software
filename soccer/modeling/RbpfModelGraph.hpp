/*
 * RbpfModelGraph.hpp
 *
 *  Represents a directed graph with weighted transitions.
 *  Each node in the graph is a RbpfParticleModel, which contains the
 *  transition model for a Kalman filter. The graph's edges represent
 *  transition probabilities.
 *
 *  To build the graph, add all the nodes using addModel() then set the
 *  transition probabilities using setTransProb().
 *
 *  Author: Philip Rogers, Nov 8th 2009
 */

#ifndef RBPFMODELGRAPH_HPP_
#define RBPFMODELGRAPH_HPP_

#include <iostream>
#include <fstream>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include "RbpfModel.hpp"
using std::ostream;
using std::endl;
typedef boost::numeric::ublas::matrix<double> Matrix;
typedef boost::ptr_vector<RbpfModel> ModelVector;

class RbpfModelGraph {
public:
	// Constructor: RbpfModelGraph()
	RbpfModelGraph();

	// Destructor: ~RbpfModelGraph()
	~RbpfModelGraph();

	// Function: addModel(RbpfModel* model)
	//   adds a model as a node in the graph and initializes transition
	//   probabilities to zero.
	void addModel(RbpfModel* model);

	// Function: getModel(int modelIndex)
	//   returns a model from the graph that corresponds to index modelIndex
	RbpfModel* getModel(int modelIndex);

	// Function: setTransProb(int A, int B, double weight)
	//   sets the transition probability from model A to model B
	void setTransProb(int indexA, int indexB, double weight);

	// Function: getTransProb(int A, int B)
	//   get the transition probability from model A to model B
	double getTransProb(int indexA, int indexB);

	// Operator: <<
	//   Used for printing this modelGraph to a stream
	friend ostream& operator<<(ostream& out, const RbpfModelGraph &graph);


	// public variables
	int j; // number of models in graph

protected:
	// protected variables
	Matrix adjacencyMatrix;
	ModelVector modelVector;
};

#endif /* RBPFMODELGRAPH_HPP_ */
