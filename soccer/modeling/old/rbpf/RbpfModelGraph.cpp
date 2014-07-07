/*
 * RbpfModelGraph.cpp
 *
 *  See: RbpfModelGraph.hpp for additional information.
 *
 *  Author: Philip Rogers, Nov 9th 2009
 */

#include "RbpfModelGraph.hpp"

using namespace LinAlg;

// initializes the number of models in the graph (j) to zero.
// note that modelVector and adjacencyMatrix are already empty.
RbpfModelGraph::RbpfModelGraph() : j(0) { }

RbpfModelGraph::~RbpfModelGraph() {
	// modelVector will be automatically freed
}

// Adds a node, resizes the adjacency matrix (containing transition
// probabilities) to account for the new edges, and explicitly sets all
// transition probabilities to zero.
void RbpfModelGraph::addModel(RbpfModel* model){
	modelVector.push_back(model); // add the node
	adjacencyMatrix.resize(j+1,j+1); // resize transition probabilities
	for(int i=0; i<j+1; i++){ // ensure new transition probabilities are zero
		adjacencyMatrix(i,j) = adjacencyMatrix(j,i) = 0.0;
	}
	j++; // increment number of nodes
}

// returns a model from the graph that corresponds to index modelIndex
// note: An exception will be thrown if modelIndex is not in the models vector
RbpfModel* RbpfModelGraph::getModel(int modelIndex){
	if(modelIndex<0 || modelIndex>=j){
		throw std::runtime_error("Attempted to access invalid model in RbpfModelGraph");
	}else{
		return &modelVector[modelIndex];
	}
}

// get the transition probability from model A to model B.
double RbpfModelGraph::getTransProb(int indexA, int indexB){
	assert(indexA >= 0 && indexA < j); // index of model A must be valid
	assert(indexB >= 0 && indexB < j); // index of model B must be valid
	return adjacencyMatrix(indexA, indexB);
}

// sets the transition probability from model A to model B. There is no
// restriction that A != B, because edges from a node and to itself are valid.
// note that the weight is a probability, and must lie between 0 and 1.
// This does not check that the total weight leaving a node sums to 1.
void RbpfModelGraph::setTransProb(int indexA, int indexB, double weight){
	assert(indexA >= 0 && indexA < j); // index of model A must be valid
	assert(indexB >= 0 && indexB < j); // index of model B must be valid
	assert(weight >= 0.0 && weight <= 1.0); // weight must be in [0,1]
	adjacencyMatrix(indexA, indexB) = weight;
}

// Used for printing this modelGraph to a stream
// displays each model and the adjacency matrix
std::ostream& operator<<(std::ostream& out, const RbpfModelGraph &g){
	// display each model (node)
	for(int i=0; i<g.j; i++){
		out << "Model(" << i << "):" << std::endl << g.modelVector[i] << std::endl;
	}
	// display each transition probability (directed, weighted edge)
	out << "Adjacency Matrix:" << std::endl;
	for(int x=0; x<g.j; x++){
		for(int y=0; y<g.j; y++)
			out << g.adjacencyMatrix(x,y) << ", ";
		out << std::endl;
	}
	return out;
}
