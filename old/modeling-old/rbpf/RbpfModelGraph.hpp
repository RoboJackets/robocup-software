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
#include <boost/ptr_container/ptr_vector.hpp>
#include <LinearAlgebra.hpp>
#include "RbpfModel.hpp"

class RbpfModelGraph {
public:
    typedef boost::ptr_vector<RbpfModel> ModelVector;

    RbpfModelGraph();

    ~RbpfModelGraph();

    // adds a model as a node in the graph and initializes transition
    // probabilities to zero.
    void addModel(RbpfModel* model);

    // returns a model from the graph that corresponds to index modelIndex
    RbpfModel* getModel(int modelIndex);

    // sets the transition probability from model A to model B
    void setTransProb(int indexA, int indexB, double weight);

    // get the transition probability from model A to model B
    double getTransProb(int indexA, int indexB);

    // Used for printing this modelGraph to a stream
    friend std::ostream& operator<<(std::ostream& out,
                                    const RbpfModelGraph& graph);

    int j;  // number of models in graph

protected:
    LinAlg::Matrix adjacencyMatrix;
    ModelVector modelVector;
};

#endif /* RBPFMODELGRAPH_HPP_ */
