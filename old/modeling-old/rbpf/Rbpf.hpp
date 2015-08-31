/*
 * Rbpf.hpp
 *
 *  Rao Blackwellized Particle Filter (RBPF)
 *  We use the method of [1] where a RBPF is used to track a ball
 *  using a particle filter over the non-linear portions of
 *  the state (the models; e.g., was ball kicked?, is ball rolling?) and an
 *  Extended Kalman Filter (EKF) over the 'linear' portions (ball dynamics.)
 *
 *  This differs slightly from a traditional RBPF because the particles are
 *  over a discrete state space (the model graph).
 *
 *  For a visual understanding of the model graph, see [1], pg.4, fig.1
 *  The algorithm here follows the algorithm in [1], pg.8, table.1 with the
 *  exception that there is no particle filter over the robot positions.
 *
 *  How to use:
 *  To create a Rbpf, you'll need to instantiate this class and determine the
 *  structure of the discrete space (the model graph). Create models and add
 *  them to the internal modelGraph using addModel(), and set the transition
 *  probabilities using setTransProb(A,B,prob).  Note that the initial state
 *  and covariance sizes you choose when creating the Rbpf should correspond to
 *  the state sizes used in your model graph.
 *
 *  Note:
 *    When possible the notation here is based on:
 *      http://en.wikipedia.org/wiki/Kalman_filter
 *      http://en.wikipedia.org/wiki/Extended_Kalman_filter
 *      http://en.wikipedia.org/wiki/Particle_filter
 *      [1]: Map-Based Multiple Model Tracking of a Moving Object
 *           Authors: Cody Kwok and Dieter Fox.
 *           DOI: 10.1007/b106671
 *
 *  Author: Philip Rogers, Nov 19th 2009
 */

#ifndef RBPF_HPP_
#define RBPF_HPP_

#include <stdlib.h>
#include <iostream>
#include <boost/ptr_container/ptr_vector.hpp>
#include <LinearAlgebra.hpp>
#include "RbpfState.hpp"
#include "RbpfModel.hpp"
#include "RbpfModelGraph.hpp"

class Rbpf {
public:
    typedef boost::ptr_vector<RbpfState> ParticleVector;

    // X: initial state, (n x 1)
    // P: initial state covariance, (n x n)
    // k: the number of particles to be initialized.
    // Where n = size of Kalman Filter state
    Rbpf(const rbpf::VectorNd& _X, const rbpf::MatrixNNd& _P, size_t _k);

    ~Rbpf();

    // updates the filter with 2d observation [x;y], change in time dt, and no
    // control input
    void update(double x, double y, double dt);

    // updates the filter for observation Z, control input U, and change in time
    // dt
    // U: control input, (m x 1)
    // Z: measurement, (s x 1)
    // dt: change in time
    // Where m and s = size of the control and measurement input
    void update(const rbpf::VectorNd& U, const rbpf::VectorSd& Z, double dt);

    // reinitialize the parameters from the config files - should be called each
    // frame
    void initParams();

    void updateMultipleObs(double x[], double y[], double dt[], int numObs);

    // adds a model to the internal modelGraph
    void addModel(RbpfModel* model);

    // sets a transition probability in the modelGraph from model A to model B
    void setTransProb(int AIdx, int BIdx, double weight);

    // returns a pointer to the best particle state
    RbpfState* getBestFilterState();

    //   Used for printing a Rbpf to a stream
    friend std::ostream& operator<<(std::ostream& out, const Rbpf& rbpf);

    int k;                      // number of particles
    RbpfModelGraph modelGraph;  // Graph of models
    int n;                      // size of state vector

protected:
    ParticleVector particleVector;     // vector of particles, (k x 1)
    ParticleVector tmpParticleVector;  // vector of temp particles, (k*j x 1)

    // Evaluates the multivariate PDF of a centered (mean=0,0) 2D Gaussian dist.
    // X: point to be evaluated (2 x 1)
    // Sigma: covariance matrix (2 x 2)
    inline double gaussianPDF2D(const rbpf::VectorSd& X,
                                const rbpf::MatrixSSd& Sigma);

    // resample k particles from out, with respect to their weights
    // NOTE: in vector has its weight changed
    void resampleParticles(ParticleVector& in, ParticleVector& out, int);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* RBPF_HPP_ */
