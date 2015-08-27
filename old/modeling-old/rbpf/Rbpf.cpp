/**
 * Rbpf.cpp
 *
 *  See: Rbpf.hpp for additional information.
 *
 *  Author: Philip Rogers, Nov 15th 2009
 */

#include <iostream>
#include "Rbpf.hpp"
#include <math.h>

using namespace std;
using namespace LinAlg;
using namespace rbpf;

// Constructor: Rbpf(X, P, k)
//   X: initial state, (n x 1)
//   P: initial state covariance, (n x n)
//   k: the number of particles to be initialized.
// initializes the particle vector, with k particle states
Rbpf::Rbpf(const VectorNd& X, const MatrixNNd& P, size_t _k)
    : k(_k), modelGraph() {
    assert(k > 0);  // Must have at least 1 particle state

    // initialize particles
    // because there are no models in the model graph, the initial model used
    // here will cause an exception to be thrown if update is called before
    // at least 1 model is added.
    int initialModelIdx = 0;  // initialize all particles with first model
    for (int i = 0; i < k; i++) {
        particleVector.push_back(new RbpfState(X, P, initialModelIdx, 0.0));
    }
}

Rbpf::~Rbpf() {}

// convenience function for calling update(U,Z,dt) with no control input
// note: assumes that control size (m) = 2 and measurement size (s) = 2
void Rbpf::update(double x, double y, double dt) {
    VectorMd U = VectorMd::Zero();  // control input
    VectorSd Z;
    Z << x, y;  // measurement
    update(U, Z, dt);
}

// Updates the filter given control input U, measurement Z, and delta t = dt
// Closely follows the algorithm in [1], pg.8, table.1.
// Brief description: For each particle, assume it is in each state, and
//                    update based on that assumption. If the resulting
//                    probability is good, that assumption must have been true
//                    and so we will end up resampling that particle.
void Rbpf::update(const VectorMd& U, const VectorSd& Z, double dt) {
    int j = modelGraph.j;  // number of models in modelGraph
    int tmpPartIdx = 0;
    float weightSum;
    RbpfModel* model;
    RbpfState* tmpParticle;

    for (int kIdx = 0; kIdx < k; kIdx++) {  // for each of the k particles
        weightSum = 0.0;
        for (int jIdx = 0; jIdx < j; jIdx++) {  // for each of the j models
            model = modelGraph.getModel(jIdx);
            model->initializeQ();  // get latest params from config
            model->initializeR();
            assert(tmpPartIdx < (int)tmpParticleVector.size());
            tmpParticle = &tmpParticleVector[tmpPartIdx];
            tmpParticle->copy(particleVector[kIdx]);
            model->computeJacobians(
                dt);  // update jacobians *before* predict/update
            model->predict(tmpParticle->X, tmpParticle->P, U,
                           dt);  // EKF Predict
            model->update(tmpParticle->X, tmpParticle->P, Z, dt);  // EKF Update

            // calculate probability of switching from previous model to this
            // (jIdx)
            double probModel =
                modelGraph.getTransProb(tmpParticle->modelIdx, jIdx);
            // calculate probability of observation
            double probObs = gaussianPDF2D(model->getInnovation(),
                                           model->getInnovationCovariance());

            weightSum += probModel * probObs;
            tmpParticle->weight = probModel * probObs;
            tmpParticle->modelIdx = jIdx;
            tmpPartIdx++;
        }
        for (int jIdx = 0; jIdx < j; jIdx++) {  // for each of the j models
            tmpParticle = &tmpParticleVector[tmpPartIdx - jIdx - 1];
            tmpParticle->weight *= weightSum;  // multiply the weightSum in for
                                               // particles with this model
        }
    }
    resampleParticles(tmpParticleVector, particleVector, k);
}

// Updates the filter given control input U, measurement Z, and delta t = dt
// Note: this is for multiple observations (possibly from multiple cameras at
// diff times)
// see: void Rbpf::update(Vector &U, Vector &Z, double dt) for more details
void Rbpf::updateMultipleObs(double xs[], double ys[], double dts[],
                             int numObs) {
    tmpParticleVector.clear();

    int j = modelGraph.j;  // number of models in modelGraph
    int tmpPartIdx = 0;
    float weightSum;
    RbpfModel* model;
    RbpfState* tmpParticle;
    VectorNd U = VectorMd::Zero();  // control input
    VectorSd Z = VectorSd::Zero();  // measurement
    double dt;

    for (int kIdx = 0; kIdx < k; kIdx++) {  // for each of the k particles
        for (int oIdx = 0; oIdx < numObs;
             oIdx++) {  // for each of the o observations
            Z(0) = xs[oIdx];
            Z(1) = ys[oIdx];
            dt = dts[oIdx];

            weightSum = 0.0;
            for (int jIdx = 0; jIdx < j; jIdx++) {  // for each of the j models
                model = modelGraph.getModel(jIdx);
                model->initializeQ();  // get latest params from config
                model->initializeR();
                VectorNd X;
                X.setZero();
                MatrixNNd P;
                P.setZero();
                tmpParticleVector.push_back(new RbpfState(X, P, 0, 0.0));
                assert(tmpPartIdx < (int)tmpParticleVector.size());
                tmpParticle = &tmpParticleVector[tmpPartIdx];
                tmpParticle->copy(particleVector[kIdx]);
                model->predict(tmpParticle->X, tmpParticle->P, U,
                               dt);  // EKF Predict
                model->update(tmpParticle->X, tmpParticle->P, Z,
                              dt);  // EKF Update

                // calculate probability of switching from previous model to
                // this (jIdx)
                double probModel =
                    modelGraph.getTransProb(tmpParticle->modelIdx, jIdx);
                // calculate probability of observation
                double probObs = gaussianPDF2D(
                    model->getInnovation(), model->getInnovationCovariance());

                weightSum += probModel * probObs;
                tmpParticle->weight = probModel * probObs;
                tmpParticle->modelIdx = jIdx;
                tmpPartIdx++;
            }
            for (int jIdx = 0; jIdx < j; jIdx++) {  // for each of the j models
                tmpParticle = &tmpParticleVector[tmpPartIdx - jIdx - 1];
                tmpParticle->weight *= weightSum;  // multiply the weightSum in
                                                   // for particles with this
                                                   // model
            }
        }
    }
    resampleParticles(tmpParticleVector, particleVector, k);
}

// resample k particles from in, with respect to their weights
// store the result in out.
// TODO: switch the Vectors to dynamic double arrays, or at least pre-allocate.
void Rbpf::resampleParticles(ParticleVector& in, ParticleVector& out,
                             const int k) {
    double NThresh = k;  // effective number of particles threshold
    int kIn = in.size();
    double oneOverK = 1.0 / ((double)k);
    Vector intervals(k), UIntervals(k), cumulativeWeights(kIn);
    int* resampleIndex = new int[k];

    // generate k uniform subintervals of (0,1]
    for (int i = 0; i < k; i++) {
        intervals[i] = i * oneOverK;
    }
    // draw a sample from each interval
    for (int i = 0; i < k; i++) {
        UIntervals[i] =
            intervals[i] + oneOverK * (rand() / ((double)RAND_MAX + 1));
    }
    // normalize particle weights
    double weightSum = 0, Neff = 0;
    for (int i = 0; i < kIn; i++) {
        weightSum += in[i].weight;
    }
    for (int i = 0; i < kIn; i++) {
        if (weightSum == 0) {
            in[i].weight = oneOverK;
        } else {
            in[i].weight /= weightSum;
        }
        Neff += (in[i].weight) * (in[i].weight);
    }
    Neff = 1.0 / Neff;
    // calculate cumulative sum of the weights
    weightSum = 0;
    for (int i = 0; i < kIn; i++) {
        weightSum += in[i].weight;
        cumulativeWeights[i] = weightSum;
    }
    if (Neff < NThresh || k != kIn) {
        // select the weight index which is responsible for each U value of
        // the cumulative weight.
        for (int i = 0; i < k; i++) {
            int j = 0;
            while (cumulativeWeights[j] < UIntervals[i]) {
                j++;
            }
            resampleIndex[i] = j;
        }
    } else {
        for (int i = 0; i < k; i++) {
            resampleIndex[i] = i;
        }
    }
    // resample
    for (int i = 0; i < k; i++) {
        out[i].copy(in[resampleIndex[i]]);
        out[i].weight = oneOverK;
    }
    delete[] resampleIndex;
}

// returns a pointer to the best particle state
RbpfState* Rbpf::getBestFilterState() {
    int bestIdx = 0;
    double bestWeight = -1.0;
    for (int i = 0; i < k; i++) {
        if (particleVector[i].weight > bestWeight) {
            bestWeight = particleVector[i].weight;
            bestIdx = i;
        }
    }
    return &(particleVector[bestIdx]);
}

// adds a model to the modelGraph and resizes the tmpParticleVector
void Rbpf::addModel(RbpfModel* model) {
    modelGraph.addModel(model);
    // tmpParticleVector is assumed to contain j*k elements, so for each model
    // that we add (j=j+1), we add k new particles to the tmp vector.
    // Each particle in tmpParticleVector simply contains space for temporary
    // particles during the update step.  The values in each particle will be
    // overwritten at each iteration, so the initial values do not matter.
    int modelIdx = 0;
    VectorNd X;
    X.setZero();
    MatrixNNd P;
    P.setZero();
    for (int i = 0; i < k; i++)
        tmpParticleVector.push_back(new RbpfState(X, P, modelIdx, 0.0));
}

// sets a transition probability in the modelGraph from model A to model B with
// a given weight.
void Rbpf::setTransProb(int AIdx, int BIdx, double weight) {
    modelGraph.setTransProb(AIdx, BIdx, weight);
}

// Evaluates the multivariate PDF of a centered (mean=0,0) 2D Gaussian
// distribution.
// X = (2 x 1), Sigma = (2 x 2)
// This function does not appear to be provided by Boost (yet).  This is
// explicitly for the 2D case because inverting Sigma (in the general multi-
// variate case) is expensive.
// Compared against Matlab's mvnpdf() with several tests all passed.
inline double Rbpf::gaussianPDF2D(const rbpf::VectorSd& X,
                                  const rbpf::MatrixSSd& Sigma) {
    double sX2 = Sigma(0, 0), sY2 = Sigma(1, 1);
    double sX = sqrt(sX2), sY = sqrt(sY2);
    double p = Sigma(0, 1) / (sX * sY);  // just take p(1), assume Pos.Semi.Def.
    double x = X(0), y = X(1);
    double expTerm = (-0.5 / (1 - p * p)) * ((x * x / sX2) + (y * y / sY2) -
                                             (2 * p * x * y / (sX * sY)));
    return (0.5 / (M_PI * sX * sY * sqrt(1 - p * p))) * std::exp(expTerm);
}

// Used for printing this state to a stream
// displays the modelGraph and each particle in the filter
std::ostream& operator<<(std::ostream& out, const Rbpf& r) {
    // display model graph
    out << "model graph:" << std::endl << r.modelGraph << std::endl;
    // display k particles
    for (int x = 0; x < r.k; x++)
        out << "particle(" << x << "):" << std::endl << r.particleVector[x]
            << std::endl;
    return out;
}
