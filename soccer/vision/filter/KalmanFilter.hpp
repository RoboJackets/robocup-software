#pragma once

#include <Eigen/Dense>

/**
 * Abstract class that should be inherited from to create specific
 * kalman filters
 *
 * Must initialize:
 *  x_k1_k1, x_k_k1, x_k_k,
 *  P_k1_k1, P_k_k1, P_k_k,
 *  F_k, B_k, H_k,
 *  Q_k, R_k
 *
 * Every predict, must update:
 *  u_k
 *
 * Every PredictWithUpdate, must update:
 *  u_k, z_k
 *
 * The most recently updated values
 *  x_k_k, P_k_k
 *
 * Taken from https://en.wikipedia.org/wiki/Kalman_filter
 *
 * Conversion between code notation and wiki notation is...
 * x_k1_k1 is X_(k-1, k-1)
 * x_k_k is X_(k, k)
 * etc
 */
class KalmanFilter {
public:
    /**
     * Creates a general kalman filter with the given sizes
     * Use a child class to setup the specific state matricies
     * Assumes 1 input
     *
     * @param stateSize The size of the state vector
     * @param observationSize The size of the observation vector
     */
    KalmanFilter(unsigned int stateSize, unsigned int observationSize) :
        x_k1_k1(stateSize), x_k_k1(stateSize), x_k_k(stateSize),
        u_k(1), z_k(observationSize),
        y_k_k1(observationSize), y_k_k(observationSize),
        P_k1_k1(stateSize, stateSize), P_k_k1(stateSize, stateSize), P_k_k(stateSize, stateSize),
        S_k(observationSize, observationSize), K_k(stateSize, observationSize),
        F_k(stateSize, stateSize), B_k(stateSize, 1), H_k(observationSize, stateSize),
        Q_k(stateSize, stateSize), R_k(observationSize, observationSize),
        I(Eigen::MatrixXd::Identity(stateSize, stateSize)) {}

    /**
     * Predicts without update
     */
    void predict();

    /**
     * Predicts with update
     * z_k must be set with the observation
     */
    void predictWithUpdate();

protected:
    Eigen::VectorXd x_k1_k1;
    Eigen::VectorXd x_k_k1;
    Eigen::VectorXd x_k_k;

    Eigen::VectorXd u_k;
    Eigen::VectorXd z_k;

    Eigen::VectorXd y_k_k1;
    Eigen::VectorXd y_k_k;

    Eigen::MatrixXd P_k1_k1;
    Eigen::MatrixXd P_k_k1;
    Eigen::MatrixXd P_k_k;

    Eigen::MatrixXd S_k;
    Eigen::MatrixXd K_k;

    Eigen::MatrixXd F_k;
    Eigen::MatrixXd B_k;
    Eigen::MatrixXd H_k;

    Eigen::MatrixXd Q_k;
    Eigen::MatrixXd R_k;

    Eigen::MatrixXd I;
};