#pragma once

#include <Eigen/Dense>

namespace vision_filter {
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
     * @param state_size The size of the state vector
     * @param observation_size The size of the observation vector
     */
    KalmanFilter(unsigned int state_size, unsigned int observation_size)
        : x_k1_k1_(state_size),
          x_k_k1_(state_size),
          x_k_k_(state_size),
          u_k_(1),
          z_k_(observation_size),
          y_k_k1_(observation_size),
          y_k_k_(observation_size),
          P_k1_k1_(state_size, state_size),
          P_k_k1_(state_size, state_size),
          P_k_k_(state_size, state_size),
          S_k_(observation_size, observation_size),
          K_k_(state_size, observation_size),
          F_k_(state_size, state_size),
          B_k_(state_size, 1),
          H_k_(observation_size, state_size),
          Q_k_(state_size, state_size),
          R_k_(observation_size, observation_size),
          identity_(Eigen::MatrixXd::Identity(state_size, state_size)) {}

    /**
     * Predicts without update
     */
    void predict();

    /**
     * Predicts with update
     * z_k must be set with the observation
     */
    void predict_with_update();

protected:
    Eigen::VectorXd x_k1_k1_;
    Eigen::VectorXd x_k_k1_;
    Eigen::VectorXd x_k_k_;

    Eigen::VectorXd u_k_;
    Eigen::VectorXd z_k_;

    Eigen::VectorXd y_k_k1_;
    Eigen::VectorXd y_k_k_;

    Eigen::MatrixXd P_k1_k1_;
    Eigen::MatrixXd P_k_k1_;
    Eigen::MatrixXd P_k_k_;

    Eigen::MatrixXd S_k_;
    Eigen::MatrixXd K_k_;

    Eigen::MatrixXd F_k_;
    Eigen::MatrixXd B_k_;
    Eigen::MatrixXd H_k_;

    Eigen::MatrixXd Q_k_;
    Eigen::MatrixXd R_k_;

    Eigen::MatrixXd identity_;
};
}  // namespace vision_filter