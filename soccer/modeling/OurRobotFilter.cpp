#include "OurRobotFilter.hpp"

OutRobotFilter::OutRobotFilter() {}

void OutRobotFilter::createConfiguration(Configuration* cfg) {
    // TODO: Add all the config variables
    // Thinking just camera_frame_delay for right now
}

void OutRobotFilter::update(const std::array<RobotObservation, Num_Cameras>& obs, RobotPose* robot, RJ::Time currentTime, u_int32_t frameNumber) {
    filt.update(obs, robot, currentTime, frameNumber);

    // Calculate 1 frame for the new camera
    // Calculate N frames for the encoders
}

void OutRobotFilter::calculateOneFrame(Eigen::Vector& x_hat_k1_k1, Eigen::Vector& p_k1_k1, 
                                       Eigen::Vector* x_hat_k_k, Eigen::Vector* p_k_k, 
                                       FilterMode mode) {
    // Rotate F_k
    // x_hat_k_k1 = F_k * x_hat_k1_k1 + B_K * u
    // P_k_k1 = F_k * p_k1_k1 * F_k' + Q
    // y_tilda_k = z_k - H_k * x_hat_k_k1
    // S_k = R_k + H_k * P_k_k1 * H_k'
    // K_k = P_k_k1 * H_k * S_k^-1
    // x_hat_k_k = x_hat_k_k1 + K_k * y_tilda_k
    // P_k_k = (I - K_k * H_k) * P_k_k1 * (I - K_k * H_k)' + K_k * R_k * K_k'
    // y_tilda_k_k = z_k - H_k * x_hat_k_k
}

void OutRobotFilter::calculateRotationMat(double heading, Eigen::Matrix* rot) {
    // cos -sin 0
    // sin  cos 0
    //   0    0 1
}