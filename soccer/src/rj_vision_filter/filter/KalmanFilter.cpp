#include <rj_vision_filter/filter/KalmanFilter.hpp>

namespace vision_filter {
void KalmanFilter::predict() {
    x_k1_k1 = x_k_k;
    P_k1_k1 = P_k_k;

    // Predict
    x_k_k1 = F_k * x_k1_k1 + B_k * u_k;
    P_k_k1 = F_k * P_k1_k1 * F_k.transpose() + Q_k;

    x_k_k = x_k_k1;
    P_k_k = P_k_k1;
}

void KalmanFilter::predictWithUpdate() {
    x_k1_k1 = x_k_k;
    P_k1_k1 = P_k_k;

    // Predict
    x_k_k1 = F_k * x_k1_k1 + B_k * u_k;
    P_k_k1 = F_k * P_k1_k1 * F_k.transpose() + Q_k;

    // Update
    y_k_k1 = z_k - H_k * x_k_k1;

    S_k = R_k + H_k * P_k_k1 * H_k.transpose();
    K_k = P_k_k1 * H_k.transpose() * S_k.inverse();

    x_k_k = x_k_k1 + K_k * y_k_k1;
    P_k_k = (I - K_k * H_k) * P_k_k1 * (I - K_k * H_k).transpose() +
            K_k * R_k * K_k.transpose();

    y_k_k = z_k - H_k * x_k_k;
}
}  // namespace vision_filter