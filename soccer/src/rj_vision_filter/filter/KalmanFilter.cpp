#include <rj_vision_filter/filter/KalmanFilter.hpp>

namespace vision_filter {
void KalmanFilter::predict() {
    x_k1_k1_ = x_k_k_;
    P_k1_k1_ = P_k_k_;

    // Predict
    x_k_k1_ = F_k_ * x_k1_k1_ + B_k_ * u_k_;
    P_k_k1_ = F_k_ * P_k1_k1_ * F_k_.transpose() + Q_k_;

    x_k_k_ = x_k_k1_;
    P_k_k_ = P_k_k1_;
}

void KalmanFilter::predict_with_update() {
    x_k1_k1_ = x_k_k_;
    P_k1_k1_ = P_k_k_;

    // Predict
    x_k_k1_ = F_k_ * x_k1_k1_ + B_k_ * u_k_;
    P_k_k1_ = F_k_ * P_k1_k1_ * F_k_.transpose() + Q_k_;

    // Update
    y_k_k1_ = z_k_ - H_k_ * x_k_k1_;

    S_k_ = R_k_ + H_k_ * P_k_k1_ * H_k_.transpose();
    K_k_ = P_k_k1_ * H_k_.transpose() * S_k_.inverse();

    x_k_k_ = x_k_k1_ + K_k_ * y_k_k1_;
    P_k_k_ = (identity_ - K_k_ * H_k_) * P_k_k1_ * (identity_ - K_k_ * H_k_).transpose() +
             K_k_ * R_k_ * K_k_.transpose();

    y_k_k_ = z_k_ - H_k_ * x_k_k_;
}
}  // namespace vision_filter