#include <cmath>

#include <rj_vision_filter/filter/KalmanFilter2D.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {

DEFINE_NS_FLOAT64(kVisionFilterParamModule, ball, init_covariance, 100.0,
                  "Initial covariance of the filter. Controls how fast it gets "
                  "to the target.")
DEFINE_NS_FLOAT64(kVisionFilterParamModule, ball, process_noise, 0.1,
                  "Controls how quickly it reacts to changes in ball accelerations.")
DEFINE_NS_FLOAT64(kVisionFilterParamModule, ball, observation_noise, 2.0,
                  "Controls how much it trusts measurements from the camera.")

KalmanFilter2D::KalmanFilter2D() : KalmanFilter(1, 1) {}

KalmanFilter2D::KalmanFilter2D(Geometry2d::Point init_pos, Geometry2d::Point init_vel)
    : KalmanFilter(4, 2) {
    // clang-format off
    // States are X pos, X vel, Y pos, Y vel
    x_k1_k1_ << init_pos.x(),
                init_vel.x(),
                init_pos.y(),
                init_vel.y();
    x_k_k1_ = x_k1_k1_;
    x_k_k_ = x_k1_k1_;

    // Initial covariance is usually extremely high to converge to the true
    // solution
    double p = ball::PARAM_init_covariance;
    P_k1_k1_ << p, 0, 0, 0,
                0, p, 0, 0,
                0, 0, p, 0,
                0, 0, 0, p;
    P_k_k1_ = P_k1_k1_;
    P_k_k_ = P_k1_k1_;

    // TODO(1565): Allow variable dt to decouple predict, update and merging.
    // State transition matrix (A)
    // Pos, velocity integrator. Assume constant velocity
    double dt = PARAM_vision_loop_dt;
    F_k_ << 1, dt,  0,  0,
            0,  1,  0,  0,
            0,  0,  1, dt,
            0,  0,  0,  1;

    // Control transition matrix (B)
    // No inputs
    B_k_ << 0,
            0,
            0,
            0;

    // Observation Matrix (C)
    // We can get positions
    H_k_ << 1, 0, 0, 0,
            0, 0, 1, 0;
    // clang-format on

    // Covariance of process noise (how wrong A is)
    // Based on a guassian white noise w_k in x_dot = A*x + B*u + G*w.
    // The noise can be propogated through the model resulting in a process
    // noise of the form
    //
    //  [1/3 T^3     1/2 T^2] * sigma^2
    //  [1/2 T^2           T]
    // Where sigma is the standard deviation of the process noise
    // the change in velocity over one time step should be around sqrt(T *
    // sigma^2).
    // Note: T is the sample period.
    // Taken from Tiger's AutoRef. Most likely found through integration of
    // error through the state matrices. See
    // https://en.wikipedia.org/wiki/Discretization#Discretization_of_process_noise
    p = ball::PARAM_process_noise;
    double sigma = sqrt(3.0 * p / dt) / dt;
    double dt3 = 1.0 / 3.0 * dt * dt * dt * sigma * sigma;
    double dt2 = 1.0 / 2.0 * dt * dt * sigma * sigma;
    double dt1 = dt * sigma * sigma;

    // clang-format off
    Q_k_ << dt3, dt2,   0,   0,
            dt2, dt1,   0,   0,
              0,   0, dt3, dt2,
              0,   0, dt2, dt1;
    // clang-format on

    // Covariance of observation noise (how wrong z_k is)
    double o = ball::PARAM_observation_noise;
    // clang-format off
    R_k_ << o, 0,
            0, o;
    // clang-format on
}

void KalmanFilter2D::predict_with_update(Geometry2d::Point observation) {
    z_k_ << observation.x(), observation.y();

    KalmanFilter::predict_with_update();
}

Geometry2d::Point KalmanFilter2D::get_pos() const {
    return Geometry2d::Point(x_k_k_(0), x_k_k_(2));
}

Geometry2d::Point KalmanFilter2D::get_vel() const {
    return Geometry2d::Point(x_k_k_(1), x_k_k_(3));
}

Geometry2d::Point KalmanFilter2D::get_pos_cov() const {
    return Geometry2d::Point(P_k_k_(0, 0), P_k_k_(2, 2));
}

Geometry2d::Point KalmanFilter2D::get_vel_cov() const {
    return Geometry2d::Point(P_k_k_(1, 1), P_k_k_(3, 3));
}

void KalmanFilter2D::set_vel(Geometry2d::Point new_vel) {
    x_k_k_(1) = new_vel.x();
    x_k_k_(3) = new_vel.y();
}
}  // namespace vision_filter