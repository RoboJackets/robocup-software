#include <cmath>

#include <rj_vision_filter/filter/kalman_filter3_d.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {

DEFINE_NS_FLOAT64(kVisionFilterParamModule, robot, init_covariance, 100.0,
                  "Initial covariance of the filter. Controls how fast it gets "
                  "to the target.")
DEFINE_NS_FLOAT64(kVisionFilterParamModule, robot, process_noise, 0.5,
                  "Controls how quickly it reacts to changes in ball accelerations.")
DEFINE_NS_FLOAT64(kVisionFilterParamModule, robot, observation_noise, 2.0,
                  "Controls how much it trusts measurements from the camera.")
DEFINE_NS_FLOAT64(kVisionFilterParamModule, robot, orientation_scale, 1.0,
                  "Scales the covariance and noise to radians instead of "
                  "meters. Shouldn't matter too much, but it's here.")

KalmanFilter3D::KalmanFilter3D() : KalmanFilter(1, 1) {}

KalmanFilter3D::KalmanFilter3D(rj_geometry::Pose init_pose, rj_geometry::Twist init_twist)
    : KalmanFilter(6, 3) {
    // States are X pos, X vel, Y pos, Y vel, theta, omega
    x_k1_k1_ << init_pose.position().x(), init_twist.linear().x(), init_pose.position().y(),
        init_twist.linear().y(), init_pose.heading(), init_twist.angular();
    x_k_k1_ = x_k1_k1_;
    x_k_k_ = x_k1_k1_;

    // Initial covariance is usually extremely high to converge to the true
    // solution
    double p = robot::PARAM_init_covariance;
    double s = robot::PARAM_orientation_scale;
    // clang-format off
    P_k1_k1_ << p,   0,   0,   0,   0,   0,
                0,   p,   0,   0,   0,   0,
                0,   0,   p,   0,   0,   0,
                0,   0,   0,   p,   0,   0,
                0,   0,   0,   0, s*p,   0,
                0,   0,   0,   0,   0, s*p;
    // clang-format on
    P_k_k1_ = P_k1_k1_;
    P_k_k_ = P_k1_k1_;

    // State transition matrix (A)
    // Pos, velocity, theta integrator. Assume constant velocity
    double dt = PARAM_vision_loop_dt;
    // clang-format off
    F_k_ << 1, dt,  0,  0,  0,  0,
            0,  1,  0,  0,  0,  0,
            0,  0,  1, dt,  0,  0,
            0,  0,  0,  1,  0,  0,
            0,  0,  0,  0,  1, dt,
            0,  0,  0,  0,  0,  1;
    // clang-format on

    // Control transition matrix (B)
    // No inputs. Possible to update for our robots since we know our accel/vel
    // command
    // clang-format off
    B_k_ << 0,
            0,
            0,
            0,
            0,
            0;

    // Observation Matrix (C)
    // We can get positions
    H_k_ << 1, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 1, 0;
    // clang-format on

    // Covariance of process noise (how wrong A is)
    // Based on a guassian white noise w_k in x_dot = A*x + B*u + G*w
    // The noise can be propogated through the model resulting in a process
    // noise of the form
    //
    //  [1/3 T^3     1/2 T^2] * sigma^2
    //  [1/2 T^2           T]
    // Where sigma is the standard deviation of the process noise
    // the change in velocity over one time step should be around sqrt(T *
    // sigma^2) Note: T is the sample period Taken from Tiger's AutoRef. Most
    // likely found through integration of error through the state matrices See
    // https://en.wikipedia.org/wiki/Discretization#Discretization_of_process_noise
    p = robot::PARAM_process_noise;
    double sigma = sqrt(3.0 * p / dt) / dt;
    double dt3 = 1.0 / 3.0 * dt * dt * dt * sigma * sigma;
    double dt2 = 1.0 / 2.0 * dt * dt * sigma * sigma;
    double dt1 = dt * sigma * sigma;

    // clang-format off
    Q_k_ << dt3,   dt2,     0,     0,     0,     0,
            dt2,   dt1,     0,     0,     0,     0,
              0,     0,   dt3,   dt2,     0,     0,
              0,     0,   dt2,   dt1,     0,     0,
              0,     0,     0,     0, s*dt3, s*dt2,
              0,     0,     0,     0, s*dt2, s*dt1;
    // clang-format on

    // Covariance of observation noise (how wrong z_k is)
    double o = robot::PARAM_observation_noise;
    // clang-format off
    R_k_ << o,   0,   0,
            0,   o,   0,
            0,   0, s*o;
    // clang-format on
}

void KalmanFilter3D::predict_with_update(rj_geometry::Pose observation) {
    z_k_ << observation.position().x(), observation.position().y(), observation.heading();

    KalmanFilter::predict_with_update();
}

rj_geometry::Point KalmanFilter3D::get_pos() const {
    return rj_geometry::Point(x_k_k_(0), x_k_k_(2));
}

double KalmanFilter3D::get_theta() const { return x_k_k_(4); }

rj_geometry::Point KalmanFilter3D::get_vel() const {
    return rj_geometry::Point(x_k_k_(1), x_k_k_(3));
}

double KalmanFilter3D::get_omega() const { return x_k_k_(5); }

rj_geometry::Point KalmanFilter3D::get_pos_cov() const {
    return rj_geometry::Point(P_k_k_(0, 0), P_k_k_(2, 2));
}

double KalmanFilter3D::get_theta_cov() const { return P_k_k_(4, 4); }

rj_geometry::Point KalmanFilter3D::get_vel_cov() const {
    return rj_geometry::Point(P_k_k_(1, 1), P_k_k_(3, 3));
}

double KalmanFilter3D::get_omega_cov() const { return P_k_k_(5, 5); }
}  // namespace vision_filter