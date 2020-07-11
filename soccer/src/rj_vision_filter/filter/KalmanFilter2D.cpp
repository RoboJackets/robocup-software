#include <cmath>
#include <rj_vision_filter/filter/KalmanFilter2D.hpp>
#include <rj_vision_filter/util/VisionFilterConfig.hpp>

REGISTER_CONFIGURABLE(KalmanFilter2D)

ConfigDouble* KalmanFilter2D::ball_init_covariance;
ConfigDouble* KalmanFilter2D::ball_process_noise;
ConfigDouble* KalmanFilter2D::ball_observation_noise;

void KalmanFilter2D::createConfiguration(Configuration* cfg) {
    ball_init_covariance = new ConfigDouble(cfg, "VisionFilter/Ball/init_covariance", 100);
    ball_process_noise = new ConfigDouble(cfg, "VisionFilter/Ball/process_noise", .1);
    ball_observation_noise = new ConfigDouble(cfg, "VisionFilter/Ball/observation_noise", 2.0);
}

KalmanFilter2D::KalmanFilter2D() : KalmanFilter(1,1) {}

KalmanFilter2D::KalmanFilter2D(Geometry2d::Point initPos, Geometry2d::Point initVel)
    : KalmanFilter(4, 2) {

    // States are X pos, X vel, Y pos, Y vel
    x_k1_k1 << initPos.x(),
               initVel.x(),
               initPos.y(),
               initVel.y();
    x_k_k1 = x_k1_k1;
    x_k_k = x_k1_k1;

    // Initial covariance is usually extremely high to converge to the true solution
    double p = *ball_init_covariance;
    P_k1_k1 << p, 0, 0, 0,
               0, p, 0, 0,
               0, 0, p, 0,
               0, 0, 0, p;
    P_k_k1 = P_k1_k1;
    P_k_k = P_k1_k1;

    // State transition matrix (A)
    // Pos, velocity integrator. Assume constant velocity
    double dt = *VisionFilterConfig::vision_loop_dt; // TODO: Take config values
    F_k << 1, dt,  0,  0,
           0,  1,  0,  0,
           0,  0,  1, dt,
           0,  0,  0,  1;

    // Control transition matrix (B)
    // No inputs
    B_k << 0,
           0,
           0,
           0;

    // Observation Matrix (C)
    // We can get positions
    H_k << 1, 0, 0, 0,
           0, 0, 1, 0;

    // Covariance of process noise (how wrong A is)
    // Based on a guassian white noise w_k in x_dot = A*x + B*u + G*w
    // The noise can be propogated through the model resulting in a process noise
    // of the form
    //
    //  [1/3 T^3     1/2 T^2] * sigma^2
    //  [1/2 T^2           T]
    // Where sigma is the standard deviation of the process noise
    // the change in velocity over one time step should be around sqrt(T * sigma^2)
    // Note: T is the sample period
    // Taken from Tiger's AutoRef. Most likely found through integration of error through the
    // state matrices
    // See https://en.wikipedia.org/wiki/Discretization#Discretization_of_process_noise
    p = *ball_process_noise;
    double sigma = sqrt(3.0 * p / dt) / dt;
    double dt3 = 1.0 / 3.0 * dt * dt * dt * sigma * sigma;
    double dt2 = 1.0 / 2.0 * dt * dt * sigma * sigma;
    double dt1 = dt * sigma * sigma;

    Q_k << dt3, dt2,   0,   0,
           dt2, dt1,   0,   0,
             0,   0, dt3, dt2,
             0,   0, dt2, dt1;

    // Covariance of observation noise (how wrong z_k is)
    double o = *ball_observation_noise;
    R_k << o, 0,
           0, o;
}

void KalmanFilter2D::predictWithUpdate(Geometry2d::Point observation) {
    z_k << observation.x(),
           observation.y();

    KalmanFilter::predictWithUpdate();
}

Geometry2d::Point KalmanFilter2D::getPos() const {
    return Geometry2d::Point(x_k_k(0), x_k_k(2));
}

Geometry2d::Point KalmanFilter2D::getVel() const {
    return Geometry2d::Point(x_k_k(1), x_k_k(3));
}

Geometry2d::Point KalmanFilter2D::getPosCov() const {
    return Geometry2d::Point(P_k_k(0,0), P_k_k(2,2));
}

Geometry2d::Point KalmanFilter2D::getVelCov() const {
    return Geometry2d::Point(P_k_k(1,1), P_k_k(3,3));
}

void KalmanFilter2D::setVel(Geometry2d::Point newVel) {
    x_k_k(1) = newVel.x();
    x_k_k(3) = newVel.y();
}