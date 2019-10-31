#include "KalmanFilter3D.hpp"

#include <cmath>

#include "vision/util/VisionFilterConfig.hpp"

REGISTER_CONFIGURABLE(KalmanFilter3D)

ConfigDouble* KalmanFilter3D::robot_init_covariance;
ConfigDouble* KalmanFilter3D::robot_process_noise;
ConfigDouble* KalmanFilter3D::robot_observation_noise;
ConfigDouble* KalmanFilter3D::orientation_scale;

void KalmanFilter3D::createConfiguration(Configuration* cfg) {
    robot_init_covariance = new ConfigDouble(cfg, "VisionFilter/Robot/init_covariance", 100);
    robot_process_noise = new ConfigDouble(cfg, "VisionFilter/Robot/process_noise", .1);
    robot_observation_noise = new ConfigDouble(cfg, "VisionFilter/Robot/observation_noise", 2.0);
    orientation_scale = new ConfigDouble(cfg, "VisionFilter/Robot/orientation_scale", 1);
}

KalmanFilter3D::KalmanFilter3D() : KalmanFilter(1,1) {}

KalmanFilter3D::KalmanFilter3D(Geometry2d::Pose initPose,
                               Geometry2d::Twist initTwist)
    : KalmanFilter(6, 3) {
    // States are X pos, X vel, Y pos, Y vel, theta, omega
    x_k1_k1 << initPose.position().x(), initTwist.linear().x(),
        initPose.position().y(), initTwist.linear().y(), initPose.heading(),
        initTwist.angular();
    x_k_k1 = x_k1_k1;
    x_k_k = x_k1_k1;

    // Initial covariance is usually extremely high to converge to the true solution
    double p = *robot_init_covariance;
    double s = *orientation_scale;
    P_k1_k1 << p,   0,   0,   0,   0,   0,
               0,   p,   0,   0,   0,   0,
               0,   0,   p,   0,   0,   0,
               0,   0,   0,   p,   0,   0,
               0,   0,   0,   0, s*p,   0,
               0,   0,   0,   0,   0, s*p;
    P_k_k1 = P_k1_k1;
    P_k_k = P_k1_k1;

    // State transition matrix (A)
    // Pos, velocity, theta integrator. Assume constant velocity
    double dt = *VisionFilterConfig::vision_loop_dt;
    F_k << 1, dt,  0,  0,  0,  0,
           0,  1,  0,  0,  0,  0,
           0,  0,  1, dt,  0,  0,
           0,  0,  0,  1,  0,  0,
           0,  0,  0,  0,  1, dt,
           0,  0,  0,  0,  0,  1;

    // Control transition matrix (B)
    // No inputs. Possible to update for our robots since we know our accel/vel command
    B_k << 0,
           0,
           0,
           0,
           0,
           0;

    // Observation Matrix (C)
    // We can get positions
    H_k << 1, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0,
           0, 0, 0, 0, 1, 0;

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
    p = *robot_process_noise;
    double sigma = sqrt(3.0 * p / dt) / dt;
    double dt3 = 1.0 / 3.0 * dt * dt * dt * sigma * sigma;
    double dt2 = 1.0 / 2.0 * dt * dt * sigma * sigma;
    double dt1 = dt * sigma * sigma;

    Q_k << dt3,   dt2,     0,     0,     0,     0,
           dt2,   dt1,     0,     0,     0,     0,
             0,     0,   dt3,   dt2,     0,     0,
             0,     0,   dt2,   dt1,     0,     0,
             0,     0,     0,     0, s*dt3, s*dt2,
             0,     0,     0,     0, s*dt2, s*dt1;

    // Covariance of observation noise (how wrong z_k is)
    double o = *robot_observation_noise;
    R_k << o,   0,   0,
           0,   o,   0,
           0,   0, s*o;
}

void KalmanFilter3D::predictWithUpdate(Geometry2d::Pose observation) {
    z_k << observation.position().x(), observation.position().y(),
        observation.heading();

    KalmanFilter::predictWithUpdate();
}


Geometry2d::Point KalmanFilter3D::getPos() const {
    return Geometry2d::Point(x_k_k(0), x_k_k(2));
}

double KalmanFilter3D::getTheta() const {
    return x_k_k(4);
}

Geometry2d::Point KalmanFilter3D::getVel() const {
    return Geometry2d::Point(x_k_k(1), x_k_k(3));
}

double KalmanFilter3D::getOmega() const {
    return x_k_k(5);
}

Geometry2d::Point KalmanFilter3D::getPosCov() const {
    return Geometry2d::Point(P_k_k(0,0), P_k_k(2,2));
}

double KalmanFilter3D::getThetaCov() const {
    return P_k_k(4,4);
}

Geometry2d::Point KalmanFilter3D::getVelCov() const {
    return Geometry2d::Point(P_k_k(1,1), P_k_k(3,3));
}

double KalmanFilter3D::getOmegaCov() const {
    return P_k_k(5,5);
}