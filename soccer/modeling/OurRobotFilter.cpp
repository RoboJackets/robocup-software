#include "OurRobotFilter.hpp"

void OurRobotFilter::createConfiguration(Configuration* cfg) {
    // TODO: Add all the config variables
    // Thinking just camera_frame_delay for right now
}

OurRobotFilter::OurRobotFilter() : filt(),
    F_k(StateSize, StateSize). B_k(StateSize, StateSize),
    H_k_both(7, StateSize), H_k_enco(4, StateSize), Q_k(StateSize, StateSize),
    R_k_both(7, StateSize), R_k_enco(4, StateSize),
    x_hat_k_k(StateSize), p_k_k(StateSize, StateSize),
    x_hat_k_k_camera(StateSize), p_k_k_camera(StateSize, StateSize) {
    
    // TODO: This feels terrible, maybe find some better way to do this
    //       Load from a .mat? Auto-gen C++ function from matlab?
    F_k <<      1.0000,        0,        0,   0.0001,   0.0001,  -0.0001,  -0.0001,        0,        0,        0,        0,
                     0,   1.0000,        0,  -0.0002,   0.0002,   0.0002,  -0.0002,        0,        0,        0,        0,
                     0,        0,   1.0000,  -0.0013,  -0.0009,  -0.0009,  -0.0013,        0,        0,        0,        0,
                     0,        0,        0,   0.5979,   0.0897,  -0.2399,   0.1173,        0,        0,        0,        0,
                     0,        0,        0,   0.0765,   0.5591,   0.1579,  -0.2442,        0,        0,        0,        0,
                     0,        0,        0,  -0.2408,   0.1453,   0.5553,   0.0898,        0,        0,        0,        0,
                     0,        0,        0,   0.1309,  -0.2452,   0.0766,   0.6028,        0,        0,        0,        0,
                     0,        0,        0,   0.0126,   0.0013,  -0.0027,   0.0015,   1.0000,        0,        0,        0,
                     0,        0,        0,   0.0011,   0.0122,   0.0019,  -0.0027,        0,   1.0000,        0,        0,
                     0,        0,        0,  -0.0027,   0.0018,   0.0122,   0.0013,        0,        0,   1.0000,        0,
                     0,        0,        0,   0.0016,  -0.0027,   0.0011,   0.0127,        0,        0,        0,   1.0000;
     
    B_k <<           0,        0,        0,        0,        0,        0,        0,        0,        0,        0,        0,
                     0,        0,        0,        0,        0,        0,        0,        0,        0,        0,        0,
                     0,        0,        0,        0,        0,        0,        0,        0,        0,        0,        0,
                     0,        0,        0,        0,        0,        0,        0,        0,        0,        0,        0,
                     0,        0,        0,        0,        0,        0,        0,        0,        0,        0,        0,
                     0,        0,        0,        0,        0,        0,        0,        0,        0,        0,        0,
                     0,        0,        0,        0,        0,        0,        0,        0,        0,        0,        0,
                     0,        0,        0,  -0.0167,        0,        0,        0,        0,        0,        0,        0,
                     0,        0,        0,        0,  -0.0167,        0,        0,        0,        0,        0,        0,
                     0,        0,        0,        0,        0,  -0.0167,        0,        0,        0,        0,        0,
                     0,        0,        0,        0,        0,        0,  -0.0167,        0,        0,        0,        0;
     
    H_k_both <<      1,        0,        0,        0,        0,        0,        0,        0,        0,        0,        0,
                     0,        1,        0,        0,        0,        0,        0,        0,        0,        0,        0,
                     0,        0,        1,        0,        0,        0,        0,        0,        0,        0,        0,
                     0,        0,        0,        1,        0,        0,        0,        0,        0,        0,        0,
                     0,        0,        0,        0,        1,        0,        0,        0,        0,        0,        0,
                     0,        0,        0,        0,        0,        1,        0,        0,        0,        0,        0,
                     0,        0,        0,        0,        0,        0,        1,        0,        0,        0,        0;
     
    H_k_enco <<      0,        0,        0,        1,        0,        0,        0,        0,        0,        0,        0,
                     0,        0,        0,        0,        1,        0,        0,        0,        0,        0,        0,
                     0,        0,        0,        0,        0,        1,        0,        0,        0,        0,        0,
                     0,        0,        0,        0,        0,        0,        1,        0,        0,        0,        0;

    R_k_both << 0.0100,        0,        0,        0,        0,        0,        0,
                     0,   0.0100,        0,        0,        0,        0,        0,
                     0,        0,   0.0100,        0,        0,        0,        0,
                     0,        0,        0,   0.0001,        0,        0,        0,
                     0,        0,        0,        0,   0.0001,        0,        0,
                     0,        0,        0,        0,        0,   0.0001,        0,
                     0,        0,        0,        0,        0,        0,   0.0001;

    R_k_enco << 0.0001,        0,        0,        0,
                     0,   0.0001,        0,        0,
                     0,        0,   0.0001,        0,
                     0,        0,        0,   0.0001;

    x_hat_k_k = Eigen::VectorXd::Zero(StateSize);
    p_k_k     = Eigen::MatrixXd::Zero(StateSize, StateSize);
    x_hat_k_k_camera = Eigen::VectorXd::Zero(StateSize);
    p_k_k_camera     = Eigen::VectorXd::Zero(StateSize, StateSize);
}

void OurRobotFilter::update(const std::array<RobotObservation, Num_Cameras>& obs, RobotPose* robot, RJ::Time currentTime, u_int32_t frameNumber) {
    filt.update(obs, robot, currentTime, frameNumber);

    // Get sensor output
    // Get robot commanded velocities

    // Calculate 1 frame for the new camera
    // Calculate N frames for the encoders
}

void OurRobotFilter::calculateOneFrame(Eigen::VectorXd& x_hat_k1_k1, Eigen::VectorXd& p_k1_k1, 
                                       Eigen::VectorXd& z_k, Eigen::VectorXd& u_k_partial,
                                       Eigen::VectorXd* x_hat_k_k, Eigen::VectorXd* p_k_k, 
                                       FilterMode mode) {
    unsigned int OutputSize;
    Eigen::MatrixXd* H_k;
    Eigen::MatrixXd* R_k;

    if (mode == FilterMode::BOTH_UPDATE) {
        OutputSize = 7; // Encoder(4) + Camera(3)
        H_k = &H_k_both;
        R_k = &R_k_both;
    } else { // mode == FilterMode::ENCODER_UPDATE
        OutputSize = 4; // Encoder(4)
        H_k = &H_k_enco;
        R_k = &R_k_enco;
    }

    Eigen::MatrixXd BodyToGlobal(3, 3);
    Eigen::MatrixXd F_k_rot(StateSize, StateSize);
    Eigen::VectorXd u_k(StateSize);
    Eigen::VectorXd x_hat_k_k1(StateSize);
    Eigen::MatrixXd p_k_k1(StateSize, StateSize);
    Eigen::VectorXd y_tilda_k(OutputSize);
    Eigen::VectorXd y_tilda_k_k(OutputSize);
    Eigen::MatrixXd s_k(OutputSize, OutputSize);
    Eigen::MatrixXd k_k(StateSize, OutputSize);
    Eigen::MatrixXd I(StateSize, StateSize);

    u_k = Eigen::VectorXd::Zero(StateSize);
    u_k.block(3, 6) = u_k_partial; // TODO: Make sure this works with vectors
    I = Eigen::MatrixXd::Identity(StateSize, StateSize);

    // Rotate F_k
    calculateRotationMat(x_hat_k_k1(2), &BodyToGlobal);
    // F_k_rot(1:3, 4:end) = BodyToGlobal * F_k(1:3, 4:end)

    x_hat_k_k1 = F_k_rot * x_hat_k1_k1 + B_k * u_k;
    p_k_k1 = F_k_rot * p_k1_k1 * F_k_rot.transpose() + Q_l;

    y_tilda_k = z_k - *H_k * x_hat_k_k1;
    s_k = *R_k + *H_k * p_k_k1 * (*H_k).transpose();

    k_k = p_k_k1 * *H_k * s_k.inverse();
    
    x_hat_k_k = x_hat_k_k1 + k_k * y_tilda_k;
    p_k_k = (I - k_k * *H_k) * p_k_k1 * (I - k_k * *H_k).transpose() + k_k * *R_k * k_k.transpose();
    y_tilda_k_k = z_k - *H_k * x_hat_k_k;
}

void OurRobotFilter::calculateRotationMat(double heading, Eigen::Matrix* rot) {d
    rot << cos(heading), -sin(heading), 0,
           sin(heading),  cos(heading), 0,
                      0,             0, 1;
}