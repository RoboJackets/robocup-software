#pragma once

#include <array>
//TODO: Include Eigen

#include "Configuration.hpp"
#include "Robot.hpp"
#include "RobotFilter.hpp"

/**
 * @brief Implements a kalman filter ontop of the normal filter to predict the position
 * using encoders, target velocities, and cmaera readings. It will also predict out to the current time even if 
 * 
 * @details
 */
class OutRobotFilter {
public:
    static constexpr int Num_Cameras = 4;

    OurRobotFilter();

    void update(const std::array<RobotObservation, Num_Cameras>& obs,
                RobotPose* robot, RJ::Time currentTime, u_int32_t frameNumber);

    static void createConfiguration(Configuration* cfg);

private:
    enum FilterMode {
        BOTH_UPDATE = 0,
        ENCODER_UPDATE = 1,
    };

    /**
     * @brief Runs a single frame of the kalman filter given the mode
     * 
     * @param x_hat_k1_k1 Previous state estimate (in)
     * @param p_k1_k1 Previous covariance estimation (in)
     * @param x_hat_k_k Output state estimate after filter
     * @param p_k_k Output covaraince estimation
     * @param Mode Whether we are using both encoder and camera data or just the encoder
     *        This changes the "C" matrix as well as the noise matrix R
     * 
     * x_hat is a state vector containing 11 values
     *  X Position
     *  Y Position
     *  Heading Angle (Reference to X+)
     *  4 Encoder values
     *  4 Encoder sigma (I term on the robot sum)
     */
    void calculateOneFrame(Eigen::Vector& x_hat_k1_k1, Eigen::Vector& p_k1_k1, 
                           Eigen::Vector* x_hat_k_k, Eigen::Vector* p_k_k, 
                           FilterMode mode);


    /**
     * @brief Calculates the rotation matrix given a heading angle (Bot -> Global)
     */
    void calculateRotationMat(double heading, Eigen::Matrix* rot);
                           
    RobotFilter flit; // Base filter to clean up the output camera data

// Kalman filter matrices
    Eigen::Matrix F_k; // A
    Eigen::Matrix B_k; // B
    Eigen::Matrix H_k_both; // C with both camera and encoder output
    Eigen::Matrix H_k_enco; // C with only encoder output
    Eigen::Matrix Q_k; // Covariance of process noise
    Eigen::Matrix R_k_both; // Variance of observation noise for both camera and encoder
    Eigen::Matrix R_k_enco; // Variance of observation noise for only the encoder

    // Estimated State Vector for step k, given k's step information
    //   X Position
    //   Y Position
    //   Heading Angle (Reference to X+)
    //   4 Encoder values
    //   4 Encoder sigma (I term sum on the robot)
    Eigen::Vector x_hat_k_k;
    Eigen::Vector p_k_k; // Covariance of the current step

    // States at the last camera frame
    // We use this as the initial state to step forward using encoder inputs
    Eigen::Vector x_hat_k_k_camera;
    Eigen::vector p_k_k_camera;

// Constants / Transformations
    // Conversion factor between encoder ticks to wheel velocity (rads / sec)
    static constexpr double EncoderToWheelVel = 0; // 2*pi / t_soccer_period / (2048 * 3)

    // Conversion factor between camera output (mm) to position estimate (m)
    static constexpr double CameraToMeter = 1.0 / 1000.0; 

    // Estimated camera delay in number of frames
    static constexpr int Camera_Frame_Delay = 5;

    // TODO: Need BotToWheel and WheelToBot transformation


// Input History
    // List of all the past encoder readings between present and the last camera reading
    std::vector<Eigen::Vector> pastEncoders; // TODO: Look into better list structures

    // Past inputs
    std::vector<Eigen::Vector> pastU;

    // Past headings
    std::vector<double> pastHeading;


}