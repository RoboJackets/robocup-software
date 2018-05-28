#pragma once

#include <Robot.hpp>
#include <array>
//TODO: Include Eigen

#include <Configuration.hpp>

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
    
// Kalman filter matrices
    Eigen::Matrix F_k; // A
    Eigen::Matrix B_k; // B
    Eigen::Matrix H_k_both; // C with both camera and encoder output
    Eigen::Matrix F_k_enco; // C with only encoder output
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

    // Contains the rotation from robot to global frame (Updated every frame)
    Eigen::Matrix rotation;

    // TODO: Need BotToWheel and WheelToBot transformation


// Input History
    // List of all the past encoder readings between present and the last camera reading
    std::vector<Eigen::Vector> pastEncoders; // TODO: Look into better list structures

    // Past inputs
    std::vector<Eigen::Vector> pastU;

    // Past headings
    std::vector<double> pastHeading;


}