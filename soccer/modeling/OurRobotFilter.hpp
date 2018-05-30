#pragma once

#include <array>
#include <Eigen/Dense>

#include <common/Geometry2d/Util.hpp>
#include <rc-fshare/robot_model.hpp>
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
     * @param z_k Measured output from sensors for this frame (in)
     * @param u_k_partial Target robot velocities (body coordinate system) for this frame? (in) 
     * @param x_hat_k_k Output state estimate after filter (out)
     * @param p_k_k Output covaraince estimation (out)
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
void calculateOneFrame(Eigen::VectorXd& x_hat_k1_k1, Eigen::VectorXd& p_k1_k1, 
                       Eigen::VectorXd& z_k, Eigen::VectorXd& u_k_partial,
                       Eigen::VectorXd* x_hat_k_k, Eigen::VectorXd* p_k_k, 
                       FilterMode mode)


    /**
     * @brief Calculates the rotation matrix given a heading angle (Bot -> Global)
     */
    void calculateRotationMat(double heading, Eigen::Matrix* rot);
                           
    RobotFilter flit; // Base filter to clean up the output camera data

// Kalman filter matrices
    static constexpr StateSize = 11;

    Eigen::MatrixXd F_k; // A
    Eigen::MatrixXd B_k; // B
    Eigen::MatrixXd H_k_both; // C with both camera and encoder output
    Eigen::MatrixXd H_k_enco; // C with only encoder output
    Eigen::MatrixXd Q_k; // Covariance of process noise
    Eigen::MatrixXd R_k_both; // Variance of observation noise for both camera and encoder
    Eigen::MatrixXd R_k_enco; // Variance of observation noise for only the encoder

    // Estimated State Vector for step k, given k's step information
    //   X Position
    //   Y Position
    //   Heading Angle (Reference to X+)
    //   4 Encoder values
    //   4 Encoder sigma (I term sum on the robot)
    Eigen::VectorXd x_hat_k_k;
    Eigen::MatrixXd p_k_k; // Covariance of the current step

    // States at the last camera frame
    // We use this as the initial state to step forward using encoder inputs
    Eigen::VectorXd x_hat_k_k_camera;
    Eigen::VectorXd p_k_k_camera;

// Constants / Transformations
    // Conversion factor between encoder ticks (per sample period) to wheel velocity (rads / sec)
    //   2048 ticks/motor revolution, 3 motor revolution/wheel revolution,
    //   1/60 secs/sample period, 2pi to convert revolutions/second to rad/sec
    static constexpr double EncoderToWheelVel = 2.0*M_PI / (1.0 / 60.0) / (2048.0 * 3.0)

    // Conversion factor between camera output (mm) to position estimate (m)
    static constexpr double CameraToMeter = 1.0 / 1000.0; 

    // Estimated camera delay in number of frames
    static constexpr int Camera_Frame_Delay = 5;

    // TODO: Need BotToWheel and WheelToBot transformation
    auto BotToWheel = RobotModel::get().BotToWheel;
    auto WheelToBot = RobotModel::get().WheelToBot;

// Input History
    // List of all the past encoder readings between present and the last camera reading
    std::vector<Eigen::VectorXd> pastEncoders; // TODO: Look into better list structures

    // Past inputs
    std::vector<Eigen::VectorXd> pastU;
}