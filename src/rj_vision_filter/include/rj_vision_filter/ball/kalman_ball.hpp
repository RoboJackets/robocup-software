#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rj_geometry/point.hpp>
#include <boost/circular_buffer.hpp>
#include <rj_vision_filter/ball/camera_ball.hpp>
#include <rj_vision_filter/filter/kalman_filter_2d.hpp>

namespace vision_filter {
class WorldBall;

/**
 * Filtered ball estimation for a single camera
 */
class KalmanBall {
public:
    /**
     * Checks the previous_world_ball to see if it's valid
     *
     * @param camera_id ID of the camera this filter belongs to
     * @param creation_time Time this filter is created
     * @param init_measurement Initial ball measurement we are creating the
     * filter at
     * @param previous_world_ball Previous prediction of ball location to
     * initialize the velocity smartly
     */
    KalmanBall(
        unsigned int camera_id,
        RJ::Time creation_time,
        CameraBall init_measurement,
        const WorldBall& previous_world_ball,
        const std::shared_ptr<rclcpp::Node>& vision_filter_node
    );

    KalmanBall(
        unsigned int camera_id,
        RJ::Time creation_time,
        CameraBall init_measurement,
        const WorldBall& previous_world_ball
    );

    /**
     * Predicts one time step forward
     *
     * @param current_time Time at the current frame
     *
     * @note Call either this OR predict_and_update once a frame
     */
    void predict(RJ::Time current_time);

    /**
     * Predicts one time step forward then triangulates towards the measurement
     *
     * @param current_time Current time of the prediction/update step
     * @param update_ball Ball measurement that we are using as feedback to the
     * filters
     *
     * @note Call either this OR predict once a frame
     */
    void predict_and_update(RJ::Time current_time, CameraBall update_ball);

    /**
     * @return Returns true when the filter hasn't been updated in a while etc
     * and should be deleted
     */
    [[nodiscard]] bool is_unhealthy() const;

    /**
     * @return The camera id this belongs to
     */
    [[nodiscard]] unsigned int get_camera_id() const;

    /**
     * @return How healthy this filter is. AKA How often it's been updated
     */
    [[nodiscard]] int get_health() const;

    /**
     * @return Best estimate of the position of the ball
     */
    [[nodiscard]] rj_geometry::Point get_pos() const;

    /**
     * @return Best estimate of the velocity of the ball
     */
    [[nodiscard]] rj_geometry::Point get_vel() const;

    /**
     * @return Covariance in X and Y direction of the position of the ball
     */
    [[nodiscard]] rj_geometry::Point get_pos_cov() const;

    /**
     * @return Covariance in X and Y direction of the velocity of the ball
     */
    [[nodiscard]] rj_geometry::Point get_vel_cov() const;

    /**
     * @return List of previous camera ball measurements for kick
     * detection/estimation
     */
    [[nodiscard]] const boost::circular_buffer<CameraBall>& get_prev_measurements() const;

    /**
     * @param new_vel new velocity to insert into the kalman filter
     *
     * Note: Only used to set the velocity when we think the ball will bounce
     * off another robot
     */
    void set_vel(rj_geometry::Point new_vel);

private:
    /**
     * @brief Initialize the stored ros parameters
     * 
     */
    void initialize_parameters(const std::shared_ptr<rclcpp::Node>& vision_filter_node);

    /**
     * @brief Update the stored ros parameters
     * 
     * @param params 
     * @return true 
     * @return false 
     */
    bool update_parameters(const std::vector<rclcpp::Parameter>& params);

    RJ::Time last_update_time_;
    RJ::Time last_predict_time_;

    // Keeps track of this for kick detection stuff
    boost::circular_buffer<CameraBall> previous_measurements_;

    KalmanFilter2D filter_;

    // The max health of the kalman filters
    int max_health_ = 20;
    // The minimum health of the kalman filters
    int min_health_ = 1;
    // How much to increment the health of the filter between measurements
    int health_increment_ = 2;
    // How much to decrement the health of the filter between measurements
    int health_decrement_ = 1;
    // Initial health of the kalman filters. (must be between min and max)
    int health_ = 2;

    unsigned int camera_id_;

    // The vision loop delta time (in seconds)
    double vision_loop_dt_ = 1.0 / 60.0;
    // The ball initial covariance
    double ball_initial_covariance_ = 100.0;
    // The ball process noise
    double ball_process_noise_ = 0.1;
    // The ball observation noise
    double ball_observation_noise_ = 2.0;
    // The maximum time in seconds that a filter can not be updated before it is removed
    double max_time_outside_vision_ = 0.2;

    // A handle to the parameter update callback
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> param_cb_handle_;

};
}  // namespace vision_filter
