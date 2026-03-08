#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/pose.hpp>
#include <boost/circular_buffer.hpp>
#include <rj_common/utils.hpp>
#include <rj_vision_filter/filter/kalman_filter3_d.hpp>
#include <rj_vision_filter/robot/camera_robot.hpp>

namespace vision_filter {
class WorldRobot;

/**
 * Filtered robot estimation for a single camera
 */
class KalmanRobot {
public:
    KalmanRobot(
        unsigned int camera_id,
        RJ::Time creation_time,
        CameraRobot init_measurement,
        const WorldRobot& previous_world_robot
    );

    /**
     * Checks previous_world_robot to see if it's valid
     *
     * @param camera_id ID of the camera this filter is applied to
     * @param creation_time Time this filter was created
     * @param init_measurement Initial robot measurement
     * @param previous_world_robot World robot from last frame (or invalid world
     * robot)
     */
    KalmanRobot(
        unsigned int camera_id,
        RJ::Time creation_time,
        CameraRobot init_measurement,
        const WorldRobot& previous_world_robot,
        const std::shared_ptr<rclcpp::Node>& vision_filter_node
    );

    /**
     * Predicts one time step forward
     *
     * @param current_time Current time of the prediction step
     */
    void predict(RJ::Time current_time);

    /**
     * Predicts one time step forward then triangulates toward the measurement
     *
     * @param current_time Current time of the prediction/update step
     * @param update_robot Robot measurement that we are using as feedback
     */
    void predict_and_update(RJ::Time current_time, CameraRobot update_robot);

    /**
     * Returns true when the filter hasn't been updated in a while and should be
     * deleted
     */
    [[nodiscard]] bool is_unhealthy() const;

    /**
     * @return The camera id this belongs to
     */
    [[nodiscard]] unsigned int get_camera_id() const;

    /**
     * @return This robot's id
     */
    [[nodiscard]] int get_robot_id() const;

    /**
     * @return How healthy this filter is. AKA How often it's been updated
     */
    [[nodiscard]] int get_health() const;

    /**
     * @return Best estimate of the linear position of the robot
     */
    [[nodiscard]] rj_geometry::Point get_pos() const;

    /**
     * @return Best estimate of the heading. Not bounded
     */
    [[nodiscard]] double get_theta() const;

    /**
     * @return Best estimate of the linear velocity of the robot
     */
    [[nodiscard]] rj_geometry::Point get_vel() const;

    /**
     * @return Best estimate of the angular velocity
     */
    [[nodiscard]] double get_omega() const;

    /**
     * @return Covariance in X and Y linear direction of the position of the
     * robot
     */
    [[nodiscard]] rj_geometry::Point get_pos_cov() const;

    /**
     * @return Covariance of theta of the robot
     */
    [[nodiscard]] double get_theta_cov() const;

    /**
     * @return Covariance in X and Y linear direction of the velocity of the
     * robot
     */
    [[nodiscard]] rj_geometry::Point get_vel_cov() const;

    /**
     * @return Covariance of omega of the robot
     */
    [[nodiscard]] double get_omega_cov() const;

    /**
     * @return List of previous camera robot measurements for kick detection
     */
    [[nodiscard]] const boost::circular_buffer<CameraRobot>& get_prev_measurements() const;

private:
    void initialize_parameters(const std::shared_ptr<rclcpp::Node>& vision_filter_node);
    bool update_parameters(const std::vector<rclcpp::Parameter>& params);

    RJ::Time last_update_time_;
    RJ::Time last_predict_time_;

    boost::circular_buffer<CameraRobot> previous_measurements_;

    KalmanFilter3D filter_;

    double previous_theta_;
    int unwrap_theta_ctr_;

    int robot_id_;

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
    double robot_initial_covariance_ = 100.0;
    // The ball process noise
    double robot_process_noise_ = 0.1;
    // The ball observation noise
    double robot_observation_noise_ = 2.0;
    // The maximum time in seconds that a filter can not be updated before it is removed
    double max_time_outside_vision_ = 0.2;

    // Handle to update the parameters when they change
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> param_cb_handle_;
};
}  // namespace vision_filter
