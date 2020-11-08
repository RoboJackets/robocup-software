#pragma once

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
    /**
     * Checks previous_world_robot to see if it's valid
     *
     * @param camera_id ID of the camera this filter is applied to
     * @param creation_time Time this filter was created
     * @param init_measurement Initial robot measurement
     * @param previous_world_robot World robot from last frame (or invalid world
     * robot)
     */
    KalmanRobot(unsigned int camera_id, RJ::Time creation_time,
                CameraRobot init_measurement,
                const WorldRobot& previous_world_robot);

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
    bool is_unhealthy() const;

    /**
     * @return The camera id this belongs to
     */
    unsigned int get_camera_id() const;

    /**
     * @return This robot's id
     */
    int get_robot_id() const;

    /**
     * @return How healthy this filter is. AKA How often it's been updated
     */
    int get_health() const;

    /**
     * @return Best estimate of the linear position of the robot
     */
    rj_geometry::Point get_pos() const;

    /**
     * @return Best estimate of the heading. Not bounded
     */
    double get_theta() const;

    /**
     * @return Best estimate of the linear velocity of the robot
     */
    rj_geometry::Point get_vel() const;

    /**
     * @return Best estimate of the angular velocity
     */
    double get_omega() const;

    /**
     * @return Covariance in X and Y linear direction of the position of the
     * robot
     */
    rj_geometry::Point get_pos_cov() const;

    /**
     * @return Covariance of theta of the robot
     */
    double get_theta_cov() const;

    /**
     * @return Covariance in X and Y linear direction of the velocity of the
     * robot
     */
    rj_geometry::Point get_vel_cov() const;

    /**
     * @return Covariance of omega of the robot
     */
    double get_omega_cov() const;

    /**
     * @return List of previous camera robot measurements for kick detection
     */
    const boost::circular_buffer<CameraRobot>& get_prev_measurements() const;

private:
    RJ::Time last_update_time_;
    RJ::Time last_predict_time_;

    boost::circular_buffer<CameraRobot> previous_measurements_;

    KalmanFilter3D filter_;

    double previous_theta_;
    int unwrap_theta_ctr_;
    int health_;

    int robot_id_;

    unsigned int camera_id_;
};
}  // namespace vision_filter
