#pragma once

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
    KalmanBall(unsigned int camera_id, RJ::Time creation_time,
               CameraBall init_measurement, const WorldBall& previous_world_ball);

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
    bool is_unhealthy() const;

    /**
     * @return The camera id this belongs to
     */
    unsigned int get_camera_id() const;

    /**
     * @return How healthy this filter is. AKA How often it's been updated
     */
    int get_health() const;

    /**
     * @return Best estimate of the position of the ball
     */
    rj_geometry::Point get_pos() const;

    /**
     * @return Best estimate of the velocity of the ball
     */
    rj_geometry::Point get_vel() const;

    /**
     * @return Covariance in X and Y direction of the position of the ball
     */
    rj_geometry::Point get_pos_cov() const;

    /**
     * @return Covariance in X and Y direction of the velocity of the ball
     */
    rj_geometry::Point get_vel_cov() const;

    /**
     * @return List of previous camera ball measurements for kick
     * detection/estimation
     */
    const boost::circular_buffer<CameraBall>& get_prev_measurements() const;

    /**
     * @param new_vel new velocity to insert into the kalman filter
     *
     * Note: Only used to set the velocity when we think the ball will bounce
     * off another robot
     */
    void set_vel(rj_geometry::Point new_vel);

private:
    RJ::Time last_update_time_;
    RJ::Time last_predict_time_;

    // Keeps track of this for kick detection stuff
    boost::circular_buffer<CameraBall> previous_measurements_;

    KalmanFilter2D filter_;

    int health_;

    unsigned int camera_id_;
};
}  // namespace vision_filter
