#pragma once

#include <list>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rj_vision_filter/ball/world_ball.hpp>
#include <rj_vision_filter/camera/camera.hpp>
#include <rj_vision_filter/camera/camera_frame.hpp>
#include <rj_vision_filter/kick/detector/fast_kick_detector.hpp>
#include <rj_vision_filter/kick/detector/slow_kick_detector.hpp>
#include <rj_vision_filter/kick/kick_event.hpp>
#include <rj_vision_filter/robot/world_robot.hpp>

namespace vision_filter {
/**
 * Keeps list of all the cameras and sends camera data down to the correct
 * location
 */
class World {
public:
    /**
     * @brief Construct a new World Detector
     *
     */
    World(std::shared_ptr<rclcpp::Node> vision_filter_node);

    /**
     * Updates all the child cameras given a set of new camera frames
     *
     * @param calc_time Current iteration time
     * @param new_frames List of new frames from ssl vision
     * @param update_all Whether to update the cameras without vision measurements
     *
     * @note Call this OR update_without_camera_frame ONCE an iteration
     */
    void update_with_camera_frame(RJ::Time calc_time, const std::vector<CameraFrame>& new_frames,
                                  bool update_all);

    /**
     * Updates all the child cameras given a set of new camera frames
     *
     * @param calc_time Current iteration time
     * @param frame new frame for a single camera, from vision
     *
     * @note Call this once per camera per iteration
     */
    void update_single_camera(RJ::Time calc_time, const CameraFrame& frame);

    /**
     * Updates all the child cameras when there are no new camera frames
     *
     * @param calc_time Current iteration time
     *
     * @note Call this OR update_with_camera_frame ONCE an iteration
     */
    void update_without_camera_frame(RJ::Time calc_time);

    /**
     * @return Best estimate of the ball
     */
    [[nodiscard]] const WorldBall& get_world_ball() const;

    /**
     * @return List of the best estimates of all the yellow robots
     */
    [[nodiscard]] const std::vector<WorldRobot>& get_robots_yellow() const;

    /**
     * @return List of the best estimates of all the blue robots
     */
    [[nodiscard]] const std::vector<WorldRobot>& get_robots_blue() const;

    /**
     * @return The best kick estimate over the last few seconds
     */
    [[nodiscard]] const KickEvent& get_best_kick_estimate() const;

    /**
     * @return Timestamp of the latest vision receiver message that was used to
     * updated the states. Initialized with RJ::Time::min().
     */
    [[nodiscard]] RJ::Time last_update_time() const { return last_update_time_; }

private:
    /**
     * Does the ball bounce calculations for each of the child cameras
     */
    void calc_ball_bounce();

    /**
     * Fills the world objects with a mix of the best kalman filters from
     * each camera
     *
     * @param calc_time Current iteration time
     */
    void update_world_objects(RJ::Time calc_time);

    /**
     * Adds the latest estimate to the kick detectors and checks for kick
     *
     * @param calc_time Current iteration time
     */
    void detect_kicks(RJ::Time calc_time);

    /**
     * @brief Fetch parameter values from the ros runtime
     * 
     */
    void initialize_parameters();

    /**
     * @brief Called when the ros parameter runtime updates
     * 
     * @param params 
     * @return true 
     * @return false 
     */
    bool update_parameters(const std::vector<rclcpp::Parameter>& params);

    /**
     * @brief Timestamp of the latest vision receiver message that was used to
     * updated the states. Initialized with RJ::Time::min().
     */
    RJ::Time last_update_time_;

    std::vector<Camera> cameras_;

    WorldBall ball_;
    std::vector<WorldRobot> robots_yellow_;
    std::vector<WorldRobot> robots_blue_;

    FastKickDetector fast_kick_;
    SlowKickDetector slow_kick_;
    KickEvent best_kick_estimate_;

    // A shared pointer to the vision filter node (used for registering callbacks for parameters)
    std::shared_ptr<rclcpp::Node> vision_filter_node_;
    // A parameter callback to update ros parameters
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> param_cb_handle_;

    // The timeout for a same kick
    RJ::Seconds same_kick_timeout_ = {};
    // The timeout for a slow kick
    RJ::Seconds slow_kick_timeout_ = {};
    // The timeout for a fast kick
    RJ::Seconds fast_kick_timeout_ = {};
    // Multiplier to scale the weighted average coefficient to be nonlinear for the ball
    double ball_merger_power_ = 1.5;
    // Multipler to scale the weighted average coefficients to be nonlinear for the robots
    double robot_merger_power_ = 1.5;
};
}  // namespace vision_filter
