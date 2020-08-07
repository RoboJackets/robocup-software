#include <list>
#include <rj_vision_filter/ball/WorldBall.hpp>
#include <rj_vision_filter/camera/Camera.hpp>
#include <rj_vision_filter/camera/CameraFrame.hpp>
#include <rj_vision_filter/kick/KickEvent.hpp>
#include <rj_vision_filter/kick/detector/FastKickDetector.hpp>
#include <rj_vision_filter/kick/detector/SlowKickDetector.hpp>
#include <rj_vision_filter/robot/WorldRobot.hpp>
#include <vector>

namespace vision_filter {
/**
 * Keeps list of all the cameras and sends camera data down to the correct
 * location
 */
class World {
public:
    World();

    /**
     * Updates all the child cameras given a set of new camera frames
     *
     * @param calc_time Current iteration time
     * @param new_frames List of new frames from ssl vision
     *
     * @note Call this OR update_without_camera_frame ONCE an iteration
     */
    void update_with_camera_frame(RJ::Time calc_time,
                               const std::vector<CameraFrame>& new_frames);

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
    const WorldBall& get_world_ball() const;

    /**
     * @return List of the best estimates of all the yellow robots
     */
    const std::vector<WorldRobot>& get_robots_yellow() const;

    /**
     * @return List of the best estimates of all the blue robots
     */
    const std::vector<WorldRobot>& get_robots_blue() const;

    /**
     * @return The best kick estimate over the last few seconds
     */
    const KickEvent& get_best_kick_estimate() const;

    /**
     * @return Timestamp of the latest vision receiver message that was used to
     * updated the states. Initialized with RJ::Time::min().
     */
    [[nodiscard]] RJ::Time last_update_time() const {
        return last_update_time_;
    }

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
};
}  // namespace vision_filter
