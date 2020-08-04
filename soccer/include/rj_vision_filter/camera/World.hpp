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
     * @param calcTime Current iteration time
     * @param newFrames List of new frames from ssl vision
     *
     * @note Call this OR updateWithoutCameraFrame ONCE an iteration
     */
    void updateWithCameraFrame(RJ::Time calcTime,
                               const std::vector<CameraFrame>& newFrames);

    /**
     * Updates all the child cameras when there are no new camera frames
     *
     * @param calcTime Current iteration time
     *
     * @note Call this OR updateWithCameraFrame ONCE an iteration
     */
    void updateWithoutCameraFrame(RJ::Time calcTime);

    /**
     * @return Best estimate of the ball
     */
    const WorldBall& getWorldBall() const;

    /**
     * @return List of the best estimates of all the yellow robots
     */
    const std::vector<WorldRobot>& getRobotsYellow() const;

    /**
     * @return List of the best estimates of all the blue robots
     */
    const std::vector<WorldRobot>& getRobotsBlue() const;

    /**
     * @return The best kick estimate over the last few seconds
     */
    const KickEvent& getBestKickEstimate() const;

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
    void calcBallBounce();

    /**
     * Fills the world objects with a mix of the best kalman filters from
     * each camera
     *
     * @param calcTime Current iteration time
     */
    void updateWorldObjects(RJ::Time calcTime);

    /**
     * Adds the latest estimate to the kick detectors and checks for kick
     *
     * @param calcTime Current iteration time
     */
    void detectKicks(RJ::Time calcTime);

    /**
     * @brief Timestamp of the latest vision receiver message that was used to
     * updated the states. Initialized with RJ::Time::min().
     */
    RJ::Time last_update_time_;

    std::vector<Camera> cameras;

    WorldBall ball;
    std::vector<WorldRobot> robotsYellow;
    std::vector<WorldRobot> robotsBlue;

    FastKickDetector fastKick;
    SlowKickDetector slowKick;
    KickEvent bestKickEstimate;
};
}  // namespace vision_filter
