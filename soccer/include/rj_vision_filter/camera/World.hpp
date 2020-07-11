#include <Configuration.hpp>
#include <list>
#include <rj_common/Utils.hpp>
#include <vector>

#include "vision/ball/WorldBall.hpp"
#include "vision/camera/Camera.hpp"
#include "vision/camera/CameraFrame.hpp"
#include "vision/kick/KickEvent.hpp"
#include "vision/kick/detector/FastKickDetector.hpp"
#include "vision/kick/detector/SlowKickDetector.hpp"
#include "vision/robot/WorldRobot.hpp"

/**
 * Keeps list of all the cameras and sends camera data down to the correct location
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
    void updateWithCameraFrame(RJ::Time calcTime, const std::vector<CameraFrame>& newFrames);

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

    static void createConfiguration(Configuration* cfg);

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

    std::vector<Camera> cameras;

    WorldBall ball;
    std::vector<WorldRobot> robotsYellow;
    std::vector<WorldRobot> robotsBlue;

    FastKickDetector fastKick;
    SlowKickDetector slowKick;
    KickEvent bestKickEstimate;

    // Only replace fast kick estimates when this much time has passed
    static ConfigDouble* fast_kick_timeout;
    // Only replace slow kick estimates when this much time has passed
    static ConfigDouble* slow_kick_timeout;
    // Only replace the fast kick estimate with a slow when the two times
    // are within this amount
    static ConfigDouble* same_kick_timeout;
};
