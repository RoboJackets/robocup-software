#include <rj_constants/constants.hpp>
#include <rj_vision_filter/camera/World.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {
DEFINE_NS_FLOAT64(kVisionFilterParamModule, kick::detector, fast_kick_timeout, 1.0,
                  "Only replace fast kick estimates when this much time has "
                  "passed. In seconds.")
DEFINE_NS_FLOAT64(kVisionFilterParamModule, kick::detector, slow_kick_timeout, 0.5,
                  "Only replace slow kick estimates when this much time has "
                  "passed. In seconds.")
DEFINE_NS_FLOAT64(kVisionFilterParamModule, kick::detector, same_kick_timeout, 0.5,
                  "Only replace fast kick estimate with a slow when the two "
                  "times are within this amount.")
using namespace kick::detector;

World::World()
    : last_update_time_{RJ::Time{RJ::Time::duration(0)}},
      cameras(PARAM_max_num_cameras),
      robotsYellow(Num_Shells, WorldRobot()),
      robotsBlue(Num_Shells, WorldRobot()) {}

void World::updateWithCameraFrame(RJ::Time calcTime, const std::vector<CameraFrame>& newFrames) {
    calcBallBounce();

    std::vector<bool> cameraUpdated(PARAM_max_num_cameras, false);

    // TODO: Take only the newest frame if 2 come in for the same camera

    for (const CameraFrame& frame : newFrames) {
        // Make sure camera from frame is created, if not, make it
        if (!cameras.at(frame.cameraID).getIsValid()) {
            cameras.at(frame.cameraID) = Camera(frame.cameraID);
        }

        // Take the non-sorted list from the frame and make a list for the
        // cameras
        std::vector<std::list<CameraRobot>> yellowTeam(Num_Shells);
        std::vector<std::list<CameraRobot>> blueTeam(Num_Shells);

        for (const CameraRobot& robot : frame.cameraRobotsYellow) {
            yellowTeam.at(robot.getRobotID()).push_back(robot);
        }

        for (const CameraRobot& robot : frame.cameraRobotsBlue) {
            blueTeam.at(robot.getRobotID()).push_back(robot);
        }

        cameras.at(frame.cameraID)
            .updateWithFrame(calcTime, frame.cameraBalls, yellowTeam, blueTeam, ball, robotsYellow,
                             robotsBlue);

        cameraUpdated.at(frame.cameraID) = true;

        // Update last_update_time_ with the latest tCapture.
        last_update_time_ = std::max(last_update_time_, frame.tCapture);
    }

    for (int i = 0; i < cameras.size(); i++) {
        if (!cameraUpdated.at(i) && cameras.at(i).getIsValid()) {
            cameras.at(i).updateWithoutFrame(calcTime);
        }
    }

    updateWorldObjects(calcTime);
    detectKicks(calcTime);
}

void World::updateWithoutCameraFrame(RJ::Time calcTime) {
    calcBallBounce();

    for (Camera& camera : cameras) {
        if (camera.getIsValid()) {
            camera.updateWithoutFrame(calcTime);
        }
    }

    updateWorldObjects(calcTime);
    detectKicks(calcTime);
}

void World::calcBallBounce() {
    for (Camera& camera : cameras) {
        if (camera.getIsValid()) {
            camera.processBallBounce(robotsYellow, robotsBlue);
        }
    }
}

void World::updateWorldObjects(RJ::Time calcTime) {
    // Fill robotsYellow/Blue with what robots we want and remove the rest
    ball = WorldBall();

    std::fill(robotsYellow.begin(), robotsYellow.end(), WorldRobot());
    std::fill(robotsBlue.begin(), robotsBlue.end(), WorldRobot());

    std::list<KalmanBall> kalmanBalls;
    std::vector<std::list<KalmanRobot>> kalmanRobotsYellow(Num_Shells);
    std::vector<std::list<KalmanRobot>> kalmanRobotsBlue(Num_Shells);

    // Take best kalman filter from every camera and combine them
    for (Camera& camera : cameras) {
        if (camera.getIsValid()) {
            std::list<KalmanBall> cameraBalls = camera.getKalmanBalls();
            std::vector<std::list<KalmanRobot>> cameraRobotsYellow = camera.getKalmanRobotsYellow();
            std::vector<std::list<KalmanRobot>> cameraRobotsBlue = camera.getKalmanRobotsBlue();

            if (!cameraBalls.empty()) {
                // Sort by health of the kalman filter
                cameraBalls.sort([](KalmanBall& a, KalmanBall& b) -> bool {
                    return a.getHealth() > b.getHealth();
                });

                kalmanBalls.push_back(cameraBalls.front());
            }

            // Take the best kalman filter from the camera
            for (int i = 0; i < cameraRobotsYellow.size(); i++) {
                if (!cameraRobotsYellow.at(i).empty()) {
                    cameraRobotsYellow.at(i).sort([](KalmanRobot& a, KalmanRobot& b) -> bool {
                        return a.getHealth() > b.getHealth();
                    });

                    kalmanRobotsYellow.at(i).push_back(cameraRobotsYellow.at(i).front());
                }
            }

            // Take the best kalman filter from the camera
            for (int i = 0; i < cameraRobotsBlue.size(); i++) {
                if (!cameraRobotsBlue.at(i).empty()) {
                    cameraRobotsBlue.at(i).sort([](KalmanRobot& a, KalmanRobot& b) -> bool {
                        return a.getHealth() > b.getHealth();
                    });

                    kalmanRobotsBlue.at(i).push_back(cameraRobotsBlue.at(i).front());
                }
            }
        }
    }

    // Only replace the invalid result if we have measurements on any camera
    if (!kalmanBalls.empty()) {
        ball = WorldBall(calcTime, kalmanBalls);
    }

    for (int i = 0; i < robotsYellow.size(); i++) {
        if (!kalmanRobotsYellow.at(i).empty()) {
            robotsYellow.at(i) =
                WorldRobot(calcTime, WorldRobot::Team::YELLOW, i, kalmanRobotsYellow.at(i));
        }
    }

    for (int i = 0; i < robotsBlue.size(); i++) {
        if (!kalmanRobotsBlue.at(i).empty()) {
            robotsBlue.at(i) =
                WorldRobot(calcTime, WorldRobot::Team::BLUE, i, kalmanRobotsBlue.at(i));
        }
    }
}

void World::detectKicks(RJ::Time calcTime) {
    KickEvent fastEvent;
    KickEvent slowEvent;

    bool isFastKick = fastKick.addRecord(calcTime, ball, robotsYellow, robotsBlue, fastEvent);
    bool isSlowKick = slowKick.addRecord(calcTime, ball, robotsYellow, robotsBlue, &slowEvent);

    // If there isn't a kick recorded already
    if (!bestKickEstimate.getIsValid()) {
        // Try to use the slow kick as it's a better estimate
        // but take fast kick if there isn't a corrsponding slow kick yet
        if (isSlowKick) {
            bestKickEstimate = slowEvent;
        } else if (isFastKick) {
            bestKickEstimate = fastEvent;
        }

        // There is a kick recorded already
    } else {
        const RJ::Seconds timeSinceBestEvent(bestKickEstimate.getKickTime() - calcTime);
        const RJ::Seconds sameKickTimeout(PARAM_same_kick_timeout);
        const RJ::Seconds slowKickTimeout(PARAM_slow_kick_timeout);
        const RJ::Seconds fastKickTimeout(PARAM_fast_kick_timeout);

        // Try using the slow kick if:
        //      - It refers to the current best kick event (and probably is a
        //      better estimate)
        //      - The old kick timed out and should be updated
        if (isSlowKick &&
            (timeSinceBestEvent < sameKickTimeout || timeSinceBestEvent > slowKickTimeout)) {
            bestKickEstimate = slowEvent;

            // Try using the fast kick if the old kick timed out
        } else if (isFastKick && timeSinceBestEvent > fastKickTimeout) {
            bestKickEstimate = fastEvent;

            // Remove the old kick if it's completely time out
        } else if (timeSinceBestEvent > slowKickTimeout + fastKickTimeout) {
            bestKickEstimate = KickEvent();
        }
    }
}

const WorldBall& World::getWorldBall() const { return ball; }

const std::vector<WorldRobot>& World::getRobotsYellow() const { return robotsYellow; }

const std::vector<WorldRobot>& World::getRobotsBlue() const { return robotsBlue; }

const KickEvent& World::getBestKickEstimate() const { return bestKickEstimate; }
}  // namespace vision_filter
