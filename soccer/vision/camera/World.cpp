#include "World.hpp"

#include <Constants.hpp>

#include "vision/util/VisionFilterConfig.hpp"

REGISTER_CONFIGURABLE(World)

ConfigDouble* World::fast_kick_timeout;
ConfigDouble* World::slow_kick_timeout;
ConfigDouble* World::same_kick_timeout;

void World::createConfiguration(Configuration* cfg) {
    // Note: slow kick timeout should be smaller than fast kick timeout
    fast_kick_timeout = new ConfigDouble(cfg, "VisionFilter/Kick/Detector/fast_kick_timeout", 1);
    slow_kick_timeout = new ConfigDouble(cfg, "VisionFilter/Kick/Detector/slow_kick_timeout", 0.5);
    same_kick_timeout = new ConfigDouble(cfg, "VisionFilter/Kick/Detector/same_kick_timeout", 0.05);
}

World::World()
    : cameras(*VisionFilterConfig::max_num_cameras),
      robotsYellow(Num_Shells, WorldRobot()),
      robotsBlue(Num_Shells, WorldRobot()) {}

void World::updateWithCameraFrame(RJ::Time calcTime, std::list<CameraFrame> newFrames) {
    calcBallBounce();

    std::vector<bool> cameraUpdated(*VisionFilterConfig::max_num_cameras, false);

    // TODO: Take only the newest frame if 2 come in for the same camera

    for (CameraFrame& frame : newFrames) {
        // Make sure camera from frame is created, if not, make it
        if (!cameras.at(frame.cameraID).getIsValid()) {
            cameras.at(frame.cameraID) = Camera(frame.cameraID);
        }

        // Take the non-sorted list from the frame and make a list for the cameras
        std::vector<std::list<CameraRobot>> yellowTeam(Num_Shells);
        std::vector<std::list<CameraRobot>> blueTeam(Num_Shells);

        for (CameraRobot& robot : frame.cameraRobotsYellow) {
            yellowTeam.at(robot.getRobotID()).push_back(robot);
        }

        for (CameraRobot& robot : frame.cameraRobotsBlue) {
            blueTeam.at(robot.getRobotID()).push_back(robot);
        }

        cameras.at(frame.cameraID).updateWithFrame(calcTime,
                                                   frame.cameraBalls,
                                                   yellowTeam,
                                                   blueTeam,
                                                   ball,
                                                   robotsYellow,
                                                   robotsBlue);

        cameraUpdated.at(frame.cameraID) = true;
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

    for (WorldRobot& robot : robotsYellow) {
        robot = WorldRobot();
    }

    for (WorldRobot& robot : robotsBlue) {
        robot = WorldRobot();
    }

    std::list<KalmanBall> kalmanBalls;
    std::vector<std::list<KalmanRobot>> kalmanRobotsYellow(Num_Shells);
    std::vector<std::list<KalmanRobot>> kalmanRobotsBlue(Num_Shells);

    // Take best kalman filter from every camera and combine them
    for (Camera& camera : cameras) {
        if (camera.getIsValid()) {
            std::list<KalmanBall> cameraBalls = camera.getKalmanBalls();
            std::vector<std::list<KalmanRobot>> cameraRobotsYellow = camera.getKalmanRobotsYellow();
            std::vector<std::list<KalmanRobot>> cameraRobotsBlue = camera.getKalmanRobotsBlue();

            if (cameraBalls.size() > 0) {
                // Sort by health of the kalman filter
                cameraBalls.sort([](KalmanBall& a, KalmanBall& b) -> bool {
                                    return a.getHealth() > b.getHealth();
                                });

                kalmanBalls.push_back(cameraBalls.front());
            }

            // Take the best kalman filter from the camera
            for (int i = 0; i < cameraRobotsYellow.size(); i++) {
                if (cameraRobotsYellow.at(i).size() > 0) {
                    cameraRobotsYellow.at(i).sort([](KalmanRobot& a, KalmanRobot& b) -> bool {
                                                         return a.getHealth() > b.getHealth();
                                                     });

                    kalmanRobotsYellow.at(i).push_back(cameraRobotsYellow.at(i).front());
                }
            }

            // Take the best kalman filter from the camera
            for (int i = 0; i < cameraRobotsBlue.size(); i++) {
                if (cameraRobotsBlue.at(i).size() > 0) {
                    cameraRobotsBlue.at(i).sort([](KalmanRobot& a, KalmanRobot& b) -> bool {
                                                       return a.getHealth() > b.getHealth();
                                                   });

                    kalmanRobotsBlue.at(i).push_back(cameraRobotsBlue.at(i).front());
                }
            }
        }
    }

    // Only replace the invalid result if we have measurements on any camera
    if (kalmanBalls.size() > 0) {
        ball = WorldBall(calcTime, kalmanBalls);
    }

    for (int i = 0; i < robotsYellow.size(); i++) {
        if (kalmanRobotsYellow.at(i).size() > 0) {
            robotsYellow.at(i) = WorldRobot(calcTime, WorldRobot::Team::YELLOW, i, kalmanRobotsYellow.at(i));
        }
    }

    for (int i = 0; i < robotsBlue.size(); i++) {
        if (kalmanRobotsBlue.at(i).size() > 0) {
            robotsBlue.at(i) = WorldRobot(calcTime, WorldRobot::Team::BLUE, i, kalmanRobotsBlue.at(i));
        }
    }
}

void World::detectKicks(RJ::Time calcTime) {
    KickEvent fastEvent;
    KickEvent slowEvent;

    bool isFastKick = fastKick.addRecord(calcTime, ball, robotsYellow, robotsBlue, fastEvent);
    bool isSlowKick = slowKick.addRecord(calcTime, ball, robotsYellow, robotsBlue, slowEvent);

    // Try to use slow kick whenever it's possible
    if (isSlowKick) {
        // Any detected kick?
        if (bestKickEstimate.getIsValid()) {
            // If they are the same event, replace with the slow estimate
            if (RJ::Seconds(bestKickEstimate.kickTime - calcTime) < RJ::Seconds(*same_kick_timeout)) {
                bestKickEstimate = slowEvent;

            // If it is probably a different kick
            } else if ((RJ::Seconds(bestKickEstimate.kickTime - calcTime) > RJ::Seconds(*slow_kick_timeout))) {
                bestKickEstimate = slowEvent;
            }
        } else {
            bestKickEstimate = slowEvent;
        }
    } else if (isFastKick) {
        // Any detected kick?
        if (!bestKickEstimate.getIsValid()) {
            // If there has been an even longer timeout between kick estimates
            if ((RJ::Seconds(bestKickEstimate.kickTime - calcTime) > RJ::Seconds(*fast_kick_timeout))) {
                bestKickEstimate = fastEvent;
            }
        } else {
            bestKickEstimate = fastEvent;
        }
    }

    // If we haven't had a kick in a while, reset out kick estimate
    if (bestKickEstimate.getIsValid() && 
        RJ::Seconds(bestKickEstimate.kickTime - calcTime) > RJ::Seconds(*slow_kick_timeout + *fast_kick_timeout)) {
        bestKickEstimate = KickEvent();
    }
}

WorldBall World::getWorldBall() {
    return ball;
}

std::vector<WorldRobot> World::getRobotsYellow() {
    return robotsYellow;
}

std::vector<WorldRobot> World::getRobotsBlue() {
    return robotsBlue;
}

KickEvent World::getBestKickEstimate() {
    return bestKickEstimate;
}
