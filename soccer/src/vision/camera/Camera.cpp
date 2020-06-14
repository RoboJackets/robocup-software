#include "Camera.hpp"

#include <Constants.hpp>
#include <Geometry2d/Point.hpp>

#include "vision/util/VisionFilterConfig.hpp"

REGISTER_CONFIGURABLE(Camera)

ConfigDouble* Camera::MHKF_radius_cutoff;
ConfigBool* Camera::use_MHKF;

ConfigInt* Camera::max_num_kalman_balls;
ConfigInt* Camera::max_num_kalman_robots;

void Camera::createConfiguration(Configuration* cfg) {
    MHKF_radius_cutoff = new ConfigDouble(cfg, "VisionFilter/Camera/MHKF_radius_cutoff", 0.5);
    use_MHKF = new ConfigBool(cfg, "VisionFilter/Camera/use_MHKF", true);

    max_num_kalman_balls = new ConfigInt(cfg, "VisionFilter/Camera/max_num_kalman_balls", 10);
    max_num_kalman_robots = new ConfigInt(cfg, "VisionFilter/Camera/max_num_kalman_robots", 10);
}

Camera::Camera() : isValid(false) {}

Camera::Camera(int cameraID)
    : isValid(true),
      cameraID(cameraID),
      kalmanRobotYellowList(Num_Shells),
      kalmanRobotBlueList(Num_Shells) {}

bool Camera::getIsValid() const {
    return isValid;
}

void Camera::processBallBounce(const std::vector<WorldRobot>& yellowRobots,
                               const std::vector<WorldRobot>& blueRobots) {
    for (KalmanBall& b : kalmanBallList) {
        Geometry2d::Point newVel;
        bool isCollision = BallBounce::CalcBallBounce(b, yellowRobots,
                                                      blueRobots, newVel);

        if (isCollision) {
            b.setVel(newVel);
        }
    }
}

void Camera::updateWithFrame(RJ::Time calcTime,
                             const std::vector<CameraBall>& ballList,
                             const std::vector<std::list<CameraRobot>>& yellowRobotList,
                             const std::vector<std::list<CameraRobot>>& blueRobotList,
                             const WorldBall& previousWorldBall,
                             const std::vector<WorldRobot>& previousYellowWorldRobots,
                             const std::vector<WorldRobot>& previousBlueWorldRobots) {
    // Prune list of balls and robots before doing anything
    removeInvalidBalls();
    removeInvalidRobots();

    updateBalls(calcTime, ballList, previousWorldBall);
    updateRobots(calcTime, yellowRobotList, blueRobotList,
                 previousYellowWorldRobots, previousBlueWorldRobots);
}

void Camera::updateWithoutFrame(RJ::Time calcTime) {
    removeInvalidBalls();
    removeInvalidRobots();

    for (KalmanBall& b : kalmanBallList) {
        b.predict(calcTime);
    }

    for (std::list<KalmanRobot>& robotList : kalmanRobotYellowList) {
        for (KalmanRobot& robot : robotList) {
            robot.predict(calcTime);
        }
    }

    for (std::list<KalmanRobot>& robotList : kalmanRobotBlueList) {
        for (KalmanRobot& robot : robotList) {
            robot.predict(calcTime);
        }
    }
}

void Camera::updateBalls(RJ::Time calcTime,
                         const std::vector<CameraBall>& ballList,
                         const WorldBall& previousWorldBall) {
    // Make sure there are actually balls in the measurement
    // and only predict if that's the case
    if (ballList.empty()) {
        for (KalmanBall& b : kalmanBallList) {
            b.predict(calcTime);
        }

        return;
    }

    // We have some balls, so choose which updater to use
    if (*use_MHKF) {
        updateBallsMHKF(calcTime, ballList, previousWorldBall);
    } else {
        updateBallsAKF(calcTime, ballList, previousWorldBall);
    }
}

void Camera::updateBallsMHKF(RJ::Time calcTime,
                             const std::vector<CameraBall>& ballList,
                             const WorldBall& previousWorldBall) {
    // If we have no existing filters, create a new one from average of everything
    // Easier than trying to figure out which ones are more than X meters away from each other
    // Only delays the filter collection by a camera frame or two
    if (kalmanBallList.empty()) {
        CameraBall avgBall = CameraBall::CombineBalls(ballList);
        kalmanBallList.emplace_back(cameraID, calcTime, avgBall, previousWorldBall);

        return;
    }

    // TODO: Try merging some of the kalman filters together

    // Create list of bools corresponding to whether we have used this ball
    // as measurement yet
    std::vector<bool> usedCameraBall(ballList.size(), false);

    // Which camera balls to apply to which kalman ball
    std::vector<std::vector<CameraBall>> appliedBallsList(kalmanBallList.size());

    // Figure out which measurements go with which kalman balls
    int kalmanBallIdx = 0;
    for (KalmanBall& kalmanBall : kalmanBallList) {
        std::vector<CameraBall>& measurementBalls = appliedBallsList.at(kalmanBallIdx);

        int cameraBallIdx = 0;
        for (const CameraBall& cameraBall : ballList) {
            double dist = (kalmanBall.getPos() - cameraBall.getPos()).mag();

            // Increase the distance of our cutoff by the velocity
            // This is so the ball doesn't move outside the kalman filter position radius when the ball
            // instantly stops (like in sim)
            if (dist < *MHKF_radius_cutoff + kalmanBall.getVel().mag()) {
                measurementBalls.push_back(cameraBall);
                usedCameraBall.at(cameraBallIdx) = true;
            }
            cameraBallIdx++;
        }

        kalmanBallIdx++;
    }

    // Apply the ball measurements to the kalman filters
    kalmanBallIdx = 0;
    for (KalmanBall& kalmanBall : kalmanBallList) {
        std::vector<CameraBall>& measurementBalls = appliedBallsList.at(kalmanBallIdx);

        // We had at least one measurement near this ball
        if (!measurementBalls.empty()) {
            CameraBall avgBall = CameraBall::CombineBalls(measurementBalls);
            kalmanBall.predictAndUpdate(calcTime, avgBall);

        // There aren't any measurements so just predict
        } else {
            kalmanBall.predict(calcTime);
        }
    }

    // Any balls not used, create a kalman ball at that position
    //
    // If there are two measurements which too far away from any
    // current kalman filter, the two measurements will be form
    // two individual kalman filters intead of a single one
    // with an update.
    // A slight delay in will most likely be seen in these cases
    for (int i = 0; i < ballList.size(); i++) {
        const CameraBall& cameraBall = ballList.at(i);
        bool wasUsed = usedCameraBall.at(i);

        if (!wasUsed && kalmanBallList.size() < *max_num_kalman_balls) {
            kalmanBallList.emplace_back(cameraID, calcTime, cameraBall, previousWorldBall);
        }
    }
}

void Camera::updateBallsAKF(RJ::Time calcTime,
                            const std::vector<CameraBall>& ballList,
                            const WorldBall& previousWorldBall) {
    // Average everything and add as measuremnet
    CameraBall avgBall = CameraBall::CombineBalls(ballList);

    // If we have no existing filters, create a new one from average of everything
    if (kalmanBallList.empty()) {
        kalmanBallList.emplace_back(cameraID, calcTime, avgBall, previousWorldBall);

        return;
    }

    // Kinda cheating, but we are only keeping a single element in the list
    kalmanBallList.front().predictAndUpdate(calcTime, avgBall);
}

void Camera::updateRobots(RJ::Time calcTime,
                          const std::vector<std::list<CameraRobot>>& yellowRobotList,
                          const std::vector<std::list<CameraRobot>>& blueRobotList,
                          const std::vector<WorldRobot>& previousYellowWorldRobots,
                          const std::vector<WorldRobot>& previousBlueWorldRobots) {

    for (int i = 0; i < Num_Shells; i++) {
        const std::list<CameraRobot>& singleYellowRobotList = yellowRobotList.at(i);
        const std::list<CameraRobot>& singleBlueRobotList = blueRobotList.at(i);

        // Make sure we actually have robots for the yellow team
        if (singleYellowRobotList.empty()) {
            for (KalmanRobot& robot : kalmanRobotYellowList.at(i)) {
                robot.predict(calcTime);
            }

        // If we do, do the fancy updates
        } else {
            if (*use_MHKF) {
                updateRobotsMHKF(calcTime,
                                 singleYellowRobotList,
                                 previousYellowWorldRobots.at(i),
                                 kalmanRobotYellowList.at(i));
            } else {
                updateRobotsAKF(calcTime,
                                singleYellowRobotList,
                                previousYellowWorldRobots.at(i),
                                kalmanRobotYellowList.at(i));
            }
        }

        // Make sure we actually have robots for the blue team
        if (singleBlueRobotList.empty()) {
            for (KalmanRobot& robot : kalmanRobotBlueList.at(i)) {
                robot.predict(calcTime);
            }

        // If we do, do the fancy updates
        } else {
            if (*use_MHKF) {
                updateRobotsMHKF(calcTime,
                                 singleBlueRobotList,
                                 previousBlueWorldRobots.at(i),
                                 kalmanRobotBlueList.at(i));
            } else {
                updateRobotsAKF(calcTime,
                                singleBlueRobotList,
                                previousBlueWorldRobots.at(i),
                                kalmanRobotBlueList.at(i));
            }
        }
    }
}

void Camera::updateRobotsMHKF(RJ::Time calcTime,
                              const std::list<CameraRobot>& singleRobotList,
                              const WorldRobot& previousWorldRobot,
                              std::list<KalmanRobot>& singleKalmanRobotList) {
    // If we have no existing filters, create a new one from average of everything
    // Easier than trying to figure out which ones are more than X meters away from each other
    // Only delays the filter collection by a camera frame or two
    if (singleKalmanRobotList.empty()) {
        CameraRobot avgRobot = CameraRobot::CombineRobots(singleRobotList);
        singleKalmanRobotList.emplace_back(cameraID, calcTime, avgRobot, previousWorldRobot);

        return;
    }

    // TODO: Merge some of the kalman filters together

    // Create list of bools corresponding to whether we have used this Robot
    // as measurement yet
    std::vector<bool> usedCameraRobot(singleRobotList.size(), false);

    // Which camera robots to apply to which kalman Robot
    std::vector<std::list<CameraRobot>> appliedRobotsList(singleKalmanRobotList.size());

    // Apply camera robots to different kalman filters based off a fixed distance
    // A single camera robots can go to multiple different kalman robots
    int kalmanRobotIdx = 0;
    for (KalmanRobot& kalmanRobot : singleKalmanRobotList) {
        std::list<CameraRobot>& measurementRobot = appliedRobotsList.at(kalmanRobotIdx);

        int cameraRobotIdx = 0;
        for (const CameraRobot& cameraRobot : singleRobotList) {
            double dist = (kalmanRobot.getPos() - cameraRobot.getPos()).mag();

            // Increase the distance of our cutoff by the velocity
            // This is so the robot doesn't move outside the kalman filter position radius when the robot
            // instantly stops (like in sim)
            if (dist < *MHKF_radius_cutoff + kalmanRobot.getVel().mag()) {
                measurementRobot.push_back(cameraRobot);
                usedCameraRobot.at(cameraRobotIdx) = true;
            }
            cameraRobotIdx++;
        }

        kalmanRobotIdx++;
    }

    // Predict and update the filters based on measurements
    kalmanRobotIdx = 0;
    for (KalmanRobot& kalmanRobot : singleKalmanRobotList) {
        std::list<CameraRobot>& measurementRobots = appliedRobotsList.at(kalmanRobotIdx);

        // We had at least one measurement near this Robot
        if (!measurementRobots.empty()) {
            CameraRobot avgRobot = CameraRobot::CombineRobots(measurementRobots);
            kalmanRobot.predictAndUpdate(calcTime, avgRobot);

        // There aren't any measurements so just predict
        } else {
            kalmanRobot.predict(calcTime);
        }
    }

    // Create kalman robots if one isn't near camera measurement
    int cameraRobotIdx = 0;
    for (const CameraRobot& cameraRobot : singleRobotList) {
        bool wasUsed = usedCameraRobot.at(cameraRobotIdx);

        if (!wasUsed && singleKalmanRobotList.size() < *max_num_kalman_robots) {
            singleKalmanRobotList.emplace_back(cameraID, calcTime, cameraRobot, previousWorldRobot);
        }
        
        cameraRobotIdx++;
    }
}

void Camera::updateRobotsAKF(RJ::Time calcTime,
                             const std::list<CameraRobot>& singleRobotList,
                             const WorldRobot& previousWorldRobot,
                             std::list<KalmanRobot>& singleKalmanRobotList) {

    // Average everything and add as measuremnet
    CameraRobot avgRobot = CameraRobot::CombineRobots(singleRobotList);

    // If we have no existing filters, create a new one from average of everything
    if (singleKalmanRobotList.empty()) {
        singleKalmanRobotList.emplace_back(cameraID, calcTime, avgRobot, previousWorldRobot);

        return;
    }

    // Kinda cheating, but we are only keeping a single element in the list
    singleKalmanRobotList.front().predictAndUpdate(calcTime, avgRobot);
}

void Camera::removeInvalidBalls() {
    // Remove all balls that are unhealthy
    kalmanBallList.remove_if([](KalmanBall& b) { return b.isUnhealthy(); } );
}

void Camera::removeInvalidRobots() {
    // Remove all the robots that are unhealthy
    for (std::list<KalmanRobot>& robotList : kalmanRobotBlueList) {
        robotList.remove_if([](KalmanRobot& r) {return r.isUnhealthy(); } );
    }

    for (std::list<KalmanRobot>& robotList : kalmanRobotYellowList) {
        robotList.remove_if([](KalmanRobot& r) {return r.isUnhealthy(); } );
    }
}

void Camera::predictAllRobots(RJ::Time calcTime, std::vector<std::list<KalmanRobot>>& robotListList) {
    for (std::list<KalmanRobot>& robotList : robotListList) {
        for (KalmanRobot& robot : robotList) {
            robot.predict(calcTime);
        }
    }
}

const std::list<KalmanBall>& Camera::getKalmanBalls() const {
    return kalmanBallList;
}

const std::vector<std::list<KalmanRobot>>& Camera::getKalmanRobotsYellow() const {
    return kalmanRobotYellowList;
}

const std::vector<std::list<KalmanRobot>>& Camera::getKalmanRobotsBlue() const {
    return kalmanRobotBlueList;
}