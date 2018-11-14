#include "Camera.hpp"

#include <Geometry2d/Point.hpp>

void Camera::createConfiguration(Configuration* cfg) {
    MHKF_radius_cutoff = new ConfigDouble("VisionFilter/Camera/MHKF_radius_cutoff", 1);
    use_MHKF = new ConfigBool("VisionFilter/Camera/use_MHKF", true);
}

Camera::Camera(int cameraID)
    : cameraID(cameraID),
      kalmanRobotYellowlist(maxRobotJerseyNum),
      kalmnaRobotBlueList(maxRobotJerseyNum) {
}

void Camera::processBallBounce(std::vector<WorldRobot> yellowRobots,
                               std::vector<WorldRobot> blueRobots) {
    for (KalmanBall& b : kalmanBallList) {
        // TODO: Use the ball collision code
        Geometry2d::Point newVel;
        bool isCollision = Geometry2d::Point(0,0);

        if (isCollision) {
            b.setVel(newVel);
        }
    }
}

void Camera::updateWithFrame(RJ::Time calcTime,
                             std::vector<CameraBall> ballList,
                             std::vector<CameraRobot> yellowRobotList,
                             std::vector<CameraRobot> blueRobotList
                             WorldBall& previousWorldBall,
                             std::vector<WorldRobot>& previousYellowWorldRobots,
                             std::vector<WorldRobot>& previousBlueWorldRobots) {
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
            robot.predict(calctime);
        }
    }

    for (std::list<KalmanRobot>& robotList : kalmanRobotBlueList) {
        for (KalmanRobot& robot : robotList) {
            robot.predict(calcTime);
        }
    }
}

void Camera::updateBalls(RJ::Time calcTime, std::vector<CameraBall> ballList, WorldBall& previousWorldBall) {
    // Make sure there are actually balls in the measurement
    // and only predict if that's the case
    if (ballList.size() == 0) {
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

void Camera::updateBallsMHKF(RJ::Time calcTime, std::vector<CameraBall> ballList, WorldBall& previousWorldBall) {
    // If we have no existing filters, create a new one from average of everything
    // Easier than trying to figure out which ones are more than X meters away from each other
    // Only delays the filter collection by a camera frame or two
    if (kalmanBallList.size() == 0) {
        CameraBall avgBall = CameraBall::CombineBalls(ballList);
        kalmanBallList.emplace_back(cameraID, calcTime, avgBall, previousWorldBall);

        return;
    }

    // TODO: Try merging some of the kalman filters together

    // Create list of bools corresponding to whether we have used this ball
    // as measurement yet
    std::vector<bool> usedCameraBall(ballList.size(), false);

    // Which camera balls to apply to which kalman ball
    std::vector<std::list<CameraBall>> appliedBallsList(kalmanBallList.size());

    int kalmanBallIdx = 0;
    for (KalmanBall& kalmanBall : kalmanBallList) {
        std::list<CameraBall>& measurementBalls = appliedBallsList.at(kalmanBallIdx);

        int cameraBallIdx = 0;
        for (CameraBall& cameraBall : ballsList) {
            double dist = (kalmanBall.getPos() - cameraBall.getPos).mag();

            if (dist < *MHKF_radius_cutoff) {
                measurementBalls.push_back(cameraball);
                usedCameraball.at(cameraballIdx) = true;
            }
            cameraBallIdx++;
        }

        kalmanBallIdx++;
    }

    kalmnaBallIdx = 0;
    for (KalmanBall& kalmanBall : kalmanBallList) {
        std::list<CameraBall>& measurementBalls = appliedBallsList.at(kalmanBallIdx);

        // We had at least one measurement near this ball
        if (measurementBalls.size() > 0) {
            CameraBall avgBall = CameraBall::CombineBalls(measurementBalls);
            kalmanBall.PredictWithUpdate(calcTime, avgBall);

        // There aren't any measurements so just predict
        } else {
            kalmanBall.predict(calcTime);
        }
    }

    for (int i = 0; i < ballList.size(); i++) {
        CameraBall& cameraBall = ballList.at(i);
        bool wasUsed = usedCameraBall.at(i);

        kalmanBallList.emplace_back(cameraID, calcTime, cameraBall, previousWorldBall);
    }
}

void Camera::updateBallsAKF(RJ::Time calcTime, std::vector<CameraBall> ballList, WorldBall& previousWorldBall) {
    // If we have no existing filters, create a new one from average of everything
    if (kalmanBallList.size() == 0) {
        CameraBall avgBall = CameraBall::CombineBalls(ballList);
        kalmanBallList.emplace_back(cameraID, calcTime, avgBall, previousWorldBall);

        return;
    }

    // Average everything and add as measuremnet
    CameraBall avgBall = CameraBall::CombineBalls(ballList);
    // Kinda cheating, but we are only keeping a single element in the list
    kalmanBallsList.front().predictAndUpdate(calcTime, avgBall);
}

void Camera::updateRobots(RJ::Time calcTime, std::vector<CameraRobot> yellowRobotList,
                                             std::vector<CameraRobot> blueRobotList) {

}

void Camera::updateRobotsMHKF(RJ::Time calcTime, std::vector<CameraRobot> yellowRobotList,
                                                std::vector<CameraRobot> blueRobotList) {

}

void Camera::updateRobotsAKF(RJ::Time calcTime, std::vector<CameraRobot> yellowRobotList,
                                                std:vector<CameraRobot> blueRobotList) {

}

void Camera::removeInvalidBalls() {

}

void Camera::removeInvalidRobots() {

}