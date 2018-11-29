#include "World.hpp"

// TODO: MaxCameraNum should be a config value
// TODO: Max num robots per team be a config value
World::World()
    : cameras(12), robotsYellow(12), robotsBlue(12) {}

void World::updateWithCameraFrame(RJ::Time calcTime, std::list<CameraFrame> newFrames) {
    calcBallBounce();

    // TODO: Use MaxCameraNum config
    std::vector<bool> cameraUpdated(12, false);

    // TODO: Take only the newest frame if 2 come in for the same camera

    for (CameraFrame& frame : newFrames) {
        // Make sure camera from frame is created, if not, make it
        if (!cameras.at(frame.getCameraID()).getIsValid()) {
            cameras.at(frame.getCameraID()) = Camera(frame.getCameraID());
        }

        // Take the non-sorted list from the frame and make a list for the cameras
        // TODO: Max num robots per team
        std::vector<std::list<CameraRobot>> yellowTeam(12);
        std::vector<std::list<CameraRobot>> blueTeam(12);

        for (CameraRobot& robot : frame.getCameraRobotsYellow()) {
            yellowTeam.at(robot.getRobotID()).push_back(robot);
        }

        for (CameraRobot& robot : frame.getCameraRobotsBlue()) {
            blueTeam.at(robot.getRobotID()).push_back(robot);
        }

        cameras.at(frame.getCameraID()).updateWithFrame(calcTime,
                                                        frame.getCameraBalls(),
                                                        yellowTeam,
                                                        blueTeam,
                                                        ball,
                                                        robotsYellow,
                                                        robotsBlue);

        cameraUpdated.at(frame.getCameraID()) = true;
    }

    for (int i = 0; i < cameras.size(); i++) {
        if (!cameraUpdated.at(i) && cameras.at(i).getIsValid()) {
            cameras.at(i).updateWithoutFrame(calcTime);
        }
    }

    updateWorldObjects();
    detectKicks(calcTime);
}

void World::updateWithoutCameraFrame(RJ::Time calcTime) {
    calcBallBounce();

    for (Camera& camera : cameras) {
        if (camera.getIsValid()) {
            camera.updateWithoutFrame(calcTime);
        }
    }

    updateWorldObjects();
    detectKicks(calcTime);
}

void World::calcBallBounce() {
    for (Camera& camera : cameras) {
        if (camera.getIsValid()) {
            camera.processBallBounce(robotsYellow, robotsBlue);
        }
    }
}

void World::updateWorldObjects() {
    // Fill robotsYellow/Blue with what robots we want and remove the rest
    ball = WorldBall();

    for (WorldRobot& robot : robotsYellow) {
        robot = WorldRobot();
    }

    for (WorldRobot& robot : robotsBlue) {
        robot = WorldRobot();
    }

    std::list<KalmanBall> kalmanBalls;
    // TODO: Max num robots
    std::vector<std::list<KalmanRobot>> kalmanRobotsYellow(12);
    std::vector<std::list<KalmanRobot>> kalmanRobotsBlue(12);

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

            for (int i = 0; i < cameraRobotsYellow.size(); i++) {
                if (cameraRobotsYellow.at(i).size() > 0) {
                    cameraRobotsYellow.at(i).sort([](KalmanRobot& a, KalmanRobot& b) -> bool {
                                                         return a.getHealth() > b.getHealth();
                                                     });

                    kalmanRobotsYellow.at(i).push_back(cameraRobotsYellow.at(0).front());
                }
            }

            for (int i = 0; i < cameraRobotsBlue.size(); i++) {
                if (cameraRobotsBlue.at(i).size() > 0) {
                    cameraRobotsBlue.at(i).sort([](KalmanRobot& a, KalmanRobot& b) -> bool {
                                                       return a.getHealth() > b.getHealth();
                                                   });

                    kalmanRobotsBlue.at(i).push_back(cameraRobotsBlue.at(0).front());
                }
            }
        }
    }

    // Only replace the invalid result if we have measurements on any camera
    if (kalmanBalls.size() > 0) {
        ball = WorldBall(kalmanBalls);
    }

    for (int i = 0; i < robotsYellow.size(); i++) {
        if (kalmanRobotsYellow.at(i).size() > 0) {
            robotsYellow.at(i) = WorldRobot(i, kalmanRobotsYellow.at(i));
        }
    }

    for (int i = 0; i < robotsBlue.size(); i++) {
        if (kalmanRobotsBlue.at(i).size() > 0) {
            robotsBlue.at(i) = WorldRobot(i, kalmanRobotsBlue.at(i));
        }
    }
}

void detectKicks(RJ::Time calcTime) {

}