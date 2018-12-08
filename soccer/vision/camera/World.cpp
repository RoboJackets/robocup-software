#include "World.hpp"

#include "vision/util/VisionFilterConfig.hpp"

// TODO: MaxCameraNum should be a config value
// TODO: Max num robots per team be a config value
World::World()
    : cameras(*VisionFilterConfig::max_num_cameras),
      robotsYellow(*VisionFilterConfig::num_robot_jerseys, WorldRobot()),
      robotsBlue(*VisionFilterConfig::num_robot_jerseys, WorldRobot()) {}

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
        std::vector<std::list<CameraRobot>> yellowTeam(*VisionFilterConfig::max_num_cameras);
        std::vector<std::list<CameraRobot>> blueTeam(*VisionFilterConfig::max_num_cameras);

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
    std::vector<std::list<KalmanRobot>> kalmanRobotsYellow(*VisionFilterConfig::num_robot_jerseys);
    std::vector<std::list<KalmanRobot>> kalmanRobotsBlue(*VisionFilterConfig::num_robot_jerseys);

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
        ball = WorldBall(kalmanBalls);
    }

    for (int i = 0; i < robotsYellow.size(); i++) {
        if (kalmanRobotsYellow.at(i).size() > 0) {
            robotsYellow.at(i) = WorldRobot(WorldRobot::Team::YELLOW, i, kalmanRobotsYellow.at(i));
        }
    }

    for (int i = 0; i < robotsBlue.size(); i++) {
        if (kalmanRobotsBlue.at(i).size() > 0) {
            robotsBlue.at(i) = WorldRobot(WorldRobot::Team::BLUE, i, kalmanRobotsBlue.at(i));
        }
    }
}

void World::detectKicks(RJ::Time calcTime) {
    // TODO: add the frame to the kick stuff
    // Run it and see what happens
}