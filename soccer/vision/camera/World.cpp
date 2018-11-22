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
        std::vector<std::list<CameraRobot> yellowTeam(12);
        std::vector<std::list<CameraRobot> blueTeam(12);

        for (CameraRobot& robot : frame.getCameraRobotsYellow()) {
            yellowTeam.at(robot.getRobotID()).push_back(robot);
        }

        for (CameraRobot& robot : frame.getCameraRobotsBlue()) {
            blueTeam.at(robot.getRobotID()).push_back(robot);
        }

        cameras.at(frame.getCameraID).updateWithFrame(calcTime,
                                                      frame.getCameraBalls,
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
    calcBallbounce();

    for (Camera& camera : cameras) {
        if (camera.getIsValid()) {
            camera.updateWithoutCameraFrame(calcTime);
        }
    }

    updateWorldObjects();
    detectKicks(calctime);
}

void World::calcBallBounce() {

}

void World::updateWorldObjects() {
// Fill robotsYellow/Blue with what robobts we want and remove the rest
    
}

void detectKicks(RJ::Time calcTime) {

}