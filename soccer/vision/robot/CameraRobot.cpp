#include "CameraRobot.hpp"

RJ::Time CameraRobot::getTimeCaptured() {
    return timeCaptured;
}

Geometry2d::Point CameraRobot::getPos() {
    return pos;
}

double CameraRobot::getTheta() {
    return theta;
}

int CameraRobot::getRobotID() {
    return robotID;
}

CameraRobot CameraRobot::CombineRobots(std::vector<CameraRobot> robots) {
    RJ::Time timeAvg = 0;
    Geometry2d::Point posAvg = Geometry2d::Point(0,0);
    double thetaAvg = 0.0;
    int robotID = -1; // Defaults to -1 if list is empty

    for (CameraRobot &cr : robots) {
        timeAvg += cr.getTimeCaptured();
        posAvg += cr.getPos();
        thetaAvg += cr.getTheta();
        robotID = cr.getRobotID(); // Shouldn't change besides the first iteration
    }

    timeAvg /= robots.size();
    posAvg /= robots.size();
    thetaAvg /= robots.size();

    return CameraRobot(timeAvg, posAvg, thetaAvg, robotID)
}