#include "CameraRobot.hpp"
#include <iostream>

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

CameraRobot CameraRobot::CombineRobots(std::list<CameraRobot> robots) {
    // Make sure we don't divide by zero due to some weird error
    if (robots.size() == 0) {
        std::cout << "CRITICAL ERROR: Number of robots to combine is zero" << std::endl;

        return CameraRobot(RJ::now(), Geometry2d::Point(0,0), 0, -1);
    }

    // Have to do the average like Ti + sum(Tn - Ti)/N
    // so that we aren't trying to add time_points. It's durations instead.
    RJ::Time initTime = robots.front().getTimeCaptured();
    RJ::Seconds timeAvg = RJ::Seconds(0);
    Geometry2d::Point posAvg = Geometry2d::Point(0,0);
    // Adding angles are done through conversion to rect coords then back to polar
    Geometry2d::Point thetaAvg = Geometry2d::Point(0,0);
    int robotID = -1;

    for (CameraRobot &cr : robots) {
        timeAvg += RJ::Seconds(cr.getTimeCaptured() - initTime);
        posAvg += cr.getPos();
        thetaAvg += Geometry2d::Point(cos(cr.getTheta()), sin(cr.getTheta()));
        robotID = cr.getRobotID(); // Shouldn't change besides the first iteration
    }

    timeAvg /= robots.size();
    posAvg /= robots.size();
    thetaAvg /= robots.size();

    return CameraRobot(initTime + timeAvg, posAvg, atan2(thetaAvg.y(), thetaAvg.x()), robotID);
}