#pragma once

#include <Geometry2d/Point.hpp>
#include <Utils.hpp>
#include <vector>

/**
 * Wrapper for the protobuf observation
 */
class CameraRobot {
public:
    /**
     * @param timeCaptured Time the frame was captured
     * @param pos Position of the robot observation
     * @param theta Heading of the robot observation
     * @param robotID ID of the robot
     */
    CameraRobot(RJ::Time timeCaptured, Geometry2d::Point pos, double theta, int robotID)
        : timeCaptured(timeCaptured), pos(pos), theta(theta), robotID(robotID) {}

    RJ::Time getTimeCaptured();
    Geometry2d::Point getPos();
    double getTheta();
    int getRobotID();

    /**
     * Combines all the robots in the list and returns a robot
     * with the average pos, time, and theta
     *
     * @param robots The list of robots to combine
     *
     * Note: All robots must have the same robotID
     */
    static CameraRobot CombineRobots(std::list<CameraRobot> robots);

private:
    RJ::Time timeCaptured;
    Geometry2d::Point pos;
    double theta;
    int robotID;
};