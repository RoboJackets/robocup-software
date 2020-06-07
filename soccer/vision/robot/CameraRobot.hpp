#pragma once

#include <geometry2d/point.h>
#include <geometry2d/pose.h>
#include <utils.h>

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
    CameraRobot(RJ::Time timeCaptured, geometry2d::Pose pose, int robotID)
        : timeCaptured(timeCaptured), pose(pose), robotID(robotID) {}

    /**
     * @return the time the detection was captured
     */
    RJ::Time getTimeCaptured() const;

    /**
     * @return the position of the measurement
     */
    geometry2d::Point getPos() const;

    /**
     * @return the heading of the measurement
     */
    double getTheta() const;

    /**
     * @return the robot ID of the measurement
     */
    int getRobotID() const;

    /**
     * @return the pose of the measurement
     */
    geometry2d::Pose getPose() const;

    /**
     * Combines all the robots in the list and returns a robot
     * with the average pos, time, and theta
     *
     * @param robots The list of robots to combine
     *
     * Note: All robots must have the same robotID
     */
    static CameraRobot CombineRobots(const std::list<CameraRobot>& robots);

private:
    RJ::Time timeCaptured;
    geometry2d::Pose pose;

    int robotID;
};
